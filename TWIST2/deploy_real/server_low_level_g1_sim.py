import argparse
import json
import time
import numpy as np
import redis
import mujoco
import torch
from rich import print
from collections import deque
import mujoco.viewer as mjv
from tqdm import tqdm
import os
from data_utils.rot_utils import quatToEuler

try:
    import onnxruntime as ort
except ImportError:
    ort = None


def neutral_mimic_observation(default_dof_pos):
    """Return the safe, stationary 35-D command used on input failure."""
    dof_pos = np.asarray(default_dof_pos, dtype=np.float32)
    if dof_pos.shape != (29,):
        raise ValueError(f"default_dof_pos must have shape (29,), got {dof_pos.shape}")
    return np.concatenate([
        np.array([0.0, 0.0, 0.8, 0.0, 0.0, 0.0], dtype=np.float32),
        dof_pos,
    ])


def validate_redis_mimic_action(
        action_payload,
        timestamp_payload,
        default_dof_pos,
        *,
        now_ms,
        timeout_ms,
):
    """Decode a fresh finite 35-D Redis action, or return a safe fallback.

    The function is deliberately independent of MuJoCo and Redis so the wire
    contract can be unit-tested without either service running.
    """
    neutral = neutral_mimic_observation(default_dof_pos)

    if action_payload is None:
        return neutral, False, "action body is missing"
    try:
        decoded_action = json.loads(action_payload) if isinstance(
            action_payload, (bytes, bytearray, str)
        ) else action_payload
        action = np.asarray(decoded_action, dtype=np.float32)
    except (TypeError, ValueError, OverflowError, json.JSONDecodeError) as exc:
        return neutral, False, f"action body is not valid JSON/numeric data ({exc})"

    if action.shape != (35,):
        return neutral, False, f"action body has shape {action.shape}, expected (35,)"
    if not np.all(np.isfinite(action)):
        return neutral, False, "action body contains NaN or infinity"

    if timestamp_payload is None:
        return neutral, False, "t_action is missing"
    try:
        decoded_timestamp = json.loads(timestamp_payload) if isinstance(
            timestamp_payload, (bytes, bytearray, str)
        ) else timestamp_payload
        timestamp_ms = float(decoded_timestamp)
    except (TypeError, ValueError, OverflowError, json.JSONDecodeError) as exc:
        return neutral, False, f"t_action is invalid ({exc})"

    if not np.isfinite(timestamp_ms):
        return neutral, False, "t_action is not finite"

    age_ms = float(now_ms) - timestamp_ms
    if age_ms < 0.0:
        return neutral, False, f"t_action is {abs(age_ms):.0f} ms in the future"
    if age_ms > float(timeout_ms):
        return neutral, False, (
            f"t_action is stale ({age_ms:.0f} ms > {float(timeout_ms):.0f} ms)"
        )

    return action, True, "fresh action"


def _decode_optional_action(payload, size):
    """Best-effort decode for currently unused hand/neck commands."""
    if payload is None:
        return np.zeros(size, dtype=np.float32)
    try:
        decoded = json.loads(payload) if isinstance(
            payload, (bytes, bytearray, str)
        ) else payload
        value = np.asarray(decoded, dtype=np.float32)
    except (TypeError, ValueError, OverflowError, json.JSONDecodeError):
        return np.zeros(size, dtype=np.float32)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        return np.zeros(size, dtype=np.float32)
    return value


class OnnxPolicyWrapper:
    """Minimal wrapper so ONNXRuntime policies mimic TorchScript call signature."""

    def __init__(self, session, input_name, output_index=0):
        self.session = session
        self.input_name = input_name
        self.output_index = output_index

    def __call__(self, obs_tensor: torch.Tensor) -> torch.Tensor:
        if isinstance(obs_tensor, torch.Tensor):
            obs_np = obs_tensor.detach().cpu().numpy()
        else:
            obs_np = np.asarray(obs_tensor, dtype=np.float32)
        outputs = self.session.run(None, {self.input_name: obs_np})
        result = outputs[self.output_index]
        if not isinstance(result, np.ndarray):
            result = np.asarray(result, dtype=np.float32)
        return torch.from_numpy(result.astype(np.float32))


def load_onnx_policy(policy_path: str, device: str) -> OnnxPolicyWrapper:
    if ort is None:
        raise ImportError("onnxruntime is required for ONNX policy inference but is not installed.")
    providers = []
    available = ort.get_available_providers()
    if device.startswith('cuda'):
        if 'CUDAExecutionProvider' in available:
            providers.append('CUDAExecutionProvider')
        else:
            print("CUDAExecutionProvider not available in onnxruntime; falling back to CPUExecutionProvider.")
    providers.append('CPUExecutionProvider')
    session = ort.InferenceSession(policy_path, providers=providers)
    input_name = session.get_inputs()[0].name
    print(f"ONNX policy loaded from {policy_path} using providers: {session.get_providers()}")
    return OnnxPolicyWrapper(session, input_name)


class RealTimePolicyController:
    def __init__(self, 
                 xml_file, 
                 policy_path, 
                 device='cuda', 
                 record_video=False,
                 record_proprio=False,
                 measure_fps=False,
                 limit_fps=True,
                 policy_frequency=50,
                 viewer_mode='desktop',
                 vr_view='front',
                 vr_distance=2.2,
                 vr_eye_height=1.35,
                 vr_target_height=0.95,
                 vr_render_scale=0.75,
                 vr_msaa=2,
                 vr_recenter_button='right-thumbstick',
                 redis_host='localhost',
                 redis_port=6379,
                 action_timeout_ms=200,
                 ):
        self.measure_fps = measure_fps
        self.limit_fps = limit_fps
        if record_video and viewer_mode == 'vr':
            raise ValueError("record_video is not supported by the VR viewer")
        self.redis_client = None
        self.redis_host = redis_host
        self.redis_port = int(redis_port)
        self.action_timeout_ms = int(action_timeout_ms)
        if self.action_timeout_ms <= 0:
            raise ValueError("action_timeout_ms must be greater than zero")
        self._action_stream_valid = None
        try:
            self.redis_client = redis.Redis(host=self.redis_host, port=self.redis_port, db=0)
            self.redis_pipeline = self.redis_client.pipeline()
        except Exception as e:
            print(f"Error connecting to Redis: {e}")

        self.device = device
        self.policy = load_onnx_policy(policy_path, device)

        # Create MuJoCo sim
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.model.opt.timestep = 0.001
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        self.viewer_mode = viewer_mode
        self.vr_view = vr_view
        self.vr_distance = vr_distance
        self.vr_eye_height = vr_eye_height
        self.vr_target_height = vr_target_height
        self.vr_render_scale = vr_render_scale
        self.vr_msaa = vr_msaa
        self.vr_recenter_button = vr_recenter_button

        self.num_actions = 29
        self.sim_duration = 100000.0
        self.sim_dt = 0.001
        # real frequency = 1 / (decimation * sim_dt)
        # ==> decimation = 1 / (real frequency * sim_dt)
        requested_decimation = 1 / (policy_frequency * self.sim_dt)
        self.sim_decimation = int(round(requested_decimation))
        if self.sim_decimation < 1 or not np.isclose(requested_decimation, self.sim_decimation):
            raise ValueError(
                f"policy_frequency={policy_frequency} must divide the 1000 Hz simulation rate exactly"
            )
        print(f"sim_decimation: {self.sim_decimation}")

        self.last_action = np.zeros(self.num_actions, dtype=np.float32)

        # G1 specific configuration
        self.default_dof_pos = np.array([
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)
                0.0, 0.0, 0.0, # torso (3)
                0.0, 0.4, 0.0, 1.2, 0.0, 0.0, 0.0, # left arm (7)
                0.0, -0.4, 0.0, 1.2, 0.0, 0.0, 0.0, # right arm (7)
            ])

        self.mujoco_default_dof_pos = np.concatenate([
            np.array([0, 0, 0.793]),
            np.array([1, 0, 0, 0]),
             np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # left leg (6)
                -0.2, 0.0, 0.0, 0.4, -0.2, 0.0,  # right leg (6)
                0.0, 0.0, 0.0, # torso (3)
                0.0, 0.2, 0.0, 1.2, 0.0, 0.0, 0.0, # left arm (7)
                0.0, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0, # right arm (7)
                ])
        ])

        self.stiffness = np.array([
                100, 100, 100, 150, 40, 40,
                100, 100, 100, 150, 40, 40,
                150, 150, 150,
                40, 40, 40, 40, 4.0, 4.0, 4.0,
                40, 40, 40, 40, 4.0, 4.0, 4.0,
            ])
        self.damping = np.array([
                2, 2, 2, 4, 2, 2,
                2, 2, 2, 4, 2, 2,
                4, 4, 4,
                5, 5, 5, 5, 0.2, 0.2, 0.2,
                5, 5, 5, 5, 0.2, 0.2, 0.2,
            ])

        
        self.torque_limits = np.array([
                100, 100, 100, 150, 40, 40,
                100, 100, 100, 150, 40, 40,
                150, 150, 150,
                40, 40, 40, 40, 4.0, 4.0, 4.0,
                40, 40, 40, 40, 4.0, 4.0, 4.0,
            ])

        self.action_scale = np.array([
                0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                0.5, 0.5, 0.5,
                0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
            ])

        self.ankle_idx = [4, 5, 10, 11]

        self.n_mimic_obs = 35  # 6 + 29 (modified: root_vel_xy + root_pos_z + roll_pitch + yaw_ang_vel + dof_pos)
        self.n_proprio = 3 + 2 + 3*29    # from config analysis
        self.n_obs_single = 35 + 3 + 2 + 3*29  # n_mimic_obs + n_proprio = 35 + 92 = 127
        self.history_len = 10
        
        self.total_obs_size = self.n_obs_single * (self.history_len + 1) + self.n_mimic_obs   # 127*11 + 35 = 1402

        print(f"TWIST2 Controller Configuration:")
        print(f"  n_mimic_obs: {self.n_mimic_obs}")
        print(f"  n_proprio: {self.n_proprio}")
        print(f"  n_obs_single: {self.n_obs_single}")
        print(f"  history_len: {self.history_len}")
        print(f"  total_obs_size: {self.total_obs_size}")

        # Initialize history buffer
        self.proprio_history_buf = deque(maxlen=self.history_len)
        for _ in range(self.history_len):
            self.proprio_history_buf.append(np.zeros(self.n_obs_single, dtype=np.float32))

        # Recording
        self.record_video = record_video
        self.record_proprio = record_proprio
        self.proprio_recordings = [] if record_proprio else None

    def _log_action_stream_transition(self, is_valid, reason):
        """Log entering/leaving neutral fallback once, without per-frame spam."""
        if self._action_stream_valid is is_valid:
            return
        previous = self._action_stream_valid
        self._action_stream_valid = is_valid
        if is_valid:
            if previous is False:
                print("Redis action stream recovered; resuming fresh teleoperation commands.")
            else:
                print("Redis action stream is fresh.")
        else:
            print(f"Redis action stream unsafe: {reason}. Using neutral command.")

    def _start_viewer(self):
        """Start either the unchanged desktop viewer or the Quest/OpenVR viewer."""
        if self.viewer_mode == 'desktop':
            self.viewer = mjv.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = 0
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = 0
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = 0
            self.viewer.cam.distance = 2.0
            return

        if self.viewer_mode == 'vr':
            from twist2_vr.mujoco_openvr import OpenVRMuJoCoViewer, OpenVRViewerConfig

            self.viewer = OpenVRMuJoCoViewer(
                self.model,
                self.data,
                OpenVRViewerConfig(
                    view=self.vr_view,
                    distance_m=self.vr_distance,
                    eye_height_m=self.vr_eye_height,
                    target_height_m=self.vr_target_height,
                    render_scale=self.vr_render_scale,
                    msaa_samples=self.vr_msaa,
                    recenter_button=self.vr_recenter_button,
                ),
            )
            self.viewer.update_state(self.data)
            return

        raise ValueError(f"unsupported viewer mode: {self.viewer_mode}")
        

    def reset_sim(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def reset(self, init_pos):
        """Reset robot to initial position"""
        self.data.qpos[:] = init_pos
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def extract_data(self):
        """Extract robot state data"""
        n_dof = self.num_actions
        dof_pos = self.data.qpos[7:7+n_dof]
        dof_vel = self.data.qvel[6:6+n_dof]
        quat = self.data.qpos[3:7]
        ang_vel = self.data.qvel[3:6]
        sim_torque = self.data.ctrl
        return dof_pos, dof_vel, quat, ang_vel, sim_torque

    def run(self):
        """Main simulation loop"""
        print("Starting TWIST2 simulation...")

        # Video recording setup
        if self.record_video:
            import imageio
            mp4_writer = imageio.get_writer('twist2_simulation.mp4', fps=30)
        else:
            mp4_writer = None

        self.reset_sim()
        self.reset(self.mujoco_default_dof_pos)

        steps = int(self.sim_duration / self.sim_dt)
        pbar = tqdm(range(steps), desc="Simulating TWIST2...")

        # Send initial proprio to redis
        initial_obs = np.zeros(self.n_obs_single, dtype=np.float32)
        self.redis_pipeline.set("state_body_unitree_g1_with_hands", json.dumps(initial_obs.tolist()))
        self.redis_pipeline.set("state_hand_left_unitree_g1_with_hands", json.dumps(np.zeros(7).tolist()))
        self.redis_pipeline.set("state_hand_right_unitree_g1_with_hands", json.dumps(np.zeros(7).tolist()))
        self.redis_pipeline.execute()

        measure_fps = self.measure_fps
        fps_measurements = []
        fps_iteration_count = 0
        fps_measurement_target = 1000
        last_policy_time = None

        # Add policy execution FPS tracking for frequent printing
        policy_execution_times = []
        policy_step_count = 0
        policy_fps_print_interval = 100

        try:
            self._start_viewer()
            for i in pbar:
                t_start = time.time()
                dof_pos, dof_vel, quat, ang_vel, sim_torque = self.extract_data()
                
                if i % self.sim_decimation == 0:
                    # Build proprioceptive observation
                    rpy = quatToEuler(quat)
                    obs_body_dof_vel = dof_vel.copy()
                    obs_body_dof_vel[self.ankle_idx] = 0.
                    obs_proprio = np.concatenate([
                        ang_vel * 0.25,
                        rpy[:2], # only use roll and pitch
                        (dof_pos - self.default_dof_pos),
                        obs_body_dof_vel * 0.05,
                        self.last_action
                    ])

                    state_body = np.concatenate([
                        ang_vel,
                        rpy[:2],
                        dof_pos]) # 3+2+29 = 34 dims

                    # Send proprio to redis
                    
                    self.redis_pipeline.set("state_body_unitree_g1_with_hands", json.dumps(state_body.tolist()))
                    self.redis_pipeline.set("state_hand_left_unitree_g1_with_hands", json.dumps(np.zeros(7).tolist()))
                    self.redis_pipeline.set("state_hand_right_unitree_g1_with_hands", json.dumps(np.zeros(7).tolist()))
                    self.redis_pipeline.set("state_neck_unitree_g1_with_hands", json.dumps(np.zeros(2).tolist()))
                    self.redis_pipeline.set("t_state", int(time.time() * 1000)) # current timestamp in ms
                    self.redis_pipeline.execute()

                    # Get mimic obs from Redis
                    keys = [
                        "action_body_unitree_g1_with_hands",
                        "action_hand_left_unitree_g1_with_hands",
                        "action_hand_right_unitree_g1_with_hands",
                        "action_neck_unitree_g1_with_hands",
                        "t_action",
                    ]
                    for key in keys:
                        self.redis_pipeline.get(key)
                    redis_results = self.redis_pipeline.execute()
                    action_mimic, action_is_valid, action_reason = validate_redis_mimic_action(
                        redis_results[0],
                        redis_results[4],
                        self.default_dof_pos,
                        now_ms=int(time.time() * 1000),
                        timeout_ms=self.action_timeout_ms,
                    )
                    self._log_action_stream_transition(action_is_valid, action_reason)
                    if action_is_valid:
                        action_left_hand = _decode_optional_action(redis_results[1], 7)
                        action_right_hand = _decode_optional_action(redis_results[2], 7)
                        action_neck = _decode_optional_action(redis_results[3], 2)
                    else:
                        action_left_hand = np.zeros(7, dtype=np.float32)
                        action_right_hand = np.zeros(7, dtype=np.float32)
                        action_neck = np.zeros(2, dtype=np.float32)

                    # Construct observation for TWIST2 controller
                    obs_full = np.concatenate([action_mimic, obs_proprio])
                    # Update history
                    obs_hist = np.array(self.proprio_history_buf).flatten()
                    self.proprio_history_buf.append(obs_full)
                    future_obs = action_mimic.copy()
                    # Combine all observations: current + history + future (set to current frame for now)
                    obs_buf = np.concatenate([obs_full, obs_hist, future_obs])
                    

                    # Ensure correct total observation size
                    assert obs_buf.shape[0] == self.total_obs_size, f"Expected {self.total_obs_size} obs, got {obs_buf.shape[0]}"
                    
                    # Run policy
                    obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to(self.device)
                    with torch.no_grad():
                        raw_action = self.policy(obs_tensor).cpu().numpy().squeeze()
                    
                    # Measure and track policy execution FPS
                    current_time = time.time()
                    if last_policy_time is not None:
                        policy_interval = current_time - last_policy_time
                        current_policy_fps = 1.0 / policy_interval
                        
                        # For frequent printing (every 100 steps)  
                        policy_execution_times.append(policy_interval)
                        policy_step_count += 1
                        
                        # Print policy execution FPS every 100 steps
                        if policy_step_count % policy_fps_print_interval == 0:
                            recent_intervals = policy_execution_times[-policy_fps_print_interval:]
                            avg_interval = np.mean(recent_intervals)
                            avg_execution_fps = 1.0 / avg_interval
                            print(f"Policy Execution FPS (last {policy_fps_print_interval} steps): {avg_execution_fps:.2f} Hz (avg interval: {avg_interval*1000:.2f}ms)")
                        
                        # For detailed measurement (every 1000 steps)
                        if measure_fps:
                            fps_measurements.append(current_policy_fps)
                            fps_iteration_count += 1
                            
                            if fps_iteration_count == fps_measurement_target:
                                avg_fps = np.mean(fps_measurements)
                                max_fps = np.max(fps_measurements)
                                min_fps = np.min(fps_measurements)
                                std_fps = np.std(fps_measurements)
                                print(f"\n=== Policy Execution FPS Results (steps {fps_iteration_count-fps_measurement_target+1}-{fps_iteration_count}) ===")
                                print(f"Average Policy FPS: {avg_fps:.2f}")
                                print(f"Max Policy FPS: {max_fps:.2f}")
                                print(f"Min Policy FPS: {min_fps:.2f}")
                                print(f"Std Policy FPS: {std_fps:.2f}")
                                print(f"Expected FPS (from decimation): {1.0/(self.sim_decimation * self.sim_dt):.2f}")
                                print(f"=================================================================================\n")
                                # Reset for next 1000 measurements
                                fps_measurements = []
                                fps_iteration_count = 0
                    last_policy_time = current_time
                    
                    self.last_action = raw_action
                    raw_action = np.clip(raw_action, -10., 10.)
                    scaled_actions = raw_action * self.action_scale
                    pd_target = scaled_actions + self.default_dof_pos

                    # self.redis_client.set("action_low_level_unitree_g1", json.dumps(raw_action.tolist()))
                    
                    if self.viewer_mode == 'desktop':
                        # Existing monitor-only camera follows the pelvis.
                        pelvis_pos = self.data.xpos[self.model.body("pelvis").id]
                        self.viewer.cam.lookat = pelvis_pos
                        self.viewer.sync()
                    else:
                        # The renderer owns OpenVR/OpenGL on a separate thread.
                        # Only the latest robot state is copied here; rendering
                        # and waitGetPoses never block the physics loop.
                        self.viewer.update_state(self.data)
                        self.viewer.raise_if_failed()
                        if not self.viewer.is_running():
                            print("VR mirror window closed; stopping simulation.")
                            break
                    
                    if mp4_writer is not None:
                        img = self.viewer.read_pixels()
                        mp4_writer.append_data(img)

                    # Record proprio if enabled
                    if self.record_proprio:
                        proprio_data = {
                            'timestamp': time.time(),
                            'dof_pos': dof_pos.tolist(),
                            'dof_vel': dof_vel.tolist(),
                            'rpy': rpy.tolist(),
                            'ang_vel': ang_vel.tolist(),
                            'target_dof_pos': action_mimic.tolist()[-29:],
                        }
                        self.proprio_recordings.append(proprio_data)

               
                # PD control
                torque = (pd_target - dof_pos) * self.stiffness - dof_vel * self.damping
                torque = np.clip(torque, -self.torque_limits, self.torque_limits)
                
                self.data.ctrl[:] = torque
                mujoco.mj_step(self.model, self.data)
                
                # Sleep to maintain real-time pace
                if self.limit_fps:
                    elapsed = time.time() - t_start
                    if elapsed < self.sim_dt:
                        time.sleep(self.sim_dt - elapsed)

                    
        except KeyboardInterrupt:
            print("Simulation stopped by user.")
        except Exception as e:
            print(f"Error in run: {e}")
            import traceback
            traceback.print_exc()
            raise
        finally:
            if mp4_writer is not None:
                mp4_writer.close()
                print("Video saved as twist2_simulation.mp4")
            
            # Save proprio recordings if enabled
            if self.record_proprio and self.proprio_recordings:
                import pickle
                with open('twist2_proprio_recordings.pkl', 'wb') as f:
                    pickle.dump(self.proprio_recordings, f)
                print("Proprioceptive recordings saved as twist2_proprio_recordings.pkl")

            if self.viewer:
                self.viewer.close()
            print("Simulation finished.")


def main():
    parser = argparse.ArgumentParser(description='Run TWIST2 policy in simulation')
    parser.add_argument('--xml', type=str, default='../assets/g1/g1_sim2sim.xml',
                        help='Path to MuJoCo XML file')
    parser.add_argument('--policy', type=str, required=True,
                        help='Path to TWIST2 ONNX policy file')
    parser.add_argument('--device', type=str, 
                        default='cuda',
                        help='Device to run policy on (cuda/cpu)')
    parser.add_argument('--record_video', action='store_true',
                        help='Record video of simulation')
    parser.add_argument('--record_proprio', action='store_true',
                        help='Record proprioceptive data')
    parser.add_argument("--measure_fps", help="Measure FPS", default=0, type=int)
    parser.add_argument("--limit_fps", help="Limit FPS with sleep", default=1, type=int)
    parser.add_argument("--policy_frequency", help="Policy frequency", default=100, type=int)
    parser.add_argument("--redis-host", default="localhost",
                        help="Redis server hostname (default: localhost)")
    parser.add_argument("--redis-port", default=6379, type=int,
                        help="Redis server port (default: 6379)")
    parser.add_argument("--action-timeout-ms", default=200, type=int,
                        help="maximum accepted age of t_action in milliseconds")
    parser.add_argument(
        "--viewer", choices=("desktop", "vr"), default="desktop",
        help="desktop monitor viewer or ALVR/SteamVR stereoscopic Quest viewer",
    )
    parser.add_argument(
        "--vr-view", choices=("front",), default="front",
        help="initial Quest view; front places the user in front of the G1",
    )
    parser.add_argument("--vr-distance", type=float, default=2.2,
                        help="front-view distance from the G1 in meters")
    parser.add_argument("--vr-eye-height", type=float, default=1.35,
                        help="neutral virtual eye height in MuJoCo world meters")
    parser.add_argument("--vr-target-height", type=float, default=0.95,
                        help="height on the G1 that the neutral camera looks at")
    parser.add_argument("--vr-render-scale", type=float, default=0.75,
                        help="fraction of the SteamVR recommended per-eye resolution")
    parser.add_argument("--vr-msaa", type=int, choices=(0, 2, 4, 8), default=2,
                        help="MuJoCo offscreen MSAA samples for the Quest image")
    parser.add_argument(
        "--vr-recenter-button",
        choices=("right-thumbstick", "right-a", "right-b", "off"),
        default="right-thumbstick",
        help="Quest controller button used to recenter the front view",
    )
    args = parser.parse_args()

    if args.viewer == 'vr' and args.record_video:
        parser.error("--record_video is not supported by the VR viewer; use the desktop viewer")
    if args.action_timeout_ms <= 0:
        parser.error("--action-timeout-ms must be greater than zero")
    
    # Verify policy file exists
    if not os.path.exists(args.policy):
        print(f"Error: Policy file {args.policy} does not exist")
        return
    
    # Verify XML file exists
    if not os.path.exists(args.xml):
        print(f"Error: XML file {args.xml} does not exist")
        return
    
    print(f"Starting TWIST2 simulation controller...")
    print(f"  XML file: {args.xml}")
    print(f"  Policy file: {args.policy}")
    print(f"  Device: {args.device}")
    print(f"  Record video: {args.record_video}")
    print(f"  Record proprio: {args.record_proprio}")
    print(f"  Measure FPS: {args.measure_fps}")
    print(f"  Limit FPS: {args.limit_fps}")
    print(f"  Redis: {args.redis_host}:{args.redis_port}")
    print(f"  Action timeout: {args.action_timeout_ms} ms")
    print(f"  Viewer: {args.viewer}")
    if args.viewer == 'vr':
        print(f"  VR view: {args.vr_view} (distance={args.vr_distance:.2f}m)")
        print(f"  VR recenter button: {args.vr_recenter_button}")
    controller = RealTimePolicyController(
        xml_file=args.xml,
        policy_path=args.policy,
        device=args.device,
        record_video=args.record_video,
        record_proprio=args.record_proprio,
        measure_fps=args.measure_fps,
        limit_fps=args.limit_fps,
        policy_frequency=args.policy_frequency,
        viewer_mode=args.viewer,
        vr_view=args.vr_view,
        vr_distance=args.vr_distance,
        vr_eye_height=args.vr_eye_height,
        vr_target_height=args.vr_target_height,
        vr_render_scale=args.vr_render_scale,
        vr_msaa=args.vr_msaa,
        vr_recenter_button=args.vr_recenter_button,
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        action_timeout_ms=args.action_timeout_ms,
    )
    controller.run()


if __name__ == "__main__":
    main()
