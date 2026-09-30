"""Optional native Isaac Sim 4.5 stereo view through SteamVR / ALVR.

No image server or browser is involved. The native XR compositor renders the
scene; its tracking-space anchor follows environment zero's pelvis translation
and yaw. This helper must be hardware-checked with a connected headset.
"""
from __future__ import annotations

import math


class G1XRView:
    def __init__(self, env, app, backend="steamvr"):
        if env.num_envs != 1:
            raise ValueError("Interactive XR view requires exactly one G1 environment")
        if backend.lower() not in ("steamvr", "openxr"):
            raise ValueError("XR backend must be steamvr or openxr")
        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import UsdGeom

        self.env, self.app = env, app
        self.anchor_path = "/World/G1TeleopXRAnchor"
        self.attached = False
        self.status = "waiting_for_headset"
        self._closed = False
        self._stage = omni.usd.get_context().get_stage()
        self._anchor = UsdGeom.Xform.Define(self._stage, self.anchor_path)
        self._transform = self._anchor.AddTransformOp()
        # Change runtime extension state only; no SteamVR/ALVR config files.
        enable_extension("omni.kit.xr.system." + backend.lower())
        enable_extension("omni.kit.xr.profile.vr")
        for _ in range(5):
            app.update()
        from omni.kit.xr.core import XRCore
        self._core_type = XRCore
        self.core = XRCore.get_singleton()
        XRCore.request_enable_profile("vr")

    def recenter(self):
        """Recalibrate the viewer at the current robot head marker on next update."""
        self.attached = False

    def update(self):
        """Follow G1 each control tick; returns True after a live-headset recenter."""
        if self._closed:
            return False
        from pxr import Gf

        root = self.env.robot.data.root_state_w[0].detach().cpu().numpy()
        w, x, y, z = (float(v) for v in root[3:7])
        yaw = math.atan2(2. * (w*z+x*y), 1. - 2. * (y*y+z*z))
        pose = Gf.Matrix4d(1.)
        pose.SetRotate(Gf.Rotation(Gf.Vec3d(0., 0., 1.), math.degrees(yaw)))
        head_device = self.core.get_input_device("/user/head")
        if head_device is None:
            self.status = "waiting_for_headset"
            return False
        physical_head = head_device.get_pose().ExtractTranslation()
        # Missing/disconnected XR devices can present a dummy identity pose.
        if not all(math.isfinite(float(v)) for v in physical_head) or sum(float(v)**2 for v in physical_head) < 0.01:
            self.status = "waiting_for_valid_headset_pose"
            return False
        keypoint = self.env.current_keypoints()[0, 0].detach().cpu().tolist()
        # A small forward offset avoids viewing the inside of head geometry.
        eye = Gf.Vec3d(float(root[0]) + math.cos(yaw)*(keypoint[0]+0.10) - math.sin(yaw)*keypoint[1],
                      float(root[1]) + math.sin(yaw)*(keypoint[0]+0.10) + math.cos(yaw)*keypoint[1],
                      float(self.env.scene.env_origins[0, 2]) + self.env.nominal_root_height + keypoint[2])
        if self.attached:
            # Compensate physical head *translation*: crouch already moves the
            # controlled robot, so adding it again would double camera motion.
            # Virtual coordinates are used only for rendering correction, never
            # as policy inputs. Invert the PREVIOUS anchor, then apply the new
            # root yaw and translation. This avoids absolute-anchor feedback.
            old_anchor = self._transform.Get()
            virtual_head = head_device.get_virtual_world_pose().ExtractTranslation()
            anchor_local_head = old_anchor.GetInverse().Transform(virtual_head)
            pose.SetTranslateOnly(eye - pose.TransformDir(anchor_local_head))
            self._transform.Set(pose)
        else:
            pose.SetTranslateOnly(Gf.Vec3d(*(float(v) for v in root[:3])))
            self._transform.Set(pose)
            look_at = eye + Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0.)
            view_pose = Gf.Matrix4d().SetLookAt(eye, look_at, Gf.Vec3d(0., 0., 1.)).GetInverse()
            self.core.schedule_teleport_to_view(self.anchor_path, view_pose)
            self.attached = True
        self.status = "native_xr_anchor_attached_hardware_display_not_verified"
        return True

    def close(self):
        if not self._closed:
            # Attempt both operations independently; a dying compositor must
            # not prevent the runner from closing its simulator and receiver.
            errors = []
            for operation in (self.core.detach_stage_anchor, self.core.request_disable_profile):
                try:
                    operation()
                except Exception as error:
                    errors.append(str(error))
            self._closed = True
            self.attached = False
            self.status = "closed" if not errors else "closed_with_cleanup_errors"
            if errors:
                print("[G1 XR] Cleanup: " + "; ".join(errors), flush=True)
