from copy import deepcopy
import errno
import socket

import numpy as np
import pytest

from twist2_vr.retargeting import DEFAULT_MIMIC_OBS
from twist2_vr.sparseposer_sim_input import select_reference
from twist2_vr.sparseposer_physics import check_simulation_port, tilt_deg


def event():
    q = np.zeros(36)
    q[2:4] = [.8, 1.]
    return {"kind": "g1_kinematic_pose", "generated_at_s": 10., "timestamp_s": 10.,
            "fit_accepted": True, "mimic_obs": np.arange(35, dtype=float).tolist(),
            "qpos": q.tolist()}


def test_exact_input_no_motion_clamps_or_mutation():
    e = event()
    original = deepcopy(e)
    ref, state = select_reference(e, 10.01)
    assert state == "tracking"
    np.testing.assert_array_equal(ref.mimic_obs, e["mimic_obs"])
    assert e == original


@pytest.mark.parametrize("kind", ["tracking_blocked", "warming_up", "fatal_error", "complete"])
def test_no_tracking_uses_explicit_neutral(kind):
    ref, reason = select_reference({"kind": kind}, 10.)
    assert reason == kind
    assert ref.qpos is None
    np.testing.assert_array_equal(ref.mimic_obs, DEFAULT_MIMIC_OBS)


def test_invalid_fit_not_promoted_to_command():
    e = event()
    e["fit_accepted"] = False
    ref, reason = select_reference(e, 10.)
    assert reason == "invalid human/GMR fit"
    np.testing.assert_array_equal(ref.mimic_obs, DEFAULT_MIMIC_OBS)


@pytest.mark.parametrize("age", [.201, -1., float("nan")])
def test_stale_generated_pose_never_retimed(age):
    e = event()
    e["generated_at_s"] = 10.-age
    _, reason = select_reference(e, 10., replay=True)
    assert reason == "stale generated reference"


def test_live_stale_sensor_and_recorded_timestamp_are_distinct():
    e = event()
    e["timestamp_s"] = 1.
    assert select_reference(e, 10.)[1] == "stale sensor input"
    assert select_reference(e, 10., replay=True)[1] == "tracking"


def test_bad_payload_rejected():
    e = event()
    e["mimic_obs"][8] = float("nan")
    with pytest.raises(ValueError):
        select_reference(e, 10.)


def test_tilt_measurement():
    assert tilt_deg([1., 0., 0., 0.]) == pytest.approx(0)
    assert tilt_deg([2**-.5, 2**-.5, 0., 0.]) == pytest.approx(90)


@pytest.mark.parametrize("reuse", [False, True])
def test_simulation_port_rejects_live_listener(reuse):
    with socket.socket() as listener:
        if reuse:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        with pytest.raises(RuntimeError, match="already in use"):
            check_simulation_port(port)
        # The check must not stop or alter the existing listener.
        with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
            accepted, _ = listener.accept()
            accepted.close()


def test_simulation_port_allows_closed_server_time_wait():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        with socket.create_connection(("127.0.0.1", port), timeout=2) as client:
            accepted, _ = listener.accept()
            # Server is the active closer, leaving TIME_WAIT on its port.
            accepted.shutdown(socket.SHUT_WR)
            accepted.close()
            assert client.recv(1) == b""
    # Reproduce the old preflight failure, then verify the corrected one.
    with socket.socket() as old_probe:
        with pytest.raises(OSError) as exc:
            old_probe.bind(("127.0.0.1", port))
        assert exc.value.errno == errno.EADDRINUSE
    check_simulation_port(port)
