"""Simulator arming transitions; no network, robot, or simulator dependencies."""
from types import SimpleNamespace
import unittest

from g1_teleop.runtime import TeleopGate


def frame(*, enabled=False, fresh=True, reset=False, session="session-a"):
    # The UDPReceiver already validates tracking/calibration and exposes the
    # effective enabled flag. The simulator gate supplies an independent latch.
    return SimpleNamespace(enabled=enabled, fresh=fresh, reset=reset, session=session)


class TeleopGateTests(unittest.TestCase):
    def test_startup_waits_for_input(self):
        gate = TeleopGate()
        self.assertFalse(gate.update(None))
        self.assertFalse(gate.update(frame(fresh=False, enabled=True)))
        self.assertFalse(gate.update(frame()))
        self.assertTrue(gate.update(frame(enabled=True)))

    def test_explicit_stop_takes_effect_on_same_poll(self):
        gate = TeleopGate()
        self.assertTrue(gate.update(frame(enabled=True)))
        self.assertFalse(gate.update(frame(enabled=False)))
        self.assertFalse(gate.active)
        self.assertTrue(gate.update(frame(enabled=True)))

    def test_missing_input_after_arming_latches_stop(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True))
        self.assertFalse(gate.update(None))
        self.assertTrue(gate.latched)
        self.assertEqual(gate.reason, "input_timeout")
        self.assertFalse(gate.update(frame(enabled=True)))

    def test_expired_enabled_input_cannot_keep_running(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True))
        self.assertFalse(gate.update(frame(enabled=True, fresh=False)))
        for _ in range(5):
            self.assertFalse(gate.update(frame(enabled=True)))

    def test_repeated_enabled_frames_do_not_bypass_any_trip(self):
        for reason in ("fall", "reset", "episode_end", "input_timeout"):
            with self.subTest(reason=reason):
                gate = TeleopGate()
                gate.update(frame(enabled=True))
                gate.trip(reason)
                for _ in range(30):
                    self.assertFalse(gate.update(frame(enabled=True)))
                self.assertFalse(gate.active)

    def test_fresh_release_then_arm_recovers_after_fall(self):
        gate = TeleopGate()
        gate.trip("fall")
        self.assertFalse(gate.update(frame(enabled=False)))
        self.assertTrue(gate.update(frame(enabled=True)))
        self.assertFalse(gate.latched)

    def test_stale_release_does_not_count_for_rearming(self):
        gate = TeleopGate()
        gate.trip("fall")
        self.assertFalse(gate.update(frame(enabled=False, fresh=False)))
        self.assertFalse(gate.update(frame(enabled=True)))
        self.assertFalse(gate.update(frame(enabled=False)))
        self.assertTrue(gate.update(frame(enabled=True)))

    def test_release_before_fall_does_not_count_after_fall(self):
        gate = TeleopGate()
        gate.update(frame(enabled=False))
        gate.update(frame(enabled=True))
        gate.trip("fall")
        self.assertFalse(gate.update(frame(enabled=True)))

    def test_reset_has_priority_over_enabled_in_same_frame(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True))
        self.assertFalse(gate.update(frame(enabled=True, reset=True)))
        self.assertEqual(gate.reason, "reset")
        self.assertFalse(gate.update(frame(enabled=True)))
        self.assertFalse(gate.update(frame(enabled=False)))
        self.assertTrue(gate.update(frame(enabled=True)))

    def test_disabled_reset_requires_next_explicit_arm(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True))
        self.assertFalse(gate.update(frame(enabled=False, reset=True)))
        self.assertFalse(gate.update(frame(enabled=False)))
        self.assertTrue(gate.update(frame(enabled=True)))

    def test_second_trip_invalidates_intermediate_release(self):
        gate = TeleopGate()
        gate.trip("fall")
        gate.update(frame(enabled=False))
        gate.trip("reset")
        self.assertFalse(gate.update(frame(enabled=True)))

    def test_outage_after_release_requires_new_release(self):
        # Regression: a release received before an outage cannot satisfy the
        # explicit rearming handshake after the outage, even while inactive.
        for stale in (None, frame(enabled=False, fresh=False)):
            with self.subTest(stale=stale):
                gate = TeleopGate()
                gate.trip("fall")
                gate.update(frame(enabled=False))
                self.assertFalse(gate.update(stale))
                self.assertFalse(gate.update(frame(enabled=True)))
                self.assertFalse(gate.update(frame(enabled=False)))
                self.assertTrue(gate.update(frame(enabled=True)))

    def test_first_valid_session_can_use_sender_explicit_arm(self):
        gate = TeleopGate()
        self.assertTrue(gate.update(frame(enabled=True, session="first-session")))
        self.assertEqual(gate.session, "first-session")

    def test_new_session_cannot_inherit_active_arm(self):
        gate = TeleopGate()
        self.assertTrue(gate.update(frame(enabled=True, session="old")))
        self.assertFalse(gate.update(frame(enabled=True, session="new")))
        self.assertEqual(gate.reason, "new_session")
        self.assertFalse(gate.update(frame(enabled=True, session="new")))
        self.assertFalse(gate.update(frame(enabled=False, session="new")))
        self.assertTrue(gate.update(frame(enabled=True, session="new")))

    def test_new_session_cannot_reuse_old_session_release(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True, session="old"))
        gate.trip("fall")
        gate.update(frame(enabled=False, session="old"))
        self.assertFalse(gate.update(frame(enabled=True, session="new")))
        self.assertFalse(gate.update(frame(enabled=False, session="new")))
        self.assertTrue(gate.update(frame(enabled=True, session="new")))

    def test_stale_foreign_session_cannot_change_current_session(self):
        gate = TeleopGate()
        gate.update(frame(enabled=True, session="current"))
        self.assertFalse(gate.update(frame(enabled=False, fresh=False, session="foreign")))
        self.assertEqual(gate.session, "current")
        self.assertFalse(gate.update(frame(enabled=True, session="current")))


if __name__ == "__main__":
    unittest.main()
