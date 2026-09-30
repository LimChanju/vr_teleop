"""Simulator-side arming latch, independent of the input sender's own controls."""


class TeleopGate:
    """After a fall or network loss, require fresh disabled -> enabled input.

    A sender that keeps streaming enabled=True cannot silently resume movement
    after the robot has reset or a long network outage has ended.
    """

    def __init__(self):
        self.latched = False
        self.seen_disabled = False
        self.active = False
        self.reason = "waiting"
        self.session = None

    def trip(self, reason):
        self.latched = True
        self.seen_disabled = False
        self.active = False
        self.reason = reason

    def update(self, frame):
        fresh = bool(frame and frame.fresh)
        if (self.active or self.latched) and not fresh:
            self.trip("input_timeout")
        incoming_session = getattr(frame, "session", None) if fresh else None
        if incoming_session:
            if self.session is not None and incoming_session != self.session:
                self.trip("new_session")
            self.session = incoming_session
        if frame and frame.reset:
            self.trip("reset")
        if fresh and not frame.enabled:
            self.seen_disabled = True
        if self.latched and fresh and frame.enabled and self.seen_disabled:
            self.latched = False
            self.seen_disabled = False
        self.active = bool(fresh and frame.enabled and not self.latched)
        return self.active
