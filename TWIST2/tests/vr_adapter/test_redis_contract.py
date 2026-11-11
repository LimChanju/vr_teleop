import json

import numpy as np
import pytest

from twist2_vr.controller import RedisSimulationSink
from twist2_vr.models import RobotReference
from twist2_vr.retargeting import DEFAULT_MIMIC_OBS


class FakePipeline:
    def __init__(self, store):
        self.store = store
        self.pending = []

    def set(self, key, value):
        self.pending.append((key, value))
        return self

    def execute(self):
        for key, value in self.pending:
            self.store[key] = value
        self.pending.clear()
        return []


class FakeRedis:
    def __init__(self):
        self.store = {}
        self.pipe = FakePipeline(self.store)

    def ping(self):
        return True

    def pipeline(self):
        return self.pipe


def test_guarded_simulation_redis_contract(monkeypatch):
    monkeypatch.setenv("TWIST2_ALLOW_SIM_REDIS", "1")
    fake = FakeRedis()
    sink = RedisSimulationSink(client=fake, allow_simulation_publish=True)
    reference = RobotReference(1.0, DEFAULT_MIMIC_OBS.copy(), None, True, "test")
    sink.publish(reference)
    assert len(json.loads(fake.store[sink.KEYS["body"]])) == 35
    assert len(json.loads(fake.store[sink.KEYS["left_hand"]])) == 7
    assert len(json.loads(fake.store[sink.KEYS["right_hand"]])) == 7
    assert len(json.loads(fake.store[sink.KEYS["neck"]])) == 2
    assert "t_action" in fake.store
    sink.fail_safe("lost tracking")
    assert not sink.armed


def test_simulation_sink_rejects_remote_or_unarmed(monkeypatch):
    monkeypatch.delenv("TWIST2_ALLOW_SIM_REDIS", raising=False)
    with pytest.raises(ValueError, match="localhost"):
        RedisSimulationSink(host="192.168.1.2", allow_simulation_publish=True, client=FakeRedis())
    with pytest.raises(PermissionError, match="disarmed"):
        RedisSimulationSink(client=FakeRedis(), allow_simulation_publish=True)
