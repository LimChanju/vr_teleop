from contextlib import contextmanager
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from twist2_vr import sparseposer_backend as backend


class Tensor:
    """Small NumPy tensor stand-in so wrapper tests do not require PyTorch."""

    def __init__(self, data):
        self.data = np.asarray(data)

    def __getitem__(self, item):
        return Tensor(self.data[item])

    def unsqueeze(self, axis):
        return Tensor(np.expand_dims(self.data, axis))

    def permute(self, *axes):
        return Tensor(self.data.transpose(axes))

    def flatten(self, start, end):
        assert (start, end) == (0, 1)
        return Tensor(self.data.reshape((-1,) + self.data.shape[2:]))

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.data


def valid_window():
    values = np.zeros((64, 6, 7))
    values[..., 3] = 1.0
    return values


@pytest.fixture
def model_backend(monkeypatch, tmp_path):
    skeleton = backend.Skeleton(
        tuple(f"joint_{index}" for index in range(22)),
        np.zeros(22, dtype=np.int64),
        np.ones((22, 3)),
        np.tile([0, 0, 0, 1], (22, 1)),
    )
    state = SimpleNamespace(no_grad=False, calls=[], model_devices=[], evaluated=[])

    @contextmanager
    def no_grad():
        state.no_grad = True
        try:
            yield
        finally:
            state.no_grad = False

    class Model:
        def __init__(self, *args):
            pass

        def to(self, device):
            state.model_devices.append(device)
            return self

        def eval(self):
            state.evaluated.append(self)

    class Dataset:
        def __init__(self, *args):
            pass

        def set_means_stds(self, *args):
            pass

        def set_offsets(self, offsets):
            state.offsets = offsets

        def normalize_offsets(self):
            state.calls.append("normalize_offsets")

        def set_motion(self, positions, rotations):
            assert state.no_grad
            state.positions = positions.copy()
            state.rotations = rotations.copy()
            state.calls.append("set_motion")

        def normalize_motion(self):
            state.calls.append("normalize_motion")

    def run_set_data(*args):
        state.calls.append("set_data")

    def generator(model):
        assert state.no_grad
        state.calls.append("generator")
        return Tensor(np.arange(64)[None, None, :] + np.zeros((1, 22 * 8, 64)))

    def ik(model, prediction, frame):
        assert state.no_grad and frame == -1
        state.calls.append("ik")
        return prediction

    def decode(dqs, parents):
        assert state.no_grad
        state.decoded = dqs.copy()
        state.output = np.tile([2.0, 0.0, 0.0, 2.0], (1, 22, 1))
        return None, state.output

    means = {"dqs": Tensor(np.full(22 * 8, 3.0))}
    stds = {"dqs": Tensor(np.full(22 * 8, 2.0))}
    train = SimpleNamespace(
        param={"sparse_joints": list(backend.SPARSE_JOINTS), "window_size": 64},
        Train_Data=lambda *args: object(), Generator_Model=Model, IK_Model=Model,
        load_model=lambda *args: (means, stds), run_set_data=run_set_data,
        run_generator=generator, run_ik=ik,
    )
    upstream = SimpleNamespace(
        torch=SimpleNamespace(device=lambda device: device, no_grad=no_grad),
        train=train, motion_data=SimpleNamespace(RunMotionData=Dataset),
        from_root_dual_quat=decode,
    )
    monkeypatch.setattr(backend, "load_skeleton", lambda *args: skeleton)
    monkeypatch.setattr(backend, "_load_upstream", lambda *args: upstream)
    for name in ("generator.pt", "ik.pt", "data.pt"):
        (tmp_path / name).touch()
    result = backend.SparsePoserBackend(tmp_path, tmp_path / "reference.bvh", tmp_path)
    return result, state


def test_inference_matches_official_sparse_layout_and_newest_frame(model_backend):
    model, state = model_backend
    window = valid_window()
    window[..., :3] = np.arange(64 * 6 * 3).reshape(64, 6, 3) / 100
    original = window.copy()
    result = model.infer(window)
    np.testing.assert_array_equal(window, original)
    np.testing.assert_array_equal(state.positions[:, backend.SPARSE_JOINTS], window[..., :3])
    np.testing.assert_array_equal(state.rotations[:, backend.SPARSE_JOINTS], window[..., 3:])
    unobserved = sorted(set(range(22)) - set(backend.SPARSE_JOINTS))
    np.testing.assert_array_equal(state.positions[:, unobserved], 0)
    np.testing.assert_array_equal(state.rotations[:, unobserved, 0], 1)
    np.testing.assert_array_equal(state.rotations[:, unobserved, 1:], 0)
    np.testing.assert_array_equal(state.decoded, np.full((1, 22, 8), 63 * 2 + 3))
    np.testing.assert_allclose(result, np.tile([0, 0, 2 ** -0.5, 2 ** -0.5], (22, 1)))
    assert state.calls == ["normalize_offsets", "set_motion", "normalize_motion", "set_data", "generator", "ik"]
    assert state.model_devices == ["cpu", "cpu"]
    assert len(state.evaluated) == 2
    assert not state.no_grad
    state.output[:] = 0
    np.testing.assert_allclose(np.linalg.norm(result, axis=-1), 1)


@pytest.mark.parametrize("kind", ["shape", "nan", "infinity", "zero_quaternion", "nonunit_quaternion"])
def test_bad_input_is_rejected_before_model_execution(model_backend, kind):
    model, state = model_backend
    window = valid_window()
    if kind == "shape":
        window = window[:63]
    elif kind == "nan":
        window[0, 0, 0] = np.nan
    elif kind == "infinity":
        window[0, 0, 0] = np.inf
    elif kind == "zero_quaternion":
        window[0, 0, 3:] = 0
    else:
        window[0, 0, 3] = 2
    with pytest.raises(ValueError):
        model.infer(window)
    assert state.calls == ["normalize_offsets"]


def test_invalid_model_output_does_not_return_a_dummy_pose(model_backend):
    model, _ = model_backend
    model._upstream.from_root_dual_quat = lambda *args: (None, np.zeros((1, 22, 4)))
    with pytest.raises(RuntimeError, match="zero joint quaternion"):
        model.infer(valid_window())


def test_private_upstream_imports_preserve_unrelated_modules_and_path(monkeypatch, tmp_path):
    (tmp_path / "train.py").write_text("MARKER = 42\nimport skeleton\nVALUE = skeleton.VALUE\n")
    (tmp_path / "skeleton.py").write_text("import train\nVALUE = train.MARKER\n")
    sentinel = ModuleType("skeleton")
    monkeypatch.setitem(sys.modules, "skeleton", sentinel)
    old_train = sys.modules.get("train")
    old_path = sys.path.copy()
    loaded = backend._import_upstream_module(tmp_path, "train")
    assert loaded.VALUE == 42
    assert loaded.skeleton.train is loaded
    assert sys.modules["skeleton"] is sentinel
    assert sys.modules.get("train") is old_train
    assert sys.path == old_path


def test_load_skeleton_uses_pymotion_without_importing_torch(monkeypatch, tmp_path):
    source = tmp_path / "python" / "src"
    source.mkdir(parents=True)
    (source / "train.py").touch()
    reference = tmp_path / "reference.bvh"
    reference.touch()
    imported = []

    class BVH:
        def load(self, path):
            assert path == str(reference)
            self.data = {
                "names": ["Hips", "Head"], "parents": [None, 0],
                "offsets": [[8, 9, 10], [0, 0.6, 0]],
                "rotations": np.zeros((1, 2, 3)), "rot_order": np.tile(["y", "x", "z"], (2, 1)),
            }

    def import_module(name):
        imported.append(name)
        if name == "pymotion.io.bvh":
            return SimpleNamespace(BVH=BVH)
        assert name == "pymotion.rotations.quat"
        return SimpleNamespace(
            from_euler=lambda angles, order: np.tile([1, 0, 0, 0], (2, 1)),
            normalize=lambda rotations: rotations,
        )

    monkeypatch.setattr(backend.importlib, "import_module", import_module)
    skeleton = backend.load_skeleton(tmp_path, reference)
    assert skeleton.names == ("Hips", "Head")
    np.testing.assert_array_equal(skeleton.parents, [0, 0])
    np.testing.assert_array_equal(skeleton.offsets, [[0, 0, 0], [0, 0.6, 0]])
    np.testing.assert_array_equal(skeleton.reference_local_rotations, [[0, 0, 0, 1], [0, 0, 0, 1]])
    assert imported == ["pymotion.io.bvh", "pymotion.rotations.quat"]
