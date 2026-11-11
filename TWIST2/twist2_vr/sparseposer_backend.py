"""Thin, in-process adapter to the unmodified official SparsePoser models.

Inputs follow ``python/src/unity.py``: 64 frames of six calibrated poses,
ordered pelvis, left foot, right foot, head, left hand, right hand. The pelvis
pose is global; other poses are pelvis-relative. Positions use the reference
BVH's units and axes, and input quaternions are WXYZ. Outputs are local XYZW
joint rotations. Calibration, world FK and retargeting belong to the caller.
"""

from __future__ import annotations

import builtins
from dataclasses import dataclass
import hashlib
import importlib
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import numpy as np


WINDOW_SIZE = 64
SPARSE_JOINTS = (0, 4, 8, 13, 17, 21)
_IMPORT_LOCK = threading.RLock()


@dataclass(frozen=True)
class Skeleton:
    names: tuple[str, ...]
    parents: np.ndarray
    offsets: np.ndarray
    reference_local_rotations: np.ndarray


def _source_directory(checkout: str | Path) -> Path:
    source = Path(checkout).expanduser().resolve() / "python" / "src"
    if not (source / "train.py").is_file():
        raise ValueError(f"SparsePoser checkout has no python/src/train.py: {checkout}")
    return source


def _import_upstream_module(source: Path, name: str) -> ModuleType:
    """Namespace upstream's flat imports without changing global import paths.

    SparsePoser uses names such as ``train``, ``loss`` and ``skeleton``. A
    module-local import hook redirects only siblings in that checkout, leaving
    existing modules with those names and third-party imports untouched.
    """
    namespace = "_twist2_sparseposer_" + hashlib.sha256(str(source).encode()).hexdigest()[:16]
    siblings = {path.stem for path in source.glob("*.py")}

    def local_import(module_name, globals=None, locals=None, fromlist=(), level=0):
        if level == 0 and module_name in siblings:
            return load(module_name)
        return builtins.__import__(module_name, globals, locals, fromlist, level)

    def load(module_name: str) -> ModuleType:
        qualified = f"{namespace}.{module_name}"
        if qualified in sys.modules:
            return sys.modules[qualified]
        spec = importlib.util.spec_from_file_location(qualified, source / f"{module_name}.py")
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load SparsePoser module: {module_name}")
        module = importlib.util.module_from_spec(spec)
        module.__dict__["__builtins__"] = {**vars(builtins), "__import__": local_import}
        sys.modules[qualified] = module  # Required for upstream's circular imports.
        try:
            spec.loader.exec_module(module)
        except BaseException:
            sys.modules.pop(qualified, None)
            raise
        return module

    with _IMPORT_LOCK:
        if namespace not in sys.modules:
            package = ModuleType(namespace)
            package.__path__ = [str(source)]
            sys.modules[namespace] = package
        before = set(sys.modules)
        try:
            return load(name)
        except BaseException:
            # Do not cache a partial import graph after a missing dependency.
            for key in set(sys.modules) - before:
                if key.startswith(namespace + "."):
                    sys.modules.pop(key, None)
            raise


def _load_upstream(checkout: str | Path) -> SimpleNamespace:
    source = _source_directory(checkout)
    try:
        return SimpleNamespace(
            torch=importlib.import_module("torch"),
            train=_import_upstream_module(source, "train"),
            motion_data=_import_upstream_module(source, "motion_data"),
            from_root_dual_quat=importlib.import_module("pymotion.ops.skeleton").from_root_dual_quat,
        )
    except ImportError as error:
        raise RuntimeError(
            "SparsePoser requires PyTorch and upc-pymotion==0.1.8; "
            f"upstream import failed: {error}"
        ) from error


def load_skeleton(checkout: str | Path, path: str | Path) -> Skeleton:
    """Read a reference BVH with PyMotion, without loading torch or weights."""
    _source_directory(checkout)
    reference = Path(path).expanduser().resolve()
    if not reference.is_file():
        raise ValueError(f"SparsePoser reference BVH not found: {reference}")
    try:
        bvh_type = importlib.import_module("pymotion.io.bvh").BVH
        quat = importlib.import_module("pymotion.rotations.quat")
    except ImportError as error:
        raise RuntimeError("reading SparsePoser skeletons requires upc-pymotion==0.1.8") from error
    bvh = bvh_type()
    bvh.load(str(reference))
    names = tuple(str(name) for name in bvh.data["names"])
    parents = np.array([0 if parent is None else parent for parent in bvh.data["parents"]], dtype=np.int64)
    offsets = np.array(bvh.data["offsets"], dtype=np.float64, copy=True)
    joint_count = len(names)
    if joint_count == 0 or parents.shape != (joint_count,) or offsets.shape != (joint_count, 3):
        raise ValueError("malformed SparsePoser reference skeleton")
    parents[0] = 0  # Same root convention as upstream train.get_info_from_bvh.
    offsets[0] = 0
    if any(parent < 0 or parent >= joint for joint, parent in enumerate(parents[1:], start=1)):
        raise ValueError("reference skeleton must list parents before their children")
    angles = np.asarray(bvh.data["rotations"], dtype=np.float64)
    if angles.ndim != 3 or angles.shape[0] < 1 or angles.shape[1:] != (joint_count, 3):
        raise ValueError("reference BVH must contain at least one complete pose")
    rotations = quat.normalize(quat.from_euler(np.radians(angles[0]), order=bvh.data["rot_order"]))
    rotations = np.array(rotations[:, [1, 2, 3, 0]], dtype=np.float64, copy=True)
    if not np.isfinite(offsets).all() or not np.isfinite(rotations).all():
        raise ValueError("reference skeleton contains nonfinite values")
    return Skeleton(names, parents, offsets, rotations)


def _validate_window(window: np.ndarray) -> np.ndarray:
    values = np.array(window, dtype=np.float64, copy=True)
    if values.shape != (WINDOW_SIZE, len(SPARSE_JOINTS), 7):
        raise ValueError("SparsePoser input must have shape (64, 6, 7)")
    if not np.isfinite(values).all():
        raise ValueError("SparsePoser input must contain only finite values")
    norms = np.linalg.norm(values[..., 3:], axis=-1)
    if not np.allclose(norms, 1.0, rtol=0.0, atol=1e-3):
        raise ValueError("SparsePoser input quaternions must be unit WXYZ quaternions")
    return values


class SparsePoserBackend:
    def __init__(
        self, checkout: str | Path, reference_bvh: str | Path,
        model_dir: str | Path, device: str = "cpu",
    ) -> None:
        self.skeleton = load_skeleton(checkout, reference_bvh)
        if len(self.skeleton.names) != 22:
            raise ValueError("released SparsePoser models require a 22-joint reference skeleton")
        model_path = Path(model_dir).expanduser().resolve()
        for filename in ("generator.pt", "ik.pt", "data.pt"):
            if not (model_path / filename).is_file():
                raise ValueError(f"SparsePoser model file not found: {model_path / filename}")
        self._upstream = _load_upstream(checkout)
        torch, train = self._upstream.torch, self._upstream.train
        self.device = torch.device(device)
        if tuple(train.param["sparse_joints"]) != SPARSE_JOINTS or train.param["window_size"] != WINDOW_SIZE:
            raise ValueError("upstream SparsePoser input layout differs from the supported six-pose model")
        self._train_data = train.Train_Data(self.device, train.param)
        self._generator = train.Generator_Model(
            self.device, train.param, self.skeleton.parents.copy(), self._train_data,
        ).to(self.device)
        self._ik = train.IK_Model(
            self.device, train.param, self.skeleton.parents.copy(), self._train_data,
        ).to(self.device)
        train.load_model(self._generator, str(model_path / "generator.pt"), self._train_data, self.device)
        self._means, self._stds = train.load_model(
            self._ik, str(model_path / "ik.pt"), self._train_data, self.device,
        )
        self._generator.eval()
        self._ik.eval()
        self._dataset = self._upstream.motion_data.RunMotionData(train.param, self.device)
        self._dataset.set_means_stds(self._means, self._stds)
        self._dataset.set_offsets(self.skeleton.offsets.copy())
        self._dataset.normalize_offsets()
        self._inference_lock = threading.Lock()

    def infer(self, window: np.ndarray) -> np.ndarray:
        """Return the newest pose as independent local XYZW rotations[J,4]."""
        values = _validate_window(window)
        joint_count = len(self.skeleton.names)
        positions = np.zeros((WINDOW_SIZE, joint_count, 3), dtype=np.float64)
        rotations = np.zeros((WINDOW_SIZE, joint_count, 4), dtype=np.float64)
        rotations[..., 0] = 1.0
        positions[:, SPARSE_JOINTS, :] = values[..., :3]
        rotations[:, SPARSE_JOINTS, :] = values[..., 3:]
        train = self._upstream.train
        with self._inference_lock, self._upstream.torch.no_grad():
            self._dataset.set_motion(positions, rotations)
            self._dataset.normalize_motion()
            train.run_set_data(self._train_data, self._dataset)
            result = train.run_generator(self._generator)
            result = train.run_ik(self._ik, result, -1)
            result = result[:, :, -1].unsqueeze(-1).permute(0, 2, 1).flatten(0, 1)
            dqs = result.detach().cpu().numpy().copy()
            dqs = dqs * self._stds["dqs"].detach().cpu().numpy() + self._means["dqs"].detach().cpu().numpy()
            _, local_wxyz = self._upstream.from_root_dual_quat(
                dqs.reshape(1, joint_count, 8), self.skeleton.parents.copy(),
            )
        local_wxyz = np.asarray(local_wxyz, dtype=np.float64)
        if local_wxyz.shape != (1, joint_count, 4) or not np.isfinite(local_wxyz).all():
            raise RuntimeError("SparsePoser produced malformed or nonfinite joint rotations")
        norms = np.linalg.norm(local_wxyz[0], axis=-1)
        if np.any(norms < 1e-8):
            raise RuntimeError("SparsePoser produced a zero joint quaternion")
        # The official Unity consumer normalizes these quaternions as well.
        return np.array(local_wxyz[0][:, [1, 2, 3, 0]] / norms[:, None], copy=True)
