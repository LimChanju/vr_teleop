#!/usr/bin/env python3
"""Create, verify and safely unpack a portable G1 policy/source bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import tarfile
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
ARCHIVE_ROOT = "vr_teleop_g1"
MAX_BUNDLE_BYTES = 4 * 1024**3
# Keep this at least as strict as scripts/g1/run.py:CONTRACT_KEYS. Tests enforce
# containment without importing/starting Isaac Sim.
MODEL_CONTRACT_KEYS = (
    "model", "body_joint_names", "observation_dim", "observation_order", "action_scale", "action_clip",
    "nominal_q", "soft_joint_limits", "control_dt", "physics_dt", "coordinate_version",
    "body_joint_stiffness", "body_joint_damping", "body_joint_armature", "body_joint_effort_limits",
    "collision_overrides", "nominal_root_height_m", "tracked_local_offsets_m", "hand_position_m", "observation_version",
    "stage", "critic_observations", "privileged_order", "tracked_body_names", "hand_joint_names",
    "nominal_keypoints_root_m",
)
OPTIONAL_CONTRACT_KEYS = ("actor_observation_order", "critic_observation_order", "target_velocity_estimator")
MAX_EVIDENCE_BYTES = 64 * 1024**2


def sha256(path):
    with Path(path).open("rb") as stream:
        return stream_hash(stream)


def stream_hash(stream):
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def safe_relative(text):
    if not isinstance(text, str) or not text or "\\" in text or any(ord(c) < 32 for c in text):
        raise ValueError(f"invalid bundle path: {text!r}")
    path = PurePosixPath(text)
    if path.is_absolute() or ".." in path.parts or str(path) != text:
        raise ValueError(f"unsafe bundle path: {text}")
    return path


def copy_stable(source, target):
    source, target = Path(source), Path(target)
    if source.is_symlink() or not source.is_file() or not stat.S_ISREG(source.stat().st_mode):
        raise ValueError(f"bundle source must be a regular non-symlink file: {source}")
    before = source.stat()
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as input_file, target.open("xb") as output:
        shutil.copyfileobj(input_file, output, 1024 * 1024)
    after = source.stat()
    if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
        raise ValueError(f"source changed while bundling; stop training or choose an immutable checkpoint: {source}")
    target.chmod(0o755 if before.st_mode & 0o111 else 0o644)


def copy_tree(source, target):
    source, target = Path(source), Path(target)
    if source.is_symlink() or not source.is_dir():
        raise ValueError(f"expected a real directory: {source}")
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        if any(part == "__pycache__" or part.startswith(".venv") for part in relative.parts):
            continue
        if path.suffix in (".pyc", ".pyo", ".pem", ".key") or path.name.startswith(".env"):
            continue
        safe_relative(relative.as_posix())
        if path.is_symlink():
            raise ValueError(f"refusing symlink in selected source tree: {path}")
        if path.is_file():
            copy_stable(path, target / relative)
        elif not path.is_dir():
            raise ValueError(f"unsupported source type: {path}")


def normalized_contract(metadata):
    if not isinstance(metadata, dict):
        raise ValueError("model metadata must be a JSON object")
    result = dict(metadata)
    result.setdefault("observation_version", "sparse_positions_v1")
    result.setdefault("training_velocity_limits", [0.0, 0.0, 0.0])
    result.setdefault("environment_variant", None)
    return result


def validate_contract(metadata, exported):
    metadata, exported = normalized_contract(metadata), normalized_contract(exported)
    for key in (*MODEL_CONTRACT_KEYS, "training_velocity_limits", "environment_variant"):
        if key not in metadata or key not in exported or exported[key] != metadata[key]:
            raise ValueError(f"exported policy contract mismatch: {key}")
    # Older v1 sidecars predate these derived descriptions. When they exist,
    # compare them; the v2 derivative estimator is an essential policy input.
    for key in OPTIONAL_CONTRACT_KEYS:
        if key in metadata and key in exported and metadata[key] != exported[key]:
            raise ValueError(f"exported policy contract mismatch: {key}")
        if metadata["observation_version"] == "sparse_tracking_v2" and (
                key not in metadata or key not in exported):
            raise ValueError(f"v2 exported policy contract missing: {key}")
    return metadata, exported


def copy_source_snapshot(run, metadata, destination):
    hashes = metadata.get("source_sha256", {})
    if not isinstance(hashes, dict):
        raise ValueError("source_sha256 must be an object")
    for relative, expected in hashes.items():
        path = Path(*safe_relative(relative).parts)
        original = run / "source" / path
        if not original.is_file() or sha256(original) != expected:
            raise ValueError(f"saved training source snapshot missing/mismatched: {relative}")
        copy_stable(original, destination / path)
    return len(hashes)


def copy_evidence(folder, destination, checkpoint_hash):
    """Include explicitly selected small reports, never a whole training run."""
    folder = Path(folder).resolve(strict=True)
    result_path = folder / "result.json"
    if not result_path.is_file():
        raise ValueError(f"evidence directory needs result.json: {folder}")
    result = json.loads(result_path.read_text())
    files = [folder / name for name in ("result.json", "run_config.json", "nominal_targets.json", "trace.npz",
                                       "scenario.jsonl", "failure_events.json")
             if (folder / name).is_file()]
    # Only the named scenario recorder and root analysis reports are added;
    # other raw tracking logs and unrelated root JSON remain excluded.
    files.extend(path for path in sorted(folder.glob("scenario_analysis*.json")) if path.is_file())
    analysis = folder / "analysis"
    if analysis.is_dir():
        allowed = {".json", ".png", ".svg", ".csv", ".npz", ".md", ".txt"}
        files.extend(path for path in sorted(analysis.rglob("*")) if path.is_file() and path.suffix in allowed)
    if len(files) > 200 or sum(path.stat().st_size for path in files) > MAX_EVIDENCE_BYTES:
        raise ValueError(f"selected evidence exceeds 200 files / 64 MiB: {folder}")
    for path in files:
        relative = path.relative_to(folder)
        safe_relative(relative.as_posix())
        # Also reject symlink parent directories, not just the selected file.
        if any(parent.is_symlink() for parent in path.parents if parent != folder and folder in parent.parents):
            raise ValueError(f"symlink in evidence path: {path}")
        copy_stable(path, destination / relative)
    reported = result.get("checkpoint")
    relation = "no_checkpoint"
    referenced_hash = None
    if reported:
        candidate = Path(reported).expanduser()
        if candidate.is_file():
            referenced_hash = sha256(candidate)
            relation = "same_checkpoint" if referenced_hash == checkpoint_hash else "other_checkpoint"
        else:
            relation = "unresolved_checkpoint"
    return {"directory": destination.name, "status": result.get("status"), "source_directory": folder.name,
            "checkpoint_relation": relation, "reported_checkpoint": reported,
            "reported_checkpoint_sha256": referenced_hash, "zero_policy": result.get("zero_policy", False),
            "files": [path.relative_to(folder).as_posix() for path in files]}


def create_bundle(run, output, checkpoint=None, policy_dir=None, assets=None, repo=ROOT, evidence=None):
    run, output, repo = Path(run).resolve(strict=True), Path(output).absolute(), Path(repo).resolve(strict=True)
    policy_dir = Path(policy_dir).resolve(strict=True) if policy_dir else run
    checkpoint = Path(checkpoint).resolve(strict=True) if checkpoint else run / "model_final.pt"
    checksum_path = output.with_name(output.name + ".sha256")
    if output.exists() or output.is_symlink() or checksum_path.exists() or checksum_path.is_symlink():
        raise FileExistsError("archive or its SHA256 sidecar already exists; choose a new output filename")
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint missing: {checkpoint}; specify --checkpoint explicitly")
    metadata = json.loads((run / "run_config.json").read_text())
    exported = json.loads((policy_dir / "policy.json").read_text())
    metadata, exported = validate_contract(metadata, exported)
    policy = policy_dir / "policy.pt"
    if exported.get("sha256") != sha256(policy):
        raise ValueError("policy.pt SHA256 differs from policy.json")
    if checkpoint != run / "model_final.pt":
        source = exported.get("source_checkpoint")
        if not source or Path(source).expanduser().resolve() != checkpoint:
            raise ValueError("non-final checkpoint needs a matching export directory (--policy-dir)")
    model_name = re.sub(r"[^A-Za-z0-9_.-]", "_", run.name)[:100]
    if model_name in ("", ".", ".."):
        raise ValueError("invalid run directory name")
    model_relative = Path("models") / model_name
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".g1-bundle-", dir=output.parent) as temporary:
        stage = Path(temporary) / ARCHIVE_ROOT
        stage.mkdir()
        for relative in ("README.md", ".gitignore", "docs/WEBXR_PROBE.md", "quest3_probe.py", "run_probe.sh",
                         "requirements.txt", "requirements.lock.txt", "scripts/setup.sh", "scripts/make_cert.py",
                         "tests/test_probe.py"):
            path = repo / relative
            if path.is_file():
                copy_stable(path, stage / relative)
        for relative in ("g1_teleop", "scripts/g1", "config/g1", "docs/g1", "tests/g1"):
            if (repo / relative).is_dir():
                copy_tree(repo / relative, stage / relative)
        model_dir = stage / model_relative
        copy_stable(checkpoint, model_dir / "model.pt")
        copy_stable(policy, model_dir / "policy.pt")
        copy_stable(policy_dir / "policy.json", model_dir / "policy.json")
        copy_stable(run / "run_config.json", model_dir / "run_config.json")
        copy_stable(run / "nominal_targets.json", model_dir / "nominal_targets.json")
        snapshot_count = copy_source_snapshot(run, metadata, model_dir / "source")
        for filename in ("result.json", "progress.json", "evaluation.json", "training_metrics.json", "training_metrics.png"):
            if (run / filename).is_file():
                copy_stable(run / filename, model_dir / filename)
        motion = metadata.get("motion_file")
        if motion:
            original = Path(motion).expanduser()
            candidates = (original, repo / original, repo / "data/motions" / original.name)
            motion_path = next((path for path in candidates if path.is_file()), None)
            if motion_path is None:
                raise FileNotFoundError(f"referenced motion data missing: {motion}")
            if metadata.get("motion_sha256") and sha256(motion_path) != metadata["motion_sha256"]:
                raise ValueError("motion SHA256 differs from the trained policy metadata")
            copy_stable(motion_path, stage / "data/motions" / motion_path.name)
            if motion_path.with_suffix(".json").is_file():
                copy_stable(motion_path.with_suffix(".json"), stage / "data/motions" / motion_path.with_suffix(".json").name)
        if assets:
            assets = Path(assets).resolve(strict=True)
            if not (assets / "g1_29dof_with_dex1_rev_1_0.usd").is_file():
                raise ValueError("--assets must point to the complete G1 Dex1 folder")
            copy_tree(assets, stage / "assets/robots/g1-29dof_wholebody_dex1")
        checkpoint_hash = sha256(model_dir / "model.pt")
        evidence_entries = []
        evidence_names = set()
        for directory in evidence or []:
            name = re.sub(r"[^A-Za-z0-9_.-]", "_", Path(directory).name)[:100]
            if name in ("", ".", "..") or name in evidence_names:
                raise ValueError("evidence directories must have distinct usable basenames")
            evidence_names.add(name)
            evidence_entries.append(copy_evidence(directory, stage / "evidence" / name, checkpoint_hash))
        descriptor = {
            "schema": "g1.bundle.v1", "created_utc": datetime.now(timezone.utc).isoformat(),
            "model_directory": model_relative.as_posix(), "checkpoint": (model_relative / "model.pt").as_posix(),
            "checkpoint_sha256": checkpoint_hash, "policy_sha256": sha256(model_dir / "policy.pt"),
            "source_checkpoint_filename": checkpoint.name, "source_run": run.name,
            "observation_version": metadata["observation_version"],
            "training_source_snapshot_files": snapshot_count, "evidence": evidence_entries,
            "assets_included": bool(assets), "physical_quest_verified": False,
            "note": "Bundle integrity does not certify policy performance or physical Quest connection; read evaluation reports.",
        }
        (stage / "BUNDLE.json").write_text(json.dumps(descriptor, indent=2) + "\n")
        files = sorted(path for path in stage.rglob("*") if path.is_file())
        if sum(path.stat().st_size for path in files) > MAX_BUNDLE_BYTES:
            raise ValueError("bundle exceeds 4 GiB; remove unrelated assets")
        manifest = "".join(f"{sha256(path)}  {path.relative_to(stage).as_posix()}\n" for path in files)
        (stage / "SHA256SUMS").write_text(manifest)
        # x prevents overwriting an existing archive even if one appears after preflight.
        created = False
        try:
            with output.open("xb") as output_file:
                created = True
                with tarfile.open(fileobj=output_file, mode="w:gz") as archive:
                    for path in sorted(stage.rglob("*")):
                        if path.is_file():
                            archive.add(path, arcname=f"{ARCHIVE_ROOT}/{path.relative_to(stage).as_posix()}", recursive=False,
                                        filter=_tar_metadata)
            verify_bundle(output)
            with checksum_path.open("x") as checksum:
                checksum.write(f"{sha256(output)}  {output.name}\n")
        except BaseException:
            if created:
                output.unlink(missing_ok=True)
            raise
    return descriptor


def _tar_metadata(info):
    info.uid = info.gid = 0
    info.uname = info.gname = ""
    info.mode = 0o755 if info.mode & 0o111 else 0o644
    return info


def checked_members(archive):
    members = archive.getmembers()
    if len(members) > 10000 or sum(member.size for member in members) > MAX_BUNDLE_BYTES:
        raise ValueError("archive exceeds entry/size limits")
    result = {}
    for member in members:
        path = safe_relative(member.name)
        if len(path.parts) < 2 or path.parts[0] != ARCHIVE_ROOT or not member.isfile():
            raise ValueError(f"only regular files below {ARCHIVE_ROOT} are accepted: {member.name}")
        relative = PurePosixPath(*path.parts[1:]).as_posix()
        if relative in result:
            raise ValueError(f"duplicate TAR entry: {relative}")
        result[relative] = member
    return result


def verify_bundle(path):
    with tarfile.open(path, "r:gz") as archive:
        members = checked_members(archive)
        manifest = members.get("SHA256SUMS")
        if manifest is None or manifest.size > 8 * 1024**2:
            raise ValueError("missing or oversized SHA256SUMS")
        entries = {}
        for line in archive.extractfile(manifest).read().decode("utf-8").splitlines():
            checksum, separator, relative = line.partition("  ")
            safe_relative(relative)
            if separator != "  " or not re.fullmatch("[0-9a-f]{64}", checksum) or relative in entries:
                raise ValueError("invalid SHA256SUMS line")
            entries[relative] = checksum
        if set(entries) != set(members) - {"SHA256SUMS"}:
            raise ValueError("SHA256SUMS does not cover the exact archive contents")
        for relative, checksum in entries.items():
            if stream_hash(archive.extractfile(members[relative])) != checksum:
                raise ValueError(f"SHA256 mismatch: {relative}")
        return len(entries)


def unpack_bundle(archive_path, destination):
    destination = Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError("unpack destination must be a new directory")
    verify_bundle(archive_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.mkdir()
    try:
        with tarfile.open(archive_path, "r:gz") as archive:
            for relative, member in checked_members(archive).items():
                target = destination.joinpath(*safe_relative(relative).parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output, 1024 * 1024)
                target.chmod(0o755 if member.mode & 0o111 else 0o644)
    except BaseException:
        shutil.rmtree(destination)
        raise
    return destination


def self_test():
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        repo, run = root / "repo", root / "run1"
        repo.mkdir()
        run.mkdir()
        (repo / "README.md").write_text("Test source")
        (repo / ".venv").mkdir()
        (repo / ".venv/private.key").write_text("must never be bundled")
        (run / "model_final.pt").write_bytes(b"test checkpoint")
        (run / "policy.pt").write_bytes(b"test policy")
        metadata = {key: "test" for key in MODEL_CONTRACT_KEYS}
        (run / "run_config.json").write_text(json.dumps(metadata))
        (run / "policy.json").write_text(json.dumps({**metadata, "sha256": sha256(run / "policy.pt")}))
        (run / "nominal_targets.json").write_text("[]")
        path = root / "bundle.tar.gz"
        create_bundle(run, path, repo=repo)
        assert verify_bundle(path) >= 7
        unpacked = unpack_bundle(path, root / "unpacked")
        assert (unpacked / "models/run1/model.pt").read_bytes() == b"test checkpoint"
        assert not (unpacked / ".venv").exists()
        for action in (lambda: create_bundle(run, path, repo=repo), lambda: unpack_bundle(path, unpacked)):
            try:
                action()
            except FileExistsError:
                pass
            else:
                raise AssertionError("existing output was overwritten")
        (run / "policy.pt").write_bytes(b"corrupt export")
        try:
            create_bundle(run, root / "bad.tar.gz", repo=repo)
        except ValueError:
            pass
        else:
            raise AssertionError("mismatched policy export was accepted")
        bad = root / "unsafe.tar.gz"
        with tarfile.open(bad, "w:gz") as archive:
            info = tarfile.TarInfo(f"{ARCHIVE_ROOT}/../escape")
            archive.addfile(info)
        try:
            verify_bundle(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe TAR path was accepted")
    print("Bundle offline checks passed: hash verification, policy contract, no-overwrite, exclusions, unpack, traversal")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    create = commands.add_parser("create")
    create.add_argument("--run", type=Path, required=True)
    create.add_argument("--output", type=Path, required=True)
    create.add_argument("--checkpoint", type=Path)
    create.add_argument("--policy-dir", type=Path)
    create.add_argument("--assets", type=Path)
    create.add_argument("--evidence", type=Path, action="append", default=[],
                        help="selected evaluation/teleop directory; repeat to include comparisons, limited to64MiB each")
    verify = commands.add_parser("verify")
    verify.add_argument("archive", type=Path)
    unpack = commands.add_parser("unpack")
    unpack.add_argument("archive", type=Path)
    unpack.add_argument("--destination", type=Path, required=True)
    commands.add_parser("self-test")
    args = parser.parse_args()
    try:
        if args.command == "create":
            print(json.dumps(create_bundle(args.run, args.output, args.checkpoint, args.policy_dir, args.assets,
                                           evidence=args.evidence), indent=2))
            print(f"Archive: {args.output}\nSHA256: {args.output}.sha256")
        elif args.command == "verify":
            print(f"Verified {verify_bundle(args.archive)} files")
        elif args.command == "unpack":
            print(unpack_bundle(args.archive, args.destination))
        else:
            self_test()
    except (OSError, ValueError, KeyError, tarfile.TarError) as exc:
        parser.exit(1, f"Bundle operation failed: {exc}\n")


if __name__ == "__main__":
    main()
