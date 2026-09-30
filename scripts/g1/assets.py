#!/usr/bin/env python3
"""Install only the G1 29-DoF free-base Dex1 USD tree without overwriting assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
ASSET_NAME = "g1-29dof_wholebody_dex1"
USD_NAME = "g1_29dof_with_dex1_rev_1_0.usd"
HF_REVISION = "394cf2448f8a9ed815c77c701a761f3d1ff1c8fb"
ZIP_SHA256 = "06fbf14549be3a81e3dbd4a8a019e7f060f48de1556b70f291a18ac8f9ead4b0"
ZIP_SIZE = 1305090539
ZIP_URL = f"https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds/resolve/{HF_REVISION}/assets.zip"
MAX_SELECTED_BYTES = 2 * 1024**3


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def regular_files(folder):
    result = []
    for path in sorted(Path(folder).rglob("*")):
        if path.is_symlink():
            raise ValueError(f"symlink is not accepted in assets: {path}")
        if path.is_file():
            if not stat.S_ISREG(path.stat().st_mode):
                raise ValueError(f"non-regular asset: {path}")
            result.append(path)
        elif not path.is_dir():
            raise ValueError(f"unsupported asset type: {path}")
    if sum(path.stat().st_size for path in result) > MAX_SELECTED_BYTES:
        raise ValueError("selected G1 asset exceeds the 2 GiB limit")
    return result


def resolve_source(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.is_file() and path.name == USD_NAME:
        path = path.parent
    choices = (path, path / ASSET_NAME, path / "robots" / ASSET_NAME,
               path / "assets/robots" / ASSET_NAME, path / "assets/robot" / ASSET_NAME)
    for candidate in choices:
        if (candidate / USD_NAME).is_file():
            regular_files(candidate)
            return candidate
    raise FileNotFoundError(f"{USD_NAME} was not found under {path}")


def download_zip(cache):
    cache = Path(cache).expanduser()
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / f"unitree-assets-{HF_REVISION}.zip"
    if target.exists() or target.is_symlink():
        if target.is_symlink() or target.stat().st_size != ZIP_SIZE or sha256(target) != ZIP_SHA256:
            raise ValueError(f"existing cache has the wrong SHA256; move it aside manually: {target}")
        return target
    fd, temporary = tempfile.mkstemp(prefix="unitree-download-", suffix=".part", dir=cache)
    temporary = Path(temporary)
    digest, count = hashlib.sha256(), 0
    try:
        print(f"Downloading pinned Unitree archive ({ZIP_SIZE / 1e9:.2f} GB): {ZIP_URL}", flush=True)
        request = urllib.request.Request(ZIP_URL, headers={"User-Agent": "vr_teleop-g1-assets/1"})
        with os.fdopen(fd, "wb") as output, urllib.request.urlopen(request, timeout=60) as response:
            while True:
                chunk = response.read(4 * 1024 * 1024)
                if not chunk:
                    break
                count += len(chunk)
                if count > ZIP_SIZE:
                    raise ValueError("download is larger than the pinned archive")
                digest.update(chunk)
                output.write(chunk)
        if count != ZIP_SIZE or digest.hexdigest() != ZIP_SHA256:
            raise ValueError("download size or SHA256 differs from the pinned HF LFS object")
        # Hard link gives atomic no-overwrite publication of the completed cache.
        os.link(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
    return target


def extract_selected(archive, destination):
    destination = Path(destination)
    prefixes = (f"assets/robots/{ASSET_NAME}/", f"assets/robot/{ASSET_NAME}/", f"{ASSET_NAME}/")
    with zipfile.ZipFile(archive) as zipped:
        entries = zipped.infolist()
        present = [prefix for prefix in prefixes if any(item.filename == prefix + USD_NAME for item in entries)]
        if len(present) != 1:
            raise ValueError("archive must contain exactly one matching G1 Dex1 asset folder")
        prefix = present[0]
        selected = [item for item in entries if item.filename.startswith(prefix) and item.filename != prefix]
        if len(selected) > 10000 or sum(item.file_size for item in selected) > MAX_SELECTED_BYTES:
            raise ValueError("selected archive entries exceed limits")
        seen = set()
        validated = []
        for item in selected:
            relative_text = item.filename[len(prefix):]
            relative = PurePosixPath(relative_text)
            mode = item.external_attr >> 16
            if (relative.is_absolute() or ".." in relative.parts or "\\" in relative_text
                    or "\x00" in relative_text or str(relative) in seen):
                raise ValueError(f"unsafe/duplicate ZIP path: {item.filename}")
            if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
                raise ValueError(f"unsupported ZIP entry: {item.filename}")
            if item.flag_bits & 1:
                raise ValueError("encrypted assets are not supported")
            seen.add(str(relative))
            validated.append((item, relative))
        for item, relative in validated:
            target = destination.joinpath(*relative.parts)
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with zipped.open(item) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output, length=1024 * 1024)


def install_assets(destination, *, source=None, archive=None):
    destination = Path(destination).expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"destination exists; nothing changed: {destination}")
    if (source is None) == (archive is None):
        raise ValueError("choose exactly one local source or ZIP archive")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".g1-assets-", dir=destination.parent) as temporary:
        stage = Path(temporary)
        if source is not None:
            source = resolve_source(source)
            for path in regular_files(source):
                if path.name == "asset_manifest.json":
                    continue
                target = stage / path.relative_to(source)
                target.parent.mkdir(parents=True, exist_ok=True)
                with path.open("rb") as input_file, target.open("xb") as output:
                    shutil.copyfileobj(input_file, output)
            provenance = {"kind": "local-copy", "source_folder": source.name}
        else:
            archive = Path(archive)
            extract_selected(archive, stage)
            provenance = {"kind": "zip", "archive_sha256": sha256(archive)}
            if provenance["archive_sha256"] == ZIP_SHA256:
                provenance.update(url=ZIP_URL, hf_revision=HF_REVISION)
        if not (stage / USD_NAME).is_file():
            raise ValueError("root USD missing after asset preparation")
        hashes = {str(path.relative_to(stage)): sha256(path) for path in regular_files(stage)}
        manifest = {"asset": ASSET_NAME, "root_usd": USD_NAME, "source": provenance, "files_sha256": hashes}
        (stage / "asset_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        # Claim a new directory, then fill it. An existing empty directory is not replaced.
        destination.mkdir()
        try:
            for child in stage.iterdir():
                shutil.move(str(child), str(destination / child.name))
        except BaseException:
            shutil.rmtree(destination)
            raise
    return destination / USD_NAME


def self_test():
    """Small offline checks with throwaway ZIPs and folders."""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        source = root / "source"
        source.mkdir()
        (source / USD_NAME).write_bytes(b"test USD")
        installed = install_assets(root / "installed", source=source)
        assert installed.read_bytes() == b"test USD"
        manifest = json.loads((installed.parent / "asset_manifest.json").read_text())
        assert manifest["files_sha256"][USD_NAME] == sha256(installed)
        try:
            install_assets(installed.parent, source=source)
        except FileExistsError:
            pass
        else:
            raise AssertionError("existing destination was not protected")
        zip_path = root / "good.zip"
        with zipfile.ZipFile(zip_path, "w") as archive:
            archive.writestr(f"assets/robots/{ASSET_NAME}/{USD_NAME}", b"zip USD")
            archive.writestr("assets/unrelated/large.dat", b"excluded")
        result = install_assets(root / "unzipped", archive=zip_path)
        assert result.read_bytes() == b"zip USD"
        assert not (root / "unzipped/unrelated").exists()
        for bad_name in ("../escaped", "nested/../../escaped"):
            bad_zip = root / "bad.zip"
            with zipfile.ZipFile(bad_zip, "w") as archive:
                archive.writestr(f"assets/robots/{ASSET_NAME}/{USD_NAME}", b"USD")
                archive.writestr(f"assets/robots/{ASSET_NAME}/{bad_name}", b"bad")
            try:
                install_assets(root / "unsafe", archive=bad_zip)
            except ValueError:
                pass
            else:
                raise AssertionError("ZIP path traversal was accepted")
            assert not (root / "unsafe").exists()
        (source / "linked").symlink_to(root / "installed" / USD_NAME)
        try:
            install_assets(root / "linked-output", source=source)
        except ValueError:
            pass
        else:
            raise AssertionError("asset symlink was accepted")
    print("Asset offline checks passed: copy/hash, no-overwrite, selected ZIP, traversal, symlink")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument("--source", type=Path, help="existing Dex1 folder, root USD, assets folder or Unitree repository")
    choice.add_argument("--zip", type=Path, help="already downloaded ZIP")
    choice.add_argument("--download", action="store_true", help="download the pinned 1.31 GB Unitree ZIP")
    parser.add_argument("--destination", type=Path, default=ROOT / "assets/robots" / ASSET_NAME)
    parser.add_argument("--cache", type=Path, default=Path.home() / ".cache/vr_teleop/assets")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not (args.source or args.zip or args.download):
        parser.error("choose --source, --zip or --download")
    try:
        if args.destination.exists() or args.destination.is_symlink():
            raise FileExistsError(f"destination exists; nothing changed: {args.destination}")
        archive = download_zip(args.cache) if args.download else args.zip
        print(install_assets(args.destination, source=args.source, archive=archive))
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        parser.exit(1, f"Asset preparation failed: {exc}\n")


if __name__ == "__main__":
    main()
