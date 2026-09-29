"""Create a local test certificate for a server IP address or DNS name."""

import argparse
import ipaddress
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", required=True, help="IP/DNS name reachable from Quest; omit https://")
    parser.add_argument("--out-dir", type=Path, default=Path("certs"))
    args = parser.parse_args()
    try:
        address = ipaddress.ip_address(args.server_host)
        entry = f"IP:{address}"
    except ValueError:
        if not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?", args.server_host):
            parser.error("--server-host must be an IP address or DNS name")
        entry = f"DNS:{args.server_host}"
    cert, key = args.out_dir / "cert.pem", args.out_dir / "key.pem"
    if cert.exists() or key.exists():
        parser.error("Certificate/key already exists; choose a new --out-dir to preserve them")
    args.out_dir.mkdir(parents=True, mode=0o700, exist_ok=True)
    # Pre-create the private key with owner-only permissions before OpenSSL writes.
    key.touch(mode=0o600, exist_ok=False)
    subprocess.run([
        "openssl", "req", "-x509", "-nodes", "-days", "365", "-newkey", "rsa:2048",
        "-keyout", str(key), "-out", str(cert), "-subj", "/CN=Quest3-Controller-Probe",
        "-addext", f"subjectAltName=DNS:localhost,IP:127.0.0.1,{entry}",
    ], check=True)
    print(f"Certificate: {cert.resolve()}")
    print(f"Private key: {key.resolve()} (keep this on the server)")


if __name__ == "__main__":
    main()
