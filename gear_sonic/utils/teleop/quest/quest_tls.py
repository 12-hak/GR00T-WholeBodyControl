"""Self-signed TLS certs for Quest Browser (WebXR requires HTTPS on LAN)."""

from __future__ import annotations

import re
import ssl
import subprocess
from pathlib import Path

_IPV4_RE = re.compile(r"^(\d{1,3}\.){3}\d{1,3}$")


def sanitize_host_ip(host_ip: str, default: str = "192.168.1.235") -> str:
    host_ip = host_ip.strip()
    if _IPV4_RE.match(host_ip):
        return host_ip
    match = re.search(r"(\d{1,3}\.){3}\d{1,3}", host_ip)
    if match:
        return match.group(0)
    print(f"[Quest] Invalid host IP {host_ip!r}, using {default}")
    return default


def ensure_tls_cert(cert_dir: Path, host_ip: str) -> tuple[Path, Path]:
    host_ip = sanitize_host_ip(host_ip)
    cert_dir.mkdir(parents=True, exist_ok=True)
    cert_path = cert_dir / "cert.pem"
    key_path = cert_dir / "key.pem"
    if cert_path.exists() and key_path.exists():
        return cert_path, key_path

    print(f"[Quest] Generating self-signed TLS cert for {host_ip} ...")
    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-keyout",
            str(key_path),
            "-out",
            str(cert_path),
            "-days",
            "365",
            "-nodes",
            "-subj",
            f"/CN={host_ip}",
            "-addext",
            f"subjectAltName=IP:{host_ip}",
        ],
        check=True,
    )
    return cert_path, key_path


def make_server_ssl_context(cert_path: Path, key_path: Path) -> ssl.SSLContext:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(str(cert_path), str(key_path))
    return ctx
