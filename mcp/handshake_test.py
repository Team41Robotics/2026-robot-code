"""Minimal MCP stdio handshake test against `mcp/server.py`.

This is a local diagnostic to validate the server speaks MCP over stdio
(the same way Claude Code will launch it).

It starts the server subprocess, sends `initialize` + `initialized`, then asks
for `tools/list`.

Run:
  ./.venv/Scripts/python.exe mcp/handshake_test.py
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SERVER_PATH = PROJECT_ROOT / "mcp" / "server.py"


def _send(proc: subprocess.Popen[bytes], msg: dict) -> None:
    body = json.dumps(msg, separators=(",", ":")).encode("utf-8")
    proc.stdin.write(body + b"\n")
    proc.stdin.flush()


def _recv_line(proc: subprocess.Popen[bytes], timeout_s: float = 5.0) -> dict:
    start = time.time()
    buf = b""
    while time.time() - start < timeout_s:
        chunk = proc.stdout.readline()
        if chunk:
            buf = chunk.strip()
            break
        time.sleep(0.01)
    if not buf:
        raise TimeoutError("No response from MCP server")
    return json.loads(buf.decode("utf-8"))


def main() -> int:
    # Launch server exactly like `.mcp.json` intends.
    python_exe = sys.executable
    proc = subprocess.Popen(
        [python_exe, str(SERVER_PATH)],
        cwd=str(PROJECT_ROOT),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    try:
        # MCP initialize
        _send(
            proc,
            {
                "jsonrpc": "2.0",
                "id": 1,
                "method": "initialize",
                "params": {
                    "protocolVersion": "2024-11-05",
                    "capabilities": {},
                    "clientInfo": {"name": "frc41-handshake-test", "version": "0"},
                },
            },
        )
        init_resp = _recv_line(proc)

        # Client sends initialized notification (current MCP spec)
        _send(proc, {"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}})

        # tools/list
        _send(proc, {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}})
        tools_resp = _recv_line(proc)

        print("initialize response keys:", sorted(init_resp.keys()))
        tools = tools_resp.get("result", {}).get("tools", [])
        print("tools/list count:", len(tools))

        if not tools:
            print("ERROR: no tools returned")
            return 2

        return 0

    finally:
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            _, err = proc.communicate(timeout=1)
        except Exception:
            err = b""
        if err:
            # Print stderr for debugging (common failure mode in Claude Code as well)
            sys.stderr.write(err.decode("utf-8", errors="replace"))


if __name__ == "__main__":
    raise SystemExit(main())
