"""Quick local smoke-test for the frc41 MCP server module.

This doesn't fully speak the MCP protocol (that's handled by Claude/clients),
but it verifies that:
- Python deps are installed
- the server module imports
- tool registration runs

Run with the repo venv Python:
  ./.venv/Scripts/python.exe mcp/smoke_test.py
"""

from __future__ import annotations

import importlib


def main() -> None:
    mod = importlib.import_module("mcp.server")
    assert mod is not None

    # Import the frc41 server entrypoint (mcp/server.py).
    import server  # type: ignore  # noqa: F401

    print("OK: imported mcp + frc41 server module")


if __name__ == "__main__":
    main()
