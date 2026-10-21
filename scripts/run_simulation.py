#!/usr/bin/env python3
"""Convenience script to run a single autonomous navigation simulation."""

import sys
from pathlib import Path

# Add src/ to path so script runs directly without requiring pip install -e .
repo_root = Path(__file__).resolve().parent.parent
src_dir = repo_root / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from autonomous_nav.main import main

if __name__ == "__main__":
    # Forward command-line arguments to main
    sys.exit(main())

