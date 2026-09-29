"""
gui/launcher/__main__.py — lets `python -m gui.launcher` keep working now
that gui.launcher is a package, not a single module.
"""
from __future__ import annotations

from gui.launcher.app import main

if __name__ == "__main__":
    main()
