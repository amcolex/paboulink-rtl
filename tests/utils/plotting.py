from __future__ import annotations

from pathlib import Path


def module_plot_dir(module_file: str) -> Path:
    """Return (and create) the plots directory for a given test module."""
    module_path = Path(module_file).resolve()
    tests_root = module_path.parent
    for parent in module_path.parents:
        if parent.name == "tests":
            tests_root = parent
            break
    plots_root = tests_root / "plots"
    plots_dir = plots_root / module_path.stem
    plots_dir.mkdir(parents=True, exist_ok=True)
    return plots_dir
