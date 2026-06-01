# Copyright (C) 2026 Mitsubishi Electric Research Laboratories (MERL)
#
# SPDX-License-Identifier: AGPL-3.0-or-later

"""Generate a conda-style environment.yaml from pyproject.toml."""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Iterable

import tomllib


def parse_project_metadata(pyproject_text: str) -> tuple[str, str | None, list[str]]:
    """Parse project metadata and merge dependencies from all project dependency tables."""
    parsed = tomllib.loads(pyproject_text)
    project = parsed.get("project")
    if not isinstance(project, dict):
        raise ValueError("Missing [project] table in pyproject.toml")

    project_name = project.get("name")
    if not isinstance(project_name, str) or not project_name:
        project_name = "generated-environment"

    requires_python = project.get("requires-python")
    if requires_python is not None and not isinstance(requires_python, str):
        raise ValueError("[project].requires-python must be a string if provided")

    dependencies: list[str] = []
    raw_dependencies = project.get("dependencies", [])
    if not isinstance(raw_dependencies, list):
        raise ValueError("[project].dependencies must be a TOML array")
    for dep in raw_dependencies:
        if not isinstance(dep, str):
            raise ValueError("All [project].dependencies entries must be strings")
        dependencies.append(dep)

    optional_dependencies = project.get("optional-dependencies", {})
    if not isinstance(optional_dependencies, dict):
        raise ValueError("[project.optional-dependencies] must be a TOML table")
    for group_deps in optional_dependencies.values():
        if not isinstance(group_deps, list):
            raise ValueError("Each [project.optional-dependencies] value must be a TOML array")
        for dep in group_deps:
            if not isinstance(dep, str):
                raise ValueError("All optional dependency entries must be strings")
            dependencies.append(dep)

    # Preserve order while dropping accidental duplicates.
    unique_dependencies = list(dict.fromkeys(dependencies))
    return project_name, requires_python, unique_dependencies


def yaml_quote(value: str) -> str:
    """Return a YAML-safe single-quoted scalar."""
    return "'" + value.replace("'", "''") + "'"


def render_environment_yaml(name: str, requires_python: str | None, deps: Iterable[str]) -> str:
    lines: list[str] = [f"name: {yaml_quote(name)}", "dependencies:"]

    if requires_python:
        lines.append(f"  - {yaml_quote(f'python{requires_python}')}")

    lines.append("  - pip")
    lines.append("  - pip:")
    for dep in deps:
        lines.append(f"      - {yaml_quote(dep)}")

    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pyproject", default="pyproject.toml", help="Path to pyproject.toml")
    parser.add_argument(
        "--output",
        default=".github/workflows/environment.yaml",
        help="Output path for generated environment.yaml",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pyproject_path = pathlib.Path(args.pyproject)
    output_path = pathlib.Path(args.output)

    project_text = pyproject_path.read_text(encoding="utf-8")
    project_name, requires_python, dependencies = parse_project_metadata(project_text)

    output = render_environment_yaml(project_name, requires_python, dependencies)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(output, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
