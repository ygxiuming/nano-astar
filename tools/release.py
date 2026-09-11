#!/usr/bin/env python3
"""One-command release for nano-astar.

Bumps the version everywhere, stamps the ``## [Unreleased]`` section of
CHANGELOG.md with the new version and today's date, commits, tags, and
pushes. Pushing the tag triggers the Release workflow, which builds the
wheels, publishes to PyPI, and creates the GitHub Release with notes taken
from the changelog section.

Workflow:
    1. During development, jot user-facing changes under ``## [Unreleased]``
       in CHANGELOG.md (Keep a Changelog subsections: Added/Fixed/Changed...).
    2. When ready:  python tools/release.py 0.2.0

Usage:
    python tools/release.py X.Y.Z [--dry-run] [--no-push]
"""

from __future__ import annotations

import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = ROOT / "pyproject.toml"
INIT = ROOT / "src" / "python" / "nano_astar" / "__init__.py"
CHANGELOG = ROOT / "CHANGELOG.md"

SEMVER = re.compile(r"^\d+\.\d+\.\d+$")


def fail(msg: str) -> "None":
    sys.exit(f"error: {msg}")


def current_version() -> str:
    m = re.search(r'^version = "([^"]+)"', PYPROJECT.read_text(encoding="utf-8"), re.M)
    if not m:
        fail("could not find version in pyproject.toml")
    return m.group(1)


def stamp_changelog(version: str, today: str, apply: bool) -> None:
    text = CHANGELOG.read_text(encoding="utf-8")
    m = re.search(r"^## \[Unreleased\]\s*\n(.*?)(?=^## \[|\Z)", text, re.M | re.S)
    if not m:
        fail("no '## [Unreleased]' section in CHANGELOG.md — "
             "document the changes there first")
    if not m.group(1).strip():
        fail("'## [Unreleased]' section is empty — document the changes first")
    new_section = f"## [Unreleased]\n\n## [{version}] - {today}\n{m.group(1)}"
    if apply:
        CHANGELOG.write_text(
            text[: m.start()] + new_section + text[m.end():], encoding="utf-8"
        )


def bump_version(old: str, new: str, apply: bool) -> None:
    for path, pattern in (
        (PYPROJECT, rf'^version = "{re.escape(old)}"'),
        (INIT, rf'^__version__ = "{re.escape(old)}"'),
    ):
        text = path.read_text(encoding="utf-8")
        if not re.search(pattern, text, re.M):
            fail(f"version '{old}' not found in {path.relative_to(ROOT)}")
        if apply:
            updated = re.sub(pattern, lambda m: m.group(0).replace(old, new),
                             text, count=1, flags=re.M)
            path.write_text(updated, encoding="utf-8")


def git(*args: str, capture: bool = False) -> str:
    r = subprocess.run(["git", *args], cwd=ROOT,
                       capture_output=capture, text=True)
    if r.returncode != 0:
        fail(f"git {' '.join(args)} failed: {r.stderr.strip()}")
    return r.stdout.strip() if capture else ""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("version", help="new version, e.g. 0.2.0")
    ap.add_argument("--dry-run", action="store_true",
                    help="validate everything without writing files or git state")
    ap.add_argument("--no-push", action="store_true",
                    help="commit and tag locally but do not push")
    args = ap.parse_args()

    if not SEMVER.match(args.version):
        fail(f"'{args.version}' is not a semver X.Y.Z")
    old = current_version()
    if args.version == old:
        fail(f"version is already {old}")

    dirty = git("status", "--porcelain", capture=True)
    if dirty:
        fail("working tree is not clean — commit or stash your changes first:\n"
             + dirty)

    today = datetime.date.today().isoformat()
    apply = not args.dry_run
    stamp_changelog(args.version, today, apply)
    bump_version(old, args.version, apply)

    tag = f"v{args.version}"
    print(f"{old} -> {args.version}  ({today})")
    if args.dry_run:
        print("dry run: no files written, no commit/tag/push performed")
        return

    git("add", str(PYPROJECT.relative_to(ROOT)),
        str(INIT.relative_to(ROOT)), str(CHANGELOG.relative_to(ROOT)))
    git("commit", "-m", f"release: {tag}")
    git("tag", tag)
    if args.no_push:
        print(f"committed and tagged {tag} locally (not pushed)")
        return
    git("push", "origin", "HEAD")
    git("push", "origin", tag)
    print(f"pushed main + {tag}")
    print("Release workflow: "
          "https://github.com/ygxiuming/nano-astar/actions/workflows/release.yml")


if __name__ == "__main__":
    main()
