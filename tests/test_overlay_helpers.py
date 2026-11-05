"""Regression tests for the facilities helper files of the overlay.

The overlay ships its own map_functions and priority_functions implementations.
Where the Vanetza tree already registers and ships a helper, the overlay must keep
the upstream file byte for byte: copying the older overlay helper over it would
undo the upstream signedness, allocation-safety and test fixes.  Only the helpers
the tree still lacks may be copied.  The file list is mapped from the registration
state that select-functions-patch.sh decides.

The last test class runs the real scripts/overlay.sh on a minimal isolated git
fixture.  The tests use the standard library only and need no Vanetza checkout.
"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from test_functions_patch import (
    BOTH_SOURCES,
    BOTH_TESTS,
    CURRENT_SOURCES,
    CURRENT_TESTS,
    PINNED_SOURCES,
    PINNED_TESTS,
    assert_registrations,
    write,
)

ROOT = Path(__file__).resolve().parents[1]
SELECT = ROOT / "scripts" / "select-functions-patch.sh"
FILES = ROOT / "scripts" / "functions-overlay-files.sh"
OVERLAY = ROOT / "scripts" / "overlay.sh"
PATCHES = ROOT / "patches"
ORIGINAL = PATCHES / "0002-facilities-add-spat-map-priority-functions.patch"
PRIORITY_ONLY = PATCHES / "0002-facilities-priority-only.patch"

MAP_FILES = (
    "vanetza/facilities/map_functions.hpp",
    "vanetza/facilities/map_functions.cpp",
    "vanetza/facilities/tests/map_functions.cpp",
)
PRIORITY_FILES = (
    "vanetza/facilities/priority_functions.hpp",
    "vanetza/facilities/priority_functions.cpp",
    "vanetza/facilities/tests/priority_functions.cpp",
)

class OverlayFilesTest(unittest.TestCase):
    def setUp(self):
        if shutil.which("sh") is None:
            self.skipTest("sh is not available")

    def make_tree(self, sources, tests):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        tree = Path(tmp.name)
        directory = tree / "vanetza" / "facilities"
        (directory / "tests").mkdir(parents=True)
        write(directory / "CMakeLists.txt", sources)
        write(directory / "tests" / "CMakeLists.txt", tests)
        return tree

    def files(self, tree, mode):
        return subprocess.run(
            ["sh", FILES.as_posix(), mode, tree.as_posix()],
            capture_output=True,
            text=True,
            check=False,
        )

    def listed(self, tree, mode):
        result = self.files(tree, mode)
        self.assertEqual(result.returncode, 0, result.stderr)
        return tuple(result.stdout.splitlines())

    def test_pinned_tree_copies_all_helpers(self):
        tree = self.make_tree(PINNED_SOURCES, PINNED_TESTS)
        self.assertEqual(self.listed(tree, "--copy"), MAP_FILES + PRIORITY_FILES)
        self.assertEqual(self.listed(tree, "--preserve"), ())

    def test_current_tree_copies_only_priority_helpers(self):
        tree = self.make_tree(CURRENT_SOURCES, CURRENT_TESTS)
        self.assertEqual(self.listed(tree, "--copy"), PRIORITY_FILES)
        self.assertEqual(self.listed(tree, "--preserve"), MAP_FILES)

    def test_tree_with_both_helpers_copies_nothing(self):
        tree = self.make_tree(BOTH_SOURCES, BOTH_TESTS)
        self.assertEqual(self.listed(tree, "--copy"), ())
        self.assertEqual(self.listed(tree, "--preserve"), MAP_FILES + PRIORITY_FILES)

    def test_rejected_tree_produces_no_files(self):
        sources = CURRENT_SOURCES.replace(
            "    path_history.cpp", "    path_history.cpp\n    priority_functions.cpp", 1
        )
        tree = self.make_tree(sources, CURRENT_TESTS)
        for mode in ("--copy", "--preserve"):
            with self.subTest(mode=mode):
                result = self.files(tree, mode)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("inconsistent priority registration", result.stderr)
                self.assertEqual(result.stdout, "")


class OverlayScriptTest(unittest.TestCase):
    """Run the real scripts/overlay.sh on a minimal isolated git fixture."""

    UPSTREAM_MARK = "// upstream helper\n"
    OVERLAY_MARK = "// overlay helper\n"

    def setUp(self):
        for tool in ("sh", "git", "tar", "mktemp"):
            if shutil.which(tool) is None:
                self.skipTest("{} is not available".format(tool))
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.src = self.root / "external" / "vanetza"

    def build(self, sources, tests, upstream_helpers):
        scripts = self.root / "scripts"
        scripts.mkdir(parents=True)
        for script in (OVERLAY, SELECT, FILES):
            shutil.copy2(script, scripts / script.name)
        patches = self.root / "patches"
        patches.mkdir()
        for patch in (ORIGINAL, PRIORITY_ONLY):
            shutil.copy2(patch, patches / patch.name)
        for name in MAP_FILES + PRIORITY_FILES:
            write(self.root / name, self.OVERLAY_MARK)
        (self.src / "vanetza" / "facilities" / "tests").mkdir(parents=True)
        write(self.src / "CMakeLists.txt", "cmake_minimum_required(VERSION 3.10)\n")
        write(self.src / "vanetza" / "facilities" / "CMakeLists.txt", sources)
        write(self.src / "vanetza" / "facilities" / "tests" / "CMakeLists.txt", tests)
        for name in upstream_helpers:
            write(self.src / name, self.UPSTREAM_MARK)
        self.git("init", "--quiet")
        self.git("config", "user.name", "test")
        self.git("config", "user.email", "test@example.invalid")
        self.git("add", "-A")
        self.git("-c", "commit.gpgsign=false", "commit", "--quiet", "-m", "upstream")

    def git(self, *args):
        result = subprocess.run(
            ["git", "-C", self.src.as_posix(), *args],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return result

    def overlay(self):
        return subprocess.run(
            ["sh", (self.root / "scripts" / "overlay.sh").as_posix()],
            capture_output=True,
            text=True,
            check=False,
        )

    def work(self):
        return self.root / "work" / "vanetza"

    def dst(self, name):
        return self.work() / name

    def assert_content(self, names, expected):
        for name in names:
            with self.subTest(file=name):
                self.assertEqual(read(self.dst(name)), expected)

    def test_current_upstream_keeps_its_map_helpers(self):
        if shutil.which("patch") is None:
            self.skipTest("patch is not available")
        self.build(CURRENT_SOURCES, CURRENT_TESTS, MAP_FILES)
        result = self.overlay()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_content(MAP_FILES, self.UPSTREAM_MARK)
        self.assert_content(PRIORITY_FILES, self.OVERLAY_MARK)
        assert_registrations(self, self.work(), 1, 1)

    def test_tree_with_both_helpers_keeps_every_upstream_helper(self):
        self.build(BOTH_SOURCES, BOTH_TESTS, MAP_FILES + PRIORITY_FILES)
        result = self.overlay()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_content(MAP_FILES + PRIORITY_FILES, self.UPSTREAM_MARK)
        assert_registrations(self, self.work(), 1, 1)

    def test_pinned_base_receives_the_overlay_helpers(self):
        if shutil.which("patch") is None:
            self.skipTest("patch is not available")
        self.build(PINNED_SOURCES, PINNED_TESTS, ())
        result = self.overlay()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_content(MAP_FILES + PRIORITY_FILES, self.OVERLAY_MARK)
        assert_registrations(self, self.work(), 1, 1)

    def test_priority_without_map_stops_the_overlay(self):
        sources = PINNED_SOURCES.replace(
            "    spat_functions.cpp", "    priority_functions.cpp\n    spat_functions.cpp", 1
        )
        tests = PINNED_TESTS.replace(
            "add_gtest(SpatFunctions ",
            "add_gtest(PriorityFunctions priority_functions.cpp)\nadd_gtest(SpatFunctions ",
            1,
        )
        self.build(sources, tests, ())
        result = self.overlay()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("without the MAP helpers", result.stderr)
        self.assertNotIn("overlay ready:", result.stdout)


def read(path):
    return path.read_text(encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
