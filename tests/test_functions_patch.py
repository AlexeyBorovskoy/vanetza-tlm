"""Regression tests for selecting the facilities patch 0002 variant.

The pinned Vanetza base needs the original patch, which registers the MAP and
priority helpers together.  Current upstream already registers the MAP helpers,
so only priority may be added there.  These tests use synthetic CMake fragments
and the standard library only; they do not need a Vanetza checkout.
"""

import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SELECT = ROOT / "scripts" / "select-functions-patch.sh"
PATCHES = ROOT / "patches"
ORIGINAL = PATCHES / "0002-facilities-add-spat-map-priority-functions.patch"
PRIORITY_ONLY = PATCHES / "0002-facilities-priority-only.patch"

PINNED_SOURCES = """\
set(CXX_SOURCES
    cam_functions.cpp
    path_history.cpp
    path_point.cpp
    spat_functions.cpp
)

"""

PINNED_TESTS = """\
include(UseGTest)

configure_gtest_directory(LINK_LIBRARIES facilities)

add_gtest(CamFunctions cam_functions.cpp LINK_LIBRARIES asn1_its_r2)
add_gtest(PathHistory path_history.cpp)
add_gtest(PathPoint path_point.cpp)
add_gtest(SpatFunctions spat_functions.cpp)
"""

CURRENT_SOURCES = """\
set(CXX_SOURCES
    cam_functions.cpp
    map_functions.cpp
    path_history.cpp
    path_point.cpp
    spat_functions.cpp
)

"""

CURRENT_TESTS = """\
include(UseGTest)

configure_gtest_directory(LINK_LIBRARIES facilities)

add_gtest(CamFunctions cam_functions.cpp LINK_LIBRARIES asn1_its_r2)
add_gtest(MapFunctions map_functions.cpp)
add_gtest(PathHistory path_history.cpp)
add_gtest(PathPoint path_point.cpp)
add_gtest(SpatFunctions spat_functions.cpp)
"""


class FunctionsPatchTest(unittest.TestCase):
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

    def select(self, tree):
        return subprocess.run(
            ["sh", SELECT.as_posix(), tree.as_posix()],
            capture_output=True,
            text=True,
            check=False,
        )

    def assert_selected(self, tree, expected):
        result = self.select(tree)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(Path(result.stdout.strip()).name, expected.name)

    def test_pinned_tree_selects_original_patch(self):
        tree = self.make_tree(PINNED_SOURCES, PINNED_TESTS)
        self.assert_selected(tree, ORIGINAL)

    def test_current_tree_selects_priority_only_patch(self):
        tree = self.make_tree(CURRENT_SOURCES, CURRENT_TESTS)
        self.assert_selected(tree, PRIORITY_ONLY)

    def test_inconsistent_map_registration_fails(self):
        cases = [
            (CURRENT_SOURCES, PINNED_TESTS),
            (PINNED_SOURCES, CURRENT_TESTS),
        ]
        for sources, tests in cases:
            with self.subTest(sources=sources, tests=tests):
                result = self.select(self.make_tree(sources, tests))
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("inconsistent MAP registration", result.stderr)
                self.assertEqual(result.stdout, "")

    def test_existing_priority_registration_fails(self):
        sources = CURRENT_SOURCES.replace(
            "    path_history.cpp", "    priority_functions.cpp\n    path_history.cpp"
        )
        tests = CURRENT_TESTS.replace(
            "add_gtest(PathHistory",
            "add_gtest(PriorityFunctions priority_functions.cpp)\nadd_gtest(PathHistory",
        )
        result = self.select(self.make_tree(sources, tests))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already registered", result.stderr)

    def test_original_patch_applies_to_pinned_tree(self):
        tree = self.make_tree(PINNED_SOURCES, PINNED_TESTS)
        self.apply(tree, ORIGINAL)
        assert_registrations(self, tree, map_expected=1, priority_expected=1)

    def test_priority_only_patch_applies_to_current_tree(self):
        tree = self.make_tree(CURRENT_SOURCES, CURRENT_TESTS)
        self.apply(tree, PRIORITY_ONLY)
        assert_registrations(self, tree, map_expected=1, priority_expected=1)

    def apply(self, tree, patch):
        if shutil.which("patch") is None:
            self.skipTest("patch is not available")
        # tests write LF files; do not let a CRLF checkout of the patch mask itself
        content = patch.read_bytes().replace(b"\r\n", b"\n")
        result = subprocess.run(
            ["patch", "-d", tree.as_posix(), "-p1", "--forward", "--quiet"],
            input=content,
            capture_output=True,
            check=False,
        )
        self.assertEqual(
            result.returncode, 0, (result.stdout + result.stderr).decode("utf-8", "replace")
        )


def write(path, text):
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def assert_registrations(test, tree, map_expected, priority_expected):
    directory = tree / "vanetza" / "facilities"
    sources = (directory / "CMakeLists.txt").read_text(encoding="utf-8")
    tests = (directory / "tests" / "CMakeLists.txt").read_text(encoding="utf-8")
    test.assertEqual(sources.count("map_functions.cpp"), map_expected)
    test.assertEqual(sources.count("priority_functions.cpp"), priority_expected)
    test.assertEqual(tests.count("add_gtest(MapFunctions "), map_expected)
    test.assertEqual(tests.count("add_gtest(PriorityFunctions "), priority_expected)


if __name__ == "__main__":
    unittest.main()
