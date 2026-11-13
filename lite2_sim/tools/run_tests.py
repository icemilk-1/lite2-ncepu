"""Run the regression suite with a sandbox-safe temporary directory factory.

On this Windows host the DSH file sandbox denies access to directories created
with mode 0o700 (``tempfile.mkdtemp`` does that).  Tests that build a MuJoCo
scene inside ``tempfile.TemporaryDirectory`` therefore fail with
PermissionError even though the code is correct.  This runner replaces
``tempfile.mkdtemp`` with a variant that creates the directory through
``os.makedirs`` (which inherits the parent ACL) and then delegates to unittest.

Usage:
    python tools/run_tests.py [unittest args...]
    python tools/run_tests.py tests.test_navigation_core -v
"""

import os
import sys
import tempfile
import unittest


def _mkdtemp(suffix=None, prefix=None, dir=None):
    """``tempfile.mkdtemp`` equivalent that works under the DSH sandbox."""
    prefix = "tmp" if prefix is None else prefix
    suffix = "" if suffix is None else suffix
    base = tempfile.gettempdir() if dir is None else dir
    for _ in range(10000):
        name = prefix + next(tempfile._get_candidate_names()) + suffix
        path = os.path.join(base, name)
        try:
            os.makedirs(path)
        except FileExistsError:
            continue
        return path
    raise FileExistsError("No usable temporary directory name found")


def main(argv):
    # Keep temporary artifacts inside the writable workspace when possible so a
    # failure leaves inspectable output instead of vanishing with the sandbox.
    if os.environ.get("LITE2_TEST_TMPDIR"):
        os.makedirs(os.environ["LITE2_TEST_TMPDIR"], exist_ok=True)
        tempfile.tempdir = os.environ["LITE2_TEST_TMPDIR"]
    tempfile.mkdtemp = _mkdtemp

    project = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project not in sys.path:
        sys.path.insert(0, project)
    start = os.path.join(project, "tests")
    if argv and not argv[0].startswith("-"):
        names = argv
        loader = unittest.defaultTestLoader
        suite = loader.loadTestsFromNames(names)
        verbosity = 2
    else:
        loader = unittest.defaultTestLoader
        suite = loader.discover(start, pattern="test_*.py")
        verbosity = 2 if "-v" in argv else 1
    result = unittest.TextTestRunner(verbosity=verbosity).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))