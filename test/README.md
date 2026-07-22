# Why this folder exists

This folder is a placeholder. It exists purely to satisfy a validation
check performed by PlatformIO's `pio test` command. It does not contain
any real tests, and nothing should ever be added here.

## The problem this solves

Before `pio test` runs a build for any environment, it first does an
upfront check: "does a test directory exist?" It looks for the global
`test_dir` option in the `[platformio]` section of `platformio.ini`
(or defaults to `./test` at the project root if that option isn't set).
This check is a simple existence check, and it happens *before* any
environment-specific configuration or scripts run.

Our repo has 5 subsystems and they keep their own tests in its own
folder (`ACU/test`, `CCU/test`, `VCF/test`, `VCR/test`), not in one shared
root-level `test/` folder. Because of that, there was no folder at
`vehicle-firmware/test` for PlatformIO's upfront check to find, which
caused `pio test` to fail immediately with:

    TestDirNotExistsError: A test folder '.../vehicle-firmware/test'
    does not exist.

This happened even though each environment's *actual* test directory is
set correctly, because the failure occurs at a check that runs earlier
than any of that per-environment logic.

## How the real per-subsystem redirection works

Once PlatformIO's upfront check passes, each native test environment
(`acu_test_systems`, `ccu_test_systems`, `vcf_test_systems`,
`vcr_test_systems`) runs `scripts/set_directory.py. That script
overrides `PROJECT_TEST_DIR` to point at the correct subsystem folder
(e.g. `ACU/test`) for the actual build and test compilation.

## Why an *empty* folder, specifically

We deliberately keep this folder empty (aside from this README) rather
than pointing the global `test_dir` at some other existing folder, like
the project root (`test_dir = .`). Pointing it at the project root
technically also satisfies the existence check, but it has a side
effect: PlatformIO's Library Dependency Finder uses `test_dir` not just
for the existence check, but also as the scope of what it scans for
`#include` statements and dependencies. Setting it to `.` widened that
scan to the entire monorepo, which caused PlatformIO to detect (and try
to install) libraries belonging to every subsystem, not just the one
actually being tested.

An empty folder avoids this entirely since there's nothing here for the
dependency scanner to find, so the scan stays effectively empty for any
environment relying on the global default.

## Where this matters in practice

This came up specifically because `scripts/prepush.sh` runs `pio test`
locally before pushing, across all subsystems. Without this folder, that
script fails immediately on the first native test environment it tries,
before ever reaching the actual test logic.