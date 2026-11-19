# test/

**Do not delete this folder.** It is intentionally empty (apart from this README and `.gitkeep`).

Each board's real tests live in `<BOARD>/test/` (e.g. `VCR/test/`). `scripts/set_directory.py`
points every environment at its board's test folder, but it runs only once a build starts.
`pio test` first checks that the project-level test folder (`./test`, PlatformIO's default)
exists, and fails with `TestDirNotExistsError` if it does not. Git does not track empty
folders, so `.gitkeep` keeps this one in the repo.
