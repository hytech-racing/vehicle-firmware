# test/

> **⚠️ Do not add files or folders here!!!**

Everything in this directory **except this README** is a **symbolic link (symlink)**, not a real directory. A symlink is a just a pointer to another folder.

Do **not**:
- Create files or folders in `test/`
- Delete or replace any symlinks
- Convert a symlink into a real directory

For a full explanation, see the bookstack page on Unit Testing.

---

## Directory Structure

```text
test/
├── README.md      # The only real file in this directory
├── acu -> ../ACU/test
├── ccu -> ../CCU/test
├── vcf -> ../VCF/test
└── vcr -> ../VCR/test
```

Each board entry is a symlink pointing to that board's actual `test/` directory.

For example:

```text
test/vcf/test_systems/foo.h
        │
        └───────────────► VCF/test/test_systems/foo.h
```

These are **the exact same file on disk**, simply accessible through two different paths.

---

## Why This Exists

PlatformIO always scans the monorepo root's `test/` directory before any board-specific redirection (via `scripts/set_directory.py`) occurs.

Without these symlinks:

```text
pio test
```

fails immediately with:

```text
Error: Nothing to build
```

even though every board already contains valid test code.

The symlinks expose each board's real test directory to PlatformIO without duplicating any files.

---

## Rules

### Edit tests in the real board directory

✅ Correct

```text
VCF/test/
ACU/test/
CCU/test/
VCR/test/
```

⚠️ Although editing through `test/vcf/...` works (it resolves to the same file), editing inside the real board directory keeps:

- editor navigation correct
- Git history (`git blame`)
- searches
- repository organization

centered on the canonical location.

---

### Never replace a symlink with a real directory

**Good**

```text
test/
└── vcf -> ../VCF/test
```

**Bad**

```text
test/
└── vcf/
```

Replacing a symlink creates a completely separate copy of the tests.

That causes Git to track two independent directories which will eventually drift out of sync.

---

### Adding a new board

Create exactly one symlink:

```bash
ln -s ../NEWBOARD/test test/newboard
```

Nothing else in this directory should be modified.

---

### If a symlink disappears

If a board appears as a real directory instead of a symlink:

```bash
find test -maxdepth 1
```

or

```bash
ls -l test
```

shows something like:

```text
vcf/
```

instead of

```text
vcf -> ../VCF/test
```

stop before running any tests.

1. Remove the incorrect directory.
2. Recreate the symlink.
3. Compare its contents with the real `<BOARD>/test/` directory to ensure nothing has diverged.

Do **not** assume the duplicate directory can simply be deleted without checking.