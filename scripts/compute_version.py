#!/usr/bin/env python3
# ^ Shebang line. Lets this file be run directly as `./compute_version.py`
#   on systems where it's executable, by telling the OS which interpreter
#   to use.

import subprocess
# ^ Standard library module for running shell commands (git, in our case)
#   from inside Python and capturing their output.

import re
# ^ Standard library module for regex matching, which is used to parse version
#   numbers out of tag strings, and branch names out of commit messages.

import sys
# ^ Used only for sys.exit(0). This is to explicitly end the script
#   when there's nothing to do, rather than letting it fall through.


BRANCH_PATTERN = re.compile(r"(feature|fix)/[\w./-]+")
# ^ A compiled regex, defined once at module level so it's not re-compiled every time we check a commit message.
#      - A regex is a mini-language for describing patterns in text. You can search for, match, or extract things
#        without writing out every possible exact string by hand.
#      - A module in Python is just a fancy word for "a single .py file". So Module Level just refers to any code
#        that sits outside of any specific function.
#
#   Breaking down the pattern itself:
#     (feature|fix)   — captures either literal word, as a group we can
#                        retrieve later with match.group(1)
#     /               — a literal slash, matching the branch naming
#                        convention feature/... or fix/...
#     [\w./-]+        — one or more "branch name" characters: letters,
#                        digits, underscore (\w), dots, slashes, hyphens.


# Reads the MAJOR version number.
def get_delivery_number():
    with open("DELIVERY_VERSION") as f:
        return f.read().strip()


def get_last_tag():
    try:
        return subprocess.check_output(["git", "describe", "--tags", "--abbrev=0"]).decode().strip()
        #  -  subprocess.check_output runs the "git descrbibe --tags -- abbrev=0" command and returns its
        #     stdout as raw bytes
        #  -  '.decode()' converts bytes to a normal Python string
        #  -  `git describe --tags --abbrev=0` specifically returns the name of the most recent tag reachable from HEAD, with
        #     --abbrev=0 suppressing the extra "-N-gHASH" suffix git would otherwise append.
    except subprocess.CalledProcessError:
        # Exception to catch when there are NO tags in the repo yet (e.g. this is the very first release ever).
        return None

def get_merge_commit_subjects(last_tag):
    range = f"{last_tag}..HEAD" if last_tag else "HEAD"
    # Python ternary that builds the git commit range to search.
    # For example, if last_tag is "v2.4.1", range becomes "v2.4.1..HEAD" meaning the commits in between.
    # If last_tag is None (first-ever release), range is just "HEAD"

    return subprocess.check_output(["git", "log", range, "--merges", "--pretty=%s"]).decode().splitlines()
    # `git log <range> --merges --pretty=%s`:
    #   <range>     — the commit range computed above
    #   --merges    — filters output to ONLY actual merge commits
    #   --pretty=%s — for each matching commit, print ONLY the subject
    #                 line (the first line of the commit message),
    #                 nothing else (no hash, no author, no date)
    # .splitlines() turns the multi-line string output into a Python list, one string per merge commit subject.
    #
    # EX: [ "Merge pull request #101 from hytech-racing/vehicle-firmware/feature/oversample-adc",
    #       "Merge pull request #102 from hytech-racing/vehicle-firmware/fix/can-bus-timeout" ]

def bump_type(merge_subjects):
    branch_kinds_found = set()
    for subject in merge_subjects:
        match = BRANCH_PATTERN.search(subject)
        # .search() scans the entire subject line for the pattern
        if match:
            branch_kinds_found.add(match.group(1))
            # match.group(1) retrieves whatever the first parenthesized group in the regex captured.
            # This should be either the literal string "feature" or "fix"
            # Ex: Say, "feature/oversample-adc" was found, then only "feature" is added to the set.

    if "feature" in branch_kinds_found:
        return "minor"
    if "fix" in branch_kinds_found:
        return "patch"
    return None

def next_version(delivery, last_tag, kind):
    if last_tag:
        match = re.match(r"v?\d+\.(\d+)\.(\d+)", last_tag)
        # Parses the last tag string to pull out its minor and patch numbers
        #   v?        — An optional literal "v" prefix (matches both "v2.4.1" and "2.4.1")
        #   \d+       — How we ignore the the major number
        #   \.        — A literal dot
        #   (\d+)     — Captur the minor number
        #   \.        — A literal dot
        #   (\d+)     — Capture the patch number
        minor, patch = (int(match.group(1)), int(match.group(2))) if match else (0, 0)
        # Converts the captured strings to integers so we can do arithmetic on them.
        # Falls back to (0, 0) if the regex somehow didn't match
    else:
        minor, patch = 0, 0
        # For the very first tag.

    if kind == "minor":
        minor += 1
        patch = 0
        # Standard semver rule: bumping minor always resets patch to 0
    elif kind == "patch":
        patch += 1

    return f"v{delivery}.{minor}.{patch}"
    # Assembles the final version string, e.g. "v2.5.0"

if __name__ == "__main__":
    # This guard ensures the code below only runs when the file is
    # EXECUTED directly (python3 compute_version.py), not if it were
    # ever imported as a module from another script.

    delivery = get_delivery_number()
    last_tag = get_last_tag()
    subjects = get_merge_commit_subjects(last_tag)
    branch_kind = bump_type(subjects)

    if not branch_kind:
        print(last_tag or f"v{delivery}.0.0")
        sys.exit(0)
        # If nothing version-worthy happened, print the EXISTING tag unchanged or if there was never a tag at all,
        # print a sensible default starting version

    print(next_version(delivery, last_tag, branch_kind))
        # This is the ONLY output the CI workflow actually captures. This gets piped into
        # $GITHUB_OUTPUT via `echo "version=$(python3 compute_version.py)"`.