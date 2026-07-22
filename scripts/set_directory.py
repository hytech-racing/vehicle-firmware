# This script runs before a build starts. It points PROJECT_SRC_DIR / TEST_DIR /
# INCLUDE_DIR / LIBSOURCE_DIRS at the right subsystem folder (ACU, CCU, VCF, VCR,
# Dashboard) instead of PlatformIO's defaults (./src, ./test, ./include, ./lib).
# This is because this repo doesn't have a separate ini file per board and each
# board has a ./src, ./test, ./include, and ./lib.
#
# Each [env:...] section must explicitly declare its subsystem, e.g.:
#     custom_subsystem = ACU


Import("env")
# Pulls the shared SCons/PlatformIO build config into this script.
# Config is from the "Environment" object SCons creates
# NOTE: "env" acts like a dictonary with key-value pairs

project_dir = env["PROJECT_DIR"]  # Absolute path to the project root (e.g. where platformio.ini lives).
env_name    = env["PIOENV"]       # Name of the environment currently being built (string), e.g. "acu-prod".

print(f">>> set_directory.py running for environment: {env_name}")

# Every environment must declare custom_subsystem; missing it is a config
# mistake, so fail loudly rather than silently falling back to PlatformIO defaults.
try:
    subsystem = env.GetProjectOption("custom_subsystem")
except Exception:
    print(f">>> ERROR: [{env_name}] is missing 'custom_subsystem' in platformio.ini")
    raise

src     = f"{project_dir}/{subsystem}/src"
test    = f"{project_dir}/{subsystem}/test"
include = f"{project_dir}/{subsystem}/include"
lib     = f"{project_dir}/{subsystem}/lib"

print(f">>> Setting PROJECT_SRC_DIR     = {src}")
print(f">>> Setting PROJECT_TEST_DIR    = {test}")
print(f">>> Setting PROJECT_INCLUDE_DIR = {include}")

env.Replace(PROJECT_SRC_DIR     = src)
env.Replace(PROJECT_TEST_DIR    = test)
env.Replace(PROJECT_INCLUDE_DIR = include)

# Here we Append instead of Replace since LIBSOURCE_DIRS may already hold other paths
# (e.g. global libs) that should still be searched.
env.Append(LIBSOURCE_DIRS = [lib])