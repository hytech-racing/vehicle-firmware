Import("env")

env_name    = env["PIOENV"]
project_dir = env["PROJECT_DIR"]

print(f">>> set_directory.py running for environment: {env_name}")

env_to_subsystem = {
    "ACU":                   "ACU",
    "ACU_test_systems":      "ACU",
    "ACU_spi_test":          "ACU",
    "CCU":                   "CCU",
    "CCU_test_systems":      "CCU",
    "VCF":                   "VCF",
    "VCF_test_systems":      "VCF",
    "VCF_test_orbis":        "VCF",
    "VCF_test_adcs":         "VCF",
    "VCF_test_can":          "VCF",
    "VCF_test_pedals":       "VCF",
    "VCR":                   "VCR",
    "VCR_test_systems":      "VCR",
    "VCR_ethernet_test":     "VCR",
    "VCR_test_can":          "VCR",
    "VCR_test_adcs":         "VCR",
    "Dashboard_H750_dfu":    "Dashboard",
    "Dashboard_H750_stlink": "Dashboard",
}

if env_name in env_to_subsystem:
    subsystem = env_to_subsystem[env_name]
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
    env.Append(LIBSOURCE_DIRS = [lib])
else:
    print(f">>> WARNING: {env_name} not found in map")