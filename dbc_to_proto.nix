{ lib, python311Packages }:

python311Packages.buildPythonApplication {
  pname = "py_dbc_proto_gen_pkg";
  version = "1.0.0";

  propagatedBuildInputs = [ (python311Packages.cantools.overridePythonAttrs (_: { doCheck = false; })) python311Packages.protobuf python311Packages.requests ];

  pyproject = true;
  build-system = [ python311Packages.setuptools ];
  src = ./dbc_to_proto;
}