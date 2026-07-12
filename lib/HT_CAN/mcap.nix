{
  fetchPypi,
  python311Packages,
}:

python311Packages.buildPythonPackage rec {
  pname = "mcap";
  version = "1.3.0";
  format = "pyproject";

  src = fetchPypi {
    inherit pname version;
    hash = "sha256-Xw1YJro7hBhQjB2ZK62kxXRAc/Gz5taFV58KygOJ+y8=";
  };

  nativeBuildInputs = with python311Packages; [
    setuptools
    wheel
    hatchling
  ];

  propagatedBuildInputs = with python311Packages; [
    numpy
    lz4
    zstandard
  ];

  doCheck = false;
  pythonImportsCheck = [ "mcap" ];
}
