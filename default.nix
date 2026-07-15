{ pkgs, python311Packages, fetchPypi, python3 }:

pkgs.stdenv.mkDerivation rec {
  name = "ht_can_pkg";

  src = ./PCAN_project;

  buildInputs = [
    (python311Packages.cantools.overridePythonAttrs (old: {
      src = fetchPypi {
        pname = "cantools";
        version = "39.4.4";
        hash = "sha256-bo6Ri2ZxpiqfOZBUbs5WI+Hetx3vsc74WplVrDAdqZ4=";
      };
      doCheck = false;
    }))

    (pkgs.callPackage ./mcap.nix { })

  ]; # Python as a build dependency

  propagatedBuildInputs = buildInputs;

  nativeBuildInputs = [ python3 ] ++ buildInputs;

  # Define the build phase to execute the scripts
  buildPhase = ''
    python3 -m cantools convert hytech.sym hytech.dbc
  '';

  # Specify the output of the build process
  # In this case, it will be the generated file
  installPhase = ''
    mkdir -p $out
    mv hytech.dbc $out/hytech.dbc
  '';
}
