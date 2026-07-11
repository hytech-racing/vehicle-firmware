## Cloning/Installation Guide
See [Module 4](https://wiki.hytechracing.org/books/hytech-racing-general-info/page/training-module-4-intro-to-firmware-3-5-hours) on our BookStack for a guide on installing VSCode, PlatformIO, and cloning the repository.

## Building code
If you are not already familiar with PlatformIO, please read [Module 4](https://wiki.hytechracing.org/books/hytech-racing-general-info/page/training-module-4-intro-to-firmware-3-5-hours) first.

To BUILD code, open a PlatformIO terminal (CTRL-SHIFT-P, then `PlatformIO: New Terminal`) and call `pio run -e teensy41`. This will call the "build" command in the `teensy41` environment, which will check if your code compiles under the same compiler we will be using when uploading code to the car.

## Uploading code
If you are not already familiar with PlatformIO, please read [Module 4](https://wiki.hytechracing.org/books/hytech-racing-general-info/page/training-module-4-intro-to-firmware-3-5-hours) first.

To UPLOAD code, open a PlatformIO terminal (CTRL-SHIFT-P, then `PlatformIO: New Terminal`) and plug your laptop into the Teensy. Then, call `pio run -t upload -e teensy41`. This will call the "upload" command in the `teensy41` environment, uploading code.

## Testing code
If you are not already familiar with PlatformIO, please read [Module 4](https://wiki.hytechracing.org/books/hytech-racing-general-info/page/training-module-4-intro-to-firmware-3-5-hours) first.

To TEST code, open a PlatformIO terminal (CTRL-SHIFT-P, then `PlatformIO: New Terminal`). Then, to test systems (non hardware-specific logic), call `pio test -e test_systems_env`. This will run all unit tests in the `test/test_systems` directory.

## Check code

in the PlatformIO terminal:

`pio check -e teensy41 --fail-on-defect high`

## FAQ: Why red squiggles?
See [this](https://wiki.hytechracing.org/books/hytech-racing-general-info/page/vscode-setup-for-auto-completion-and-code-navigation) page on our BookStack for help configuring VSCode for auto completion and code navigation within external library dependencies.
