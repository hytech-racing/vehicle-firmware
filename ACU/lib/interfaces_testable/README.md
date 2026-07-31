# interfaces_testable/ — symlink folder, not a real library

Everything in this folder except this README is a **symlink**, not a real
file. Do not copy real files in here, and do not edit through these links
without knowing they point straight back into `../interfaces/`.

## What's actually here

interfaces_testable/
├── README.md                          ← the only real file
├── include/
│   ├── ADCInterface.h       -> ../../interfaces/include/ADCInterface.h
│   ├── MAX114XInterface.h   -> ../../interfaces/include/MAX114XInterface.h
│   └── MAX114XInterface.tpp -> ../../interfaces/include/MAX114XInterface.tpp
└── src/
    └── ADCInterface.cpp     -> ../../interfaces/src/ADCInterface.cpp

Editing a file through either path edits the exact same file on disk —
there is no duplicate copy of anything here, and nothing can drift out
of sync with `../interfaces/`.

## Why this folder exists

PlatformIO's Library Dependency Finder (LDF) discovers dependencies at
folder granularity, not file granularity. Including ADCInterface.h alone
was pulling in every hardware-only file in ../interfaces/ (CAN, Ethernet,
SD, EEPROM, LTC SPI), none of which ADC logic actually depends on.
lib_ignore can only exclude whole folders, so this smaller folder of
symlinks gives it just the files that are actually testable.

## Why symlinks instead of moving the real files

No duplication/drift risk, no changes to the real interfaces/ folder
that acu-prod and hardware builds still rely on, and no #ifdef ARDUINO
scattered through application logic just to unblock one class.

## Adding a new testable file later

ln -s ../../interfaces/include/SomeInterface.h ACU/lib/interfaces_testable/include/SomeInterface.h

Always verify with `cat`, not just `ls -la` — a broken symlink can still
look fine in a directory listing.