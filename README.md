# TIGERs Mannheim Open-Source Firmware
This repository contains the firmware for our v2016 robots, v2020 robots and for our base station.

Important: Avoid whitespaces in all paths (repository, toolchain, OpenOCD)!

## Prerequisites

Under Windows, install/unpack the followings tools:
* gnu-arm-none-eabi GCC toolchain (tested version 8-2019-q3, zip download [here](https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-win32.zip?revision=2f0fd855-d015-423c-9c76-c953ae7e730b?product=GNU%20Arm%20Embedded%20Toolchain,ZIP,,Windows,8-2019-q3-update))
* git (available for windows [here](https://gitforwindows.org/))
* [cmake](https://cmake.org)
* [MinGW](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) x86_64 8.1.0 posix seh
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.11.0-4/xpack-openocd-0.11.0-4-win32-x64.zip) software for flashing and debugging

Under Linux, install/unpack the following tools:
* gnu-arm-none-eabi GCC toolchain (tested version 8-2019-q3, tar.bz2 download [here](https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2?revision=c34d758a-be0c-476e-a2de-af8c6e16a8a2?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,8-2019-q3-update))
* git (via your preferred package manager)
* cmake (via your preferred package manager)
* make (via your preferred package manager)
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.11.0-4/xpack-openocd-0.11.0-4-linux-x64.tar.gz) software for flashing and debugging
    * Copy udev rules to allow OpenOCD access to the USB interface. Go into the extracted openocd folder and run: `sudo cp contrib/60-openocd.rules /etc/udev/rules.d/`

# Command Line Usage

1. Linux build
   ```
   cmake -B build .
   cmake --build build -j
   ```
1. Windows build (assuming MinGW environment)
   ```
   cmake -B build -G "MinGW Makefiles" .
   cmake --build build -j
   ```
1. Flash v2020 robot, v2016 robot, or base station
   ```
   cmake --build build -t flash-mb2019
   cmake --build build -t flash-mb2016
   cmake --build build -t flash-bs2018
   ```

# IDE Usage
We use the Eclipse environment for development. The instructions have been tested with Eclipse 2022-03.

## Setup

1. Download Eclipse Embedded for your platform: [2022-03 Embedded](https://www.eclipse.org/downloads/packages/release/2022-03/r/eclipse-ide-embedded-cc-developers)
1. Unpack the file and start Eclipse
    1. Select a workspace. It should not contain any whitespaces!
1. Go to Help => Eclipse Marketplace. Install the following plugins:
    1. cmake4eclipse
    1. CMake Editor (optional, syntax highlighting for cmake files)
    1. DevStyle (optional, for a nice dark theme)
1. Restart Eclipse after plugin installation
1. In Eclipse open: Window => Preferences
    1. Go to: Run/Debug => String Substitution
    1. Click: New..., Name: TIGERS_ARM_TOOLCHAIN, Value: The folder where you extracted the ARM toolchain
    1. Click: New..., Name: TIGERS_OPENOCD, Value: The folder where you extracted OpenOCD
    1. (Windows only) Go to: C/C++ => Cmake4eclipse => General tab
    1. Change default build system to: MinGW Makefiles
    
## Project Import

1. In Eclipse select: File => Import...
1. Under "GIT" select "Projects from Git", hit Next
1. Select "Clone URI", hit Next
    1. For TIGERs members the URI is: `https://gitlab.tigers-mannheim.de/main/Firmware.git`. Enter your username and password under "Authentication", click Next.
    1. For the public release `https://github.com/TIGERs-Mannheim/Firmware.git`
1. Select at least the master branch, others are optional, click Next
1. Select a local directoy for the project, click Next
1. Wait for the download to finish and then select "Import existing projects", click Next
1. Select the Firmware project and click "Finish"

## Compiling
1. Right click on the Firmware project and go to: Build Configurations => Set Active
1. There are different configurations, for the v2016 mainboard (mb2016), v2019 mainboard (mb2019) and for the Base Station (bs2018). All in debug and release configuration. The `All_Release` configuration will build all projects and can take some time.
1. Choose the configuration you wish to build
1. Right click on the project and select Build Project
1. You can also select the build configuration and the build command in the toolbar. It is the small hammer symbol and the symbol left of it. Make sure you select the Firmware project before using the buttons.

## Flashing
1. Flashing of all processors is possible via a "Run Configuration"
1. Go to: Run => Run Configurations...
1. There are entries for flashing the base station or the mainboard
1. Just select the desired configuration and hit Run (requires OpenOCD to be setup correctly)
1. This can also be selected in the toolbar (white arrow in green circle)
1. All files required to flash the target are build automatically before the flash procedure starts

## Debugging
Each processor can be debugged individually
1. Go to: Run => Debug Configurations...
1. You can find all configurations already set up under the "GDB OpenOCD Debugging" node
1. Select the processor you wish to debug and click "Debug" (requires OpenOCD to be setup correctly)
1. The appropriate debug build image will automatically be loaded onto the corresponding processor
1. Happy debugging!

# Troubleshooting

### OpenOCD is unable to find device on Windows

1. To use the debugging hardware the WinUSB driver is required
1. This is most easily installed with Zadig's tool found here: http://zadig.akeo.ie/, download it and start it while your adapter is plugged in
1. Select: Options => List All Devices
1. In the drop-down list select the correct device. E.g. "Mainboard v11 (Interface 0)". For the mainboard and base station always select "Interface 0".
1. Select the WinUSB driver and click "Replace driver", confirm the installation
1. Close the tool, Done!

# Project Details

The firmware project contains all the code for our base station (BS) and for our robot (MB). There are different build configurations to select what to build. 

For each processor there is one folder below the "src" folder and a separate cmake projects. The base station has only one processor, called "bs2018". The robot mainboard uses another microcontroller which is called "main2016". 

For v2020 robots there is "main2019" for the primary microcontroller, "ir2019" for the infrared barrier processor and "motor2019" for motor controllers.

The Eclipse build configurations will exclude unused parts of the source files to make the Eclipse Indexer as happy as possible. Otherwise it would complain about multiple defines in the different processor include paths.

Furthermore, each processor has a custom bootloader. Therefore, the flash programming area has been divided into two parts. This allows the robot to be reprogrammed via our wireless interface. Although the base station also has a bootloader, this feature is currently not used.

The SWD adapter for programming and debugging is integrated on the Mainboard from v11 onwards and on the Base Station since v3 (2018). Only a micro USB cable is needed.
