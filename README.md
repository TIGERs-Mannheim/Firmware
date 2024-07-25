# TIGERs Mannheim Open-Source Firmware
This repository contains the firmware for our v2020 robots and for our base station.

Important: Avoid whitespaces in all paths (repository, toolchain, OpenOCD)!

## Prerequisites

Under Windows, install/unpack the followings tools:
* gnu-arm-none-eabi GCC toolchain (tested version 12.3, zip download [here](https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-mingw-w64-i686-arm-none-eabi.zip?rev=e6948d78806d4815912a858a6f6a85f6&hash=B20A83F31B9938D5EF819B14924A67E3))
* git (available for windows [here](https://gitforwindows.org/))
* [cmake](https://cmake.org)
* [MinGW](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) `x86_64 8.1.0 posix seh` or any other source for make.exe (e.g. [MSYS2](https://www.msys2.org/))
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-win32-x64.zip) software for flashing and debugging
* (optional) tigerflash software for updating TIGERs hardware with a functional bootloader

Under Linux, install/unpack the following tools:
* gnu-arm-none-eabi GCC toolchain for x86_64 hosts (tested version 12.3, tar download [here](https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz?rev=dccb66bb394240a98b87f0f24e70e87d&hash=B788763BE143D9396B59AA91DBA056B6))
* For other hosts and silicon types (e.g. Apple/MacOS) please check [here](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads#panel2a). Use version 12.3 for bare-metal targets.
* git (via your preferred package manager)
* cmake (via your preferred package manager)
* make (via your preferred package manager)
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-x64.tar.gz) software for flashing and debugging
    * Copy udev rules to allow OpenOCD access to the USB interface. Go into the extracted openocd folder and run: `sudo cp contrib/60-openocd.rules /etc/udev/rules.d/`
* (optional) tigerflash software for updating TIGERs hardware with a functional bootloader

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
1. Flash v2020 robot or base station via OpenOCD
   ```
   cmake --build build -t flash-mb2019
   cmake --build build -t flash-bs2018
   ```
1. Flash v2020 robot or base station via tigerflash (working bootloader required)
   ```
   cmake --build build -t tigerflash-mb2019
   cmake --build build -t tigerflash-bs2018
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
    1. (optional) Click: New..., Name: TIGERS_TIGERFLASH, Value: The folder where you extracted tigerflash
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
1. There are different configurations, for the v2019 mainboard (mb2019) and for the Base Station (bs2018). All in debug and release configuration. The `All_Release` configuration will build all projects and can take some time.
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
1. Alternatively, if you have tigerflash set up, you can use the mb2019_Release_Flash or bs2018_Release_Flash build configurations which will automatically flash the corresponding device after build

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
1. In the drop-down list select the correct device. E.g. "Powerboard v1 (Interface 0)". For the mainboard and base station always select "Interface 0".
1. Select the WinUSB driver and click "Replace driver", confirm the installation
1. Close the tool, Done!

# Project Details

The firmware project contains all the code for our base station (BS) and for our robot (MB). There are different build configurations to select what to build. 

For each processor there is one folder below the "src" folder and a separate cmake projects. The base station has only one processor, called "bs2018".

For v2020 robots there is "main2019" for the primary microcontroller, "ir2019" for the infrared barrier processor and "motor2019" for motor controllers.

The Eclipse build configurations will exclude unused parts of the source files to make the Eclipse Indexer as happy as possible. Otherwise it would complain about multiple defines in the different processor include paths.

Furthermore, each processor has a custom bootloader. Therefore, the flash programming area has been divided into two parts. This allows the robot to be reprogrammed via our wireless interface. Although the base station also has a bootloader, this feature is currently not used.

The SWD adapter for programming and debugging is integrated on the Mainboard from v11 onwards and on the Base Station since v3 (2018). Only a micro USB cable is needed.
