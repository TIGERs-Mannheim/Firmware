# TIGERs Mannheim Open-Source Firmware
This repository contains the firmware for our v2020 robots and for our base station.

Important: Avoid whitespaces in all paths (repository, toolchain, OpenOCD)!

## Prerequisites

Under Windows, install/unpack the followings tools:
* gnu-arm-none-eabi GCC toolchain (tested version 12.3, zip download [here](https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-mingw-w64-i686-arm-none-eabi.zip?rev=e6948d78806d4815912a858a6f6a85f6&hash=B20A83F31B9938D5EF819B14924A67E3))
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-win32-x64.zip) software for flashing and debugging
* (optional) tigerflash software for updating TIGERs hardware with a functional bootloader
* The following tools are bundled in CLion. If you are not using CLion install them manually:
   * git (available for windows [here](https://gitforwindows.org/))
   * [cmake](https://cmake.org)
   * [MinGW](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) `x86_64 8.1.0 posix seh` or any other source for make.exe (e.g. [MSYS2](https://www.msys2.org/))

Under Linux, install/unpack the following tools:
* gnu-arm-none-eabi GCC toolchain for x86_64 hosts (tested version 12.3, tar download [here](https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-eabi.tar.xz?rev=dccb66bb394240a98b87f0f24e70e87d&hash=B788763BE143D9396B59AA91DBA056B6))
   * For other hosts and silicon types (e.g. Apple/MacOS) please check [here](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads#panel2a). Use version 12.3 for bare-metal targets.
* [OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.12.0-2/xpack-openocd-0.12.0-2-linux-x64.tar.gz) software for flashing and debugging
   * Copy udev rules to allow OpenOCD access to the USB interface. Go into the extracted openocd folder and run: `sudo cp contrib/60-openocd.rules /etc/udev/rules.d/`
* (optional) tigerflash software for updating TIGERs hardware with a functional bootloader
* The following tools are bundled in CLion. If you are not using CLion install them manually:
   * git (via your preferred package manager)
   * cmake (via your preferred package manager)
   * make (via your preferred package manager)

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

# CLion Usage
The instructions have been tested with CLion 2024.3.5.

## Setup
1. Download [Clion](https://www.jetbrains.com/de-de/clion/download/#section=windows)
   1. You can get a free license as a student from [GitHub Education](https://education.github.com/discount_requests/application) or directly from [JetBrains](https://www.jetbrains.com/community/education/#students)
1. Open and login with the account you have a license with
1. Add path variables:
   1. On the welcome screen, click the gear symbol in the lower left corner and select "Settings"
   1. Go to: Appearance & Behavior => Path Variables
   1. Click: +, Name: TIGERS_ARM_TOOLCHAIN, Value: The folder where you extracted the ARM toolchain
   1. Click: +, Name: TIGERS_OPENOCD, Value: The folder where you extracted OpenOCD
   1. (optional) Click: +, Name: TIGERS_TIGERFLASH, Value: The folder where you extracted tigerflash
   1. Note: If you do these steps after project import: delete the build directory and then click on CMake on the left toolbar, then reload the CMake Project
1. (optional) Add the "PlatformIO" plugin for serial connections

## Project Import
1. On the welcome screen click "Clone Repository"
   1. If git is not already installed click "Download and Install"
   1. Restart your PC, so git is in your Path
1. Paste the GIT URL
   1. For TIGERs members use: `https://gitlab.tigers-mannheim.de/main/Firmware.git` or `git@gitlab.tigers-mannheim.de:main/Firmware.git`    
      It will later ask you for a Token, for that click "Generate" and follow the instructions
   1. For public release use `https://github.com/TIGERs-Mannheim/Firmware.git`
1. (optional) Select the directory to save in
1. Click Clone

## Compiling

In the top right corner you can see two drop-down menus: CMake Profiles (first one) and Run Configurations (second one)

1. Select the "Release" CMake Profile for normal development
1. Go to the "Develop" subfolder in the Run Configuration drop-down:
   1. Select "Robot" or "Base Station" depending on your needs
   1. Select either the OpenOCD or Tigerflash variant depending on how you want to flash your target
   1. If you just want to compile the code you may choose any variant
1. Click the hammer symbol to compile the selected Profile + Run Configuration

## Flashing

1. Follow the steps under "Compiling" to select a Profile and Run Configuration
1. Click the green triangle to start the flash process

## Debugging

Note: Debugging requires OpenOCD and does not work with tigerflash.

1. Select the "Debug" CMake Profile in the top toolbar
1. In the Run Configuration drop-down, go to the Debug subfolder and:
   1. Select MB to debug a robot or BS to debug a base station
   1. Select Boot or Run depending on if you want to debug the bootloader or the application program
1. Click the green bug symbol to flash the debug build and start debugging


# Eclipse Usage
Using Eclipse is deprecated. It is recommended to use CLion for the best development experience. In particular, the Eclipse Indexer is often confused by the multi-project setup. Nevertheless, all tasks can  be accomplished with Eclipse.
The instructions have been tested with Eclipse 2022-03.

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
    1. For TIGERs members the URI is: `https://gitlab.tigers-mannheim.de/main/Firmware.git` or `git@gitlab.tigers-mannheim.de:main/Firmware.git`.    
       Enter your username and password under "Authentication", click Next.
    1. For the public release `https://github.com/TIGERs-Mannheim/Firmware.git`
1. Select at least the master branch, others are optional, click Next
1. Select a local directory for the project, click Next
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

## Debugging
Each processor can be debugged individually
1. Go to: Run => Debug Configurations...
1. You can find all configurations already set up under the "GDB OpenOCD Debugging" node
1. Select the processor you wish to debug and click "Debug" (requires OpenOCD to be setup correctly)
1. The appropriate debug build image will automatically be loaded onto the corresponding processor
1. Happy debugging!



# Troubleshooting

## Clion
### Cmake config "Default" finds no compiler
Delete the cmake config:
1. Go to File | Settings | Build, Execution, Deployment | CMake
2. Select the "Default" configuration.
3. Click - to delete the config.

## Other
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

Each processor has a custom bootloader. Therefore, the flash programming area has been divided into two parts. This allows the robot to be reprogrammed via our wireless interface. Although the base station also has a bootloader, this feature is currently not used.

The SWD adapter for programming and debugging is integrated on the Mainboard from v11 onwards and on the Base Station since v3 (2018). Only a micro USB cable is needed.
