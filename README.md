# TIGERs Mannheim Open-Source Firmware
This repository contains the firmware for our v2016 robots, v2020 robots and for our base station.

# How to Use
This How-To will show you the required steps to setup your IDE, compile the TIGERs source code for the robot and our base station, and to debug it.

## System Requirements
Any windows or linux machine can be used. We tested Windows 7 x64, Windows 8.1 x64, Windows 10 x64, Ubuntu 18.04, and Arch Linux.

## IDE Setup
We use the Eclipse CDT environment for development. The instructions have been tested with Eclipse 2019-06.
1. Download Eclipse CDT for your platform: [2019-06 CDT](https://www.eclipse.org/downloads/packages/release/2019-06/r/eclipse-ide-cc-developers)
1. Unpack the CDT and start Eclipse
    1. Select a workspace. It should not contain any whitespaces!
1. Install the [GNU MCU Eclipse](https://gnu-mcu-eclipse.github.io/) plug-in
    1. In Eclipse select: Help => Install New Software...
    1. In the "Work with" field enter: http://gnu-mcu-eclipse.netlify.com/v4-neon-updates and hit enter
    1. Select at least the packages labeled "GNU MCU C/C++ ARM Cross Compiler" and "GNU MCU C/C++ OpenOCD Debugging"
    1. Click next, accept the licence, click finish. During the installation confirm the installation of unsigned content
    1. Alternatively, you may also install the plugin over the Eclipse Marketplace, search for "GNU MCU"
    1. Restart Eclipse when prompted

## ARM Toolchain
The ARM Toolchain contains the compiler, linker and debugger for the embedded microcontrollers. The recommended and mostly tested version is GCC 8.3. There are known issues with GCC 10.x.
1. Download the appropriate toolchain for your platform (we recommend the zip or tar.bz variant) from the [GNU ARM Embedded](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) project.
2. Unpack the toolchain to a location of your choice. Again, avoid whitespaces.

Notice to linux users: In addition to `arm-none-eabi-gcc`, installing `arm-none-eabi-newlib` is required for compiling the projects.

### Configure Toolchain
1. In eclipse open: Window => Preferences
1. Select: MCU => Global ARM Toolchains Paths
1. Next to the toolchain folder, click on browse and select the "bin" subfolder of where you installed/extracted your ARM toolchain
1. Click Apply and OK

## OpenOCD
OpenOCD is used as a debugging interface for embedded platforms. It forms the connection between GDB and the hardware. The tested version is 0.10.0 (build 10.3.2020).

The SWD adapter for programming and debugging is integrated on the Mainboard from v11 onwards and on the Base Station since v3. Only a micro USB cable is needed.


### Installation
1. Pre-built binaries for Windows, Linux (Debian-based), and OSX can be found [here](https://github.com/xpack-dev-tools/openocd-xpack/releases/) or more up-to-date for windows only [here](https://gnutoolchains.com/arm-eabi/openocd/).
1. For windows:
    1. Download a pre-built binary from [here](https://gnutoolchains.com/arm-eabi/openocd/)
    1. Unpack the content to a folder of your choice
    1. To use the debugging hardware the WinUSB driver is required
    1. This is most easily installed with Zadig's tool found here: http://zadig.akeo.ie/, download it and start it while your adapter is plugged in
    1. Select: Options => List All Devices
    1. In the drop-down list select the correct device. E.g. "Mainboard v11 (Interface 0)". For the mainboard and base station always select "Interface 0".
    1. Select the WinUSB driver and click "Replace driver", confirm the installation
    1. Close the tool, Done!
1. For Linux:
    1. Download a prebuild binary from: https://github.com/xpack-dev-tools/openocd-xpack/releases/ or run the following command:
       ```
       wget https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v0.10.0-14/xpack-openocd-0.10.0-14-linux-x64.tar.gz
       ```
    1. Copy udev rules to allow OpenOCD access to the USB interface. Go into the extracted openocd folder and run:
       ```
       sudo cp contrib/60-openocd.rules /etc/udev/rules.d/
       ```

### Configure OpenOCD
1. Open: Window => Preferences
1. Go to: MCU => Global OpenOCD Path
1. Set the executable to "openocd.exe" (on Windows) or "openocd" (on Linux).
1. Set the folder variable to the "bin" subfolder of your OpenOCD installation.
1. Click OK and close the window

# Project Setup and Usage
The firmware project contains all the code for our base station (BS) and for our robot (MB). There are different build configurations to select what to build. 

For each processor there is one folder below the "app" folder and a separate project. The base station has only one processor, called "bs2018". The robot mainboard uses another microcontroller which is called "main2016". 

For v2019 robots there is "main2019" for the primary microcontroller, "ir" for the infrared barrier processor and "motor" for motor controllers.

A separate project for each processor is required to make the Eclipse Indexer as happy as possible. Otherwise it would complain about multiple defines in the different processor include paths.

Furthermore, each processor has a custom bootloader. Therefore, the flash programming area has been divided into two parts. This allows the robot to be reprogrammed via our wireless interface. Although the base station also has a bootloader, this feature is currently not used.

## Import from Public Release
If you are using our public software release please follow the GIT instructions on the next paragraph and replace the repository URL with the one from GitHub.

## Import from GIT
If you are a member of the TIGERs team you can import the latest version from our GIT repository.
1. In Eclipse select: File => Import...
1. Under "GIT" select "Projects from Git", hit Next
1. Select "Clone URI", hit Next
1. The URI is: https://gitlab.tigers-mannheim.de/main/Firmware.git
1. Enter your username and password under "Authentication", click Next
1. Select at least the master branch, others are optional, click Next
1. Select a local directoy for the project, click Next
1. Wait for the download to finish and then select "Import existing projects", click Next
1. Select all projects and click "Finish"

## Compiling
1. Right click on the Firmware project and go to: Build Configurations => Set Active
1. There are six configurations, for the v2016 mainboard (MB2016), v2019 mainboard (MB2019) and for the Base Station (BS2018). Both in a debug and a release build.
1. Choose the one you wish to build
1. Right click on the project and select Build Project
1. You can also select the build configuration and the build command in the toolbar. It is the small hammer symbol and the symbol left of it. Make sure you select the Firmware project before using the buttons.
1. Building the project via the build command always compiles the bootloader and the run code. You may select a more fine grained control by using the configured make targets. If the "Make Target" view is not open, select it via Window => Show View. The make targets can be found in the Firmware folder.

## Flashing
1. Flashing of all processors is possible via a "Run Configuration"
1. Go to: Run => Run Configurations...
1. There are entries for flashing the base station or the mainboard (each with bootloader and run configuration)
1. Just select the desired configuration and hit Run (requires OpenOCD to be setup correctly)
1. If there is no code on the processors you will need to flash the bootloader first
1. This can also be selected in the toolbar (white arrow in green circle)

## Debugging
Each processor can be debugged individually
1. Go to: Run => Debug Configurations...
1. You can find all configurations already set up under the "GDB OpenOCD Debugging" node
1. Select the processor and mode (bootloader/run) you wish to debug and click "Debug" (requires OpenOCD to be setup correctly)
1. The appropriate debug build image will automatically be loaded onto the corresponding processor
1. Happy debugging!