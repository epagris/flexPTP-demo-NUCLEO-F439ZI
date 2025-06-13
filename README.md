# STM32 NUCELO-F439ZI flexPTP demo

<!-- <img src="Modules/flexptp/manual/media/flexPTP_logo.png" alt="flexPTP-logo" width="200">
<img src="https://www.st.com/bin/ecommerce/api/image.PF266519.en.feature-description-include-personalized-no-cpn-large.jpg" alt="NUCLEO-F439ZI" width="200"> -->

![flexPTP CLI](NUCLEO-H743ZI.gif)

## What's this?

> **This is a [flexPTP](https://github.com/epagris/flexPTP) demo project showcasing the capabilities of the flexPTP [IEEE 1588 Precision Time Protocol](https://ieeexplore.ieee.org/document/9120376) implementation for the [STMicroelectronics NUCLEO-F439ZI](https://www.st.com/en/evaluation-tools/nucleo-f439zi.html) devboard.**

Still not clear what is it useful for? No worries, it's a behind-the-scenes support technology that use unaware every day if you have a smartphone or when you are connected to the internet. Modern telecommunication and measurement systems often rely on precise time synchronization down to the nanoseconds' scale. Methods got standardized by the IEEE and now it's known by the name of the Precision Time Protocol. This software project is an evaluation environment to showcase the capabilities of our IEEE 1588 PTP implementation named `flexPTP` on the STMicroelectronics NUCLEO-F439ZI board.

> [!TIP]
>**Just want to try the demo and skip compiling? Download one of the precompiled binaries and jump to [Deploying](#Deploying)!**


### Get the sources

> [!NOTE]
> To acquire the full source tree after cloning the repo, please fetch the linked *submodules* as well:

```
git clone https://gitea.epagris.com/epagris/flexPTP-demo-NUCLEO-F439ZI.git
cd flexPTP-demo-NUCLEO-F439ZI
git submodule init
git submodule update
```

## Building

### Prerequisites

The following two pieces of software are necessary for building:
- `arm-none-eabi-gcc` (v12+): the GCC ARM Cortex-M C cross-compiler
- `cmake` (v3.22+): KitWare's build management system

> [!NOTE]
> Both applications are available on common Linux systems through the official package repository. For building on Windows we recommend to install the [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) bundle shipping the `arm-none-eabi-{gcc|gdb}`, `openocd` and ST's proprietary STLink downloading tools. The [CMake](https://cmake.org/) needs to be installed separately.

### Compiling

The project is fully CMake managed. Configure and invoke the cross-compiler using the commands below:

```
cmake . -B build
cmake --build build --target flexptp-demo --
```
Once the building has concluded the output binaries would be deposited in the `build` directory: `flexptp-demo.elf`, `flexptp-demo.bin`, `flexptp-demo.hex`

## Deploying

### Downloading the firmware

The compiled binaries can be downloaded onto the devboard through several tools:

### Using the STM32CubeProgrammer

The [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) recognizes the MCU once connected. Any of the generated binary output can be used for programming on the *Erasing & Programming* pane.

### Using the `st-flash` utility

This tool is part of the open source [stlink-tools](https://github.com/stlink-org/stlink) bundle, that is also available through common Linux package managers.

To program the MCU use the following command: `st-flash write build/flexptp-demo.elf 0x08000000`

### Using the `openocd` application

The [OpenOCD](https://openocd.org/) programming/debugging tool can also be used to upload the firmware using the following command:

`openocd -f "board/st_nucleo_f4.cfg" -c init -c halt -c "program build/flexptp-demo.elf" -c exit`

OpenOCD is also available through the common Linux package managers.

### Interacting with the firmware

The firmware prints its messages to and expect user input coming on the board controller's virtual serial port using the `115200-8-N-1` configuration. Use [Putty](https://www.putty.org/) or any equivalent (e.g. GtkTerm) serial port terminal to communicate with the firmware. On Linux, the device will show up as `/dev/ttyACMx`.

> [!NOTE]
> Read the firmware's bootup hints and messages carefully!

### PPS signal

The 1PPS signal is emitted on the `PG0` pin.

## Development

An all-around [Visual Studio Code](https://code.visualstudio.com/) project is packaged along the project to enable easy development, debugging and editing. To enable these powerful features, install the [STM32Cube for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension), [CMakeTools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools), [Cortex-Debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug), [Embedded Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-embedded-tools) extensions, [OpenOCD](https://openocd.org/) and the [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) software package. The latter one is required for downloading the firmware right away using the Cortex-Debug extension (if not done through OpenOCD) and for providing the Embedded Tools extension with register description `.SVD` files. [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) and clang-format are highly recommended.

Thas project has predefined *Launch* and *Attach* tasks.

### Software structure

The project is relying on the following large software building blocks:
- the [FreeRTOS](https://www.freertos.org/) embedded operating system,
- the [CMSIS RTOS V2](https://arm-software.github.io/CMSIS_6/latest/RTOS2/index.html) module as a wrapper for FreeRTOS,
- [STM32F4xx Hardware Abstraction Layer library](https://github.com/STMicroelectronics/stm32f4xx-hal-driver) (HAL) providing MCU specific functionality,
- the [Lightweight IP](https://github.com/lwip-tcpip/lwip) (lwip) Ethernet stack extended with PTP timestamp support,
- the [EtherLib](https://gitea.epagris.com/epagris/EtherLib), our in-house developed Ethernet stack,
- the [embfmt](https://gitea.epagris.com/epagris/embfmt) a printf()-like formatted printer implementation for embedded systems.

> [!TIP]
> The project can either use LwIP or EtherLib Ethernet stacks. This can be easily changed by setting the `ETH_STACK` CMake variable in the top `CMakeLists.txt`:
> ```
>set(ETH_STACK "LWIP") # select "LWIP" or "ETHERLIB"
>```
> The development of the EtherLib network stack was motivated in the beginning by just curiosity and our need for something that is suitable for our university research. Gradually it became a handy tool so we've decided to promote it into our public projects as well.
>

The project is organized the following way:

```
ROOT
  Drivers
    CMSIS: ARM CMSIS-related files (CMSIS RTOS V2 and device headers)
    EthDrv: a custom Ethernet driver (with PTP timestamp support), a PHY-interface and network stack interfaces
    stm32f4xx-hal-driver: ST's STM32F4xx HAL driver files (submodule)
  Inc: headers for compile-time library configuration
  Middlewares: FreeRTOS CMake configuration
  Modules
    flexptp: our PTP implementation (submodule)
    lwip and lwip_port: the LwIP Ethernet stack and its system dependent module (submodule)
    etherlib: our in-house developed Ethernet stack (submodule)
    embfmt: a printf()-like formatted printer implementation for embedded systems (submodule)
    blocking_io: a simple blocking FIFO implementation
  Src
    cliutils: CLI-interface implementation
    ethernet: Ethernet stack initialization

    cmds.c: custom CLI commands
```
> [!NOTE]
> The flexPTP parameters are defined in the [flexptp_options.h](Inc/flexptp_options.h) header.

### Printing and logging

In this project the memory-heavy `printf()` is replaced by the more embedded-optimized `MSG()` function backed by the `embfmt` library. Parameters and format specifiers are fully `printf()` compatible.

### CLI commands

The software offers you with the following multitude, most flexPTP-related of commands:

```
?                                                  Print this help (22/48)
hist                                               Print command history
osinfo                                             Print OS-related information
flexptp                                            Start flexPTP daemon
ptp pps {freq}                                     Set or query PPS signal frequency [Hz]
ptp servo params [Kp Kd]                           Set or query K_p and K_d servo parameters
ptp servo log internals {on|off}                   Enable or disable logging of servo internals
ptp reset                                          Reset PTP subsystem
ptp servo offset [offset_ns]                       Set or query clock offset
ptp log {def|corr|ts|info|locked|bmca} {on|off}    Turn on or off logging
time [ns]                                          Print time
ptp master [[un]prefer] [clockid]                  Master clock settings
ptp info                                           Print PTP info
ptp domain [domain]                                Print or get PTP domain
ptp addend [addend]                                Print or set addend
ptp transport [{ipv4|802.3}]                       Set or get PTP transport layer
ptp delmech [{e2e|p2p}]                            Set or get PTP delay mechanism
ptp transpec [{def|gPTP}]                          Set or get PTP transportSpecific field (majorSdoId)
ptp profile [preset [<name>]]                      Print or set PTP profile, or list available presets
ptp tlv [preset [name]|unload]                     Print or set TLV-chain, or list available TLV presets
ptp pflags [<flags>]                               Print or set profile flags
ptp period <delreq|sync|ann> [<lp>|matched]        Print or set log. periods
ptp coarse [threshold]                             Print or set coarse correction threshold
ptp priority [<p1> <p2>]                           Print or set clock priority fields
```

> [!TIP]
> The above hint can be listed by typing '?'.

## Notes

> [!WARNING]
> By default, the MCU clock is sourced by the onboard (STLink) board controller on this devboard. According to our observations, this clock signal is loaded with heavy noise rendering the clock synchronization unable to settle precisely. We highly recommend to solder a 8 MHz oscillator onto  the designated X3 pads to achieve the best results!


## Related papers and references

[Time Synchronization Extension for the IO-Link Industrial Communication Protocol](https://ieeexplore.ieee.org/document/10747727)

[Distributed Measurement System for Performance Evaluation of Embedded Clock Synchronization Solutions](https://ieeexplore.ieee.org/document/9805958/)

[Portable, PTP-based Clock Synchronization Implementation for Microcontroller-based Systems and its Performance Evaluation](https://ieeexplore.ieee.org/document/9615250)

[Synchronization of Sampling in a Distributed Audio Frequency Range Data Acquisition System Utilizing Microcontrollers](https://ieeexplore.ieee.org/document/9918455/)

[Methods of Peripheral Synchronization in Real-Time Cyber-Physical Systems](https://ieeexplore.ieee.org/document/10178979/)



## License

The project is created by András Wiesner (Epagris) in 2025 and published under the MIT license. Contributions are welcome! :)



