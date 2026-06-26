# TIGERs Mannheim Wireless Communication

TL;DR Feature List
- 2.4GHz solution based on SX1280 from Semtech
- Uses a single transceiver on robot/base station
- Requires only a single 1.2MHz wide channel
- Conflict-free TDMA scheduling
- Bi-directional half-duplex transfers
- Broadcast option to send same data to all robots
- Robots can receive data from other robots
- 100% predictable upper latency
- Robust and proven since 2018, performance improved 2025
- Detailed statistics available (e.g. update rate, packet loss, signal strength)

For details on how to compile this firmware please check out the [README](README.md).

For details on the hardware please check out our electronics open-source release.

# Specifications

| Parameter                   | Symbol  | Value               | Notes                                                |
|-----------------------------|---------|---------------------|------------------------------------------------------|
| Time per TDMA slot          | T_slot  | ~540us              | 466us TX, 614us RX (worst case), measured            |
| Update Cycle                | F_upd   | ~80Hz               | Update frequency for 11 robots + 1 broadcast         |
| Round-trip latency          | RTT     | 1080us              | maximum, guarenteed by design                        |
| Start Up Time               | T_start | <0.5s               | estimated                                            |
| Power Consumption           | P_in    | <0.5W               | estimated from SX1280 and SKY66112 datasheet         |
| Data bitrate (over-the-air) | DR_ota  | 1300kb/s            | From SX1280 datasheet                                |
| Effective payload data rate | DR_pl   | ~77.8 kB/s          | 1s/T_slot * 42B                                      |
| Frequency Range             | F       | 2.300GHz - 2.555GHz |                                                      |
| Modulation                  | M       | FLRC                |                                                      |
| Output power                | P_RF    | 20dBm               |                                                      |
| Bandwidth                   | BW      | 1.2MHz              | From SX1280 datasheet                                |
| Detect Interference         | -       | No                  |                                                      |
| Cost                        | -       | $10 USD             | estimated, based on SX1280 and SKY66112 market price |

## Data recording

Our base station sends out a statistics frame with 10Hz on the ethernet port. This frame is send to our controlling
central software and recorded there in a standard SSL gamelog with a custom identifier. The definition can be found
[here](src/shared/commands.h) at the `BaseStationWifiStatsV2` struct.

The statistics frame contains data about packet loss, signal strength, etc. of all robots. Post-processing the
gamelog after a match allows to retrieve detailed metrics about wireless performance.

Measured performance will be published here once it is available.

# How it actually works

The following section outlines how our wireless communication works. It starts at the hardware level and goes up the
stack to the point where application packets are exchanged. References to source files are given for those who want to
play along. Most of the code is located in the [radio](src/shared/module/radio) subfolder.

A more developer-oriented readme with radio state machines is available [here](src/shared/module/radio/README.md).
Nevertheless, it is recommended to read the current document first before going to the developer readme.

Note that most of the low-level code is purely interrupt driven and regularly uses callbacks to invoke functions from
a layer above. Callbacks are used to minimize dependencies. Usually higher layers know their lower layers, but not
the other way around. Whenever possible references to the callbacks will be given in this documentation to clarify
data flow.

This code has only been tested on Cortex-M microcontrollers from ST Microelectronics and the radio part requires a
strict and very small interrupt (IRQ) latency. To accomplish this all radio interrupts are operating above our 
operating system's priority level (sometimes called _fast interrupts_).
We use [ChibiOS](https://www.chibios.org) on robot and base station and the OS maximum IRQ priority mask will not
block radio IRQs. This also means that OS functions are not available in the radio IRQs. How we still safely exchange
data within these IRQs and also report events back to the OS is described further down below.

## SX1280 Transceiver and SKY66112 Front-End Module

The whole wireless solution is based on the off-the-shelf radio chip SX1280 from Semtech and the SKY66112
front-end module (FEM) from Skyworks. The FEM integrates a power amplifier for up to +21dBm output power and a
low-noise amplifier with 11dB receive gain. The SX1280 operates in the license-free 2.4GHz ISM band.

Robots and base station use a single SX1280 and FEM each. A single channel is used to communicate with up to 32 robots.
The SX1280 can either transmit or receive data, not both. Due to this half-duplex nature our implementation switches
transmit and receive frequently.

The SX1280 has an SPI interface and some digital IOs (DIO). In addition to the SPI, at least one DIO and the BUSY pin
must be connected to a host (micro-)controller. The host must be interrupt-capable on the DIO and BUSY pins.
The SPI transfer speed affects overall throughput and should be as close to the maximum of the SX1280 as possible (approx. 18MHz).
We use 12.5MHz on our robot and 13.5MHz on our base station (due to clock prescaler limitations).
The FEM simply uses three GPIOs to switch between RX, TX, and OFF.

Please consult our open-source electronics release for details on schematics and how to connect these chips.

## SPI Driver and Timer Interface

The SPI driver can be found in [spi_lld](src/shared/hal/spi_lld.h). As it is called from within fast interrupts it
must not use any OS functions. It transfers data via DMA and offers a callback which is used to signal transfer completion.

Apart from a hardware SPI interface a [timer](src/shared/hal/timer_simple_lld.h) is also required. It must have a 1us
resolution and call a function after a specified time. It basically acts as a variable-delay one-shot timer.
It is used to avoid busy loops so that the microcontroller can do other tasks while waiting for a radio operation.
All radio operations have timeouts specified. Either as a safety net if an expected operation does not finish in time
or as part of intended operations (timed delays).

## SX1280 Low-Level Driver

The [sx1280_lld](src/shared/drv/sx1280_lld.h) driver executes basic SX1280 commands. These are also listed in the 
[SX1280](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280) datasheet. Readers are encouraged to have
a look at the datasheet to get familier with the commands.

SX1280 commands are simple operations like setting the radio mode to transmit/receive or to read/write some data.
The driver always waits until the SX1280 is no longer busy as signaled by the BUSY pin. Then it does an SPI transfer
with the respective command and waits until the DIO interrupt signals completion of the command. Afterward, the next
higher layer is called with the information that a command completed
(usually [radio_phy::lldDoneCallback](src/shared/module/radio/radio_phy.c), see below).

Note that the whole command state machine is driven by interrupts: BUSY pin IRQ, SPI transfer complete IRQ, DIO IRQ.
If a command fails the timer IRQ may also kick in.

## Radio PHY Layer

The [radio_phy](src/shared/module/radio/radio_phy.h) driver combines multiple SX1280 commands and a state machine
to offer even higher level functions. We call a combination of multiple commands a _PHY operation_. This can e.g. be a
transmit operation which includes an optional initial delay, writing data to the SX1280, clear previous IRQ bits,
transmit the data, and wait for it to finish.

radio_phy also includes the FEM interface. It sets the FEM to the correct mode if a receive or transmit operation
was requested.

Apart from receive and transmit this layer also offers a bypass operation (called CFG) where an SX1280 command is used
directly or a simple register read-modify-write operation (MOD_REG). Both are mainly used for initial setup or radio
frequency changes.

The radio PHY layer uses two callbacks to upper layers. One is used to fetch the next operation to perform. It is one of:
- [radio_module::phyOpPrepareNextSetup](src/shared/module/radio/radio_module.c) Only during initial setup of the module, configures operating mode.
- [radio_base::phyOpPrepareNextBase](src/shared/module/radio/radio_base.c) When running on a base station.
- [radio_bot::phyOpPrepareNextClient](src/shared/module/radio/radio_bot.c) When running on a robot.

The other callback is used to inform the upper layer of operation completion. It is one of:
- [radio_base::phyOpDoneBase](src/shared/module/radio/radio_base.c) When running on a base station.
- [radio_bot::phyOpDoneClient](src/shared/module/radio/radio_bot.c) When running on a robot.

## Radio Module Layer

The [radio_module](src/shared/module/radio/radio_module.h) layer is a relatively simple layer. It just takes care of
initial configuration of the module by using a special setup callback on the PHY layer. To ensure a robust setup it also
has a GPIO connected to an LDO supplying the SX1280 and FEM. It can turn off the LDO to power-cycle these chips and to
put them in a known state with all registers restored to factory defaults.

## Buffer Management and Framing

The high-level buffer interface allows application code to transfer packets of up to 128B. A packet is stored in a 
[RadioBufferEntry](src/shared/module/radio/radio_buf.h). When a packet is enqueued in a RadioBufferEntry a CRC16
is appended, and it is automatically [COBS](src/shared/util/cobs.h) encoded. This eliminates all zeros in the data with
minimal overhead. Multiple zeros can be compressed. An appended zero is then used as a packet delimiter. The buffer
data layout is shown below:

```
 ┌───────────┬───────┬───────────┐ 
 │ User Data │ CRC16 │ Delimiter │ 
 │  0-128B   │  2B   │     1B    │ 
 └───────────┴───────┴───────────┘ 
```

A RadioBuffer holds a limited number of RadioBufferEntries (depending on available RAM). All entries in a RadioBuffer
are to or from a specific client. Hence, a base station has multiple RadioBuffers. One for each robot and direction.

After the packet is enqueued its ownership is transferred (via a volatile flag) to the radio driver (base station or robot).

The content of a RadioBufferEntry may be too large to send at once over-the-air, or it may be so small that multiple fit 
in one over-the-air packet. Hence, a streaming approach is used here. Data from multiple buffers or partial data from a 
single buffer may be concatenated in a single over-the-air packet.

Wireless packets over-the-air have a fixed length (currently 42B) and layout:

```
 ┌────────┬─────┬───────────┬─────────┐ 
 │ header │ seq │ rxSeqLoss │ payload │ 
 │   1B   │  1B │    1B     │   39B   │ 
 └────────┴─────┴───────────┴─────────┘ 
```

The header contains the robot ID, a flag for broadcast, and a flag for direction (from base or robot). The `seq` field
is a sequence number and increased for every transmission. `rxSeqLoss` is a counter for gaps in received sequence numbers.
Both fields together allow the base station to assess receive and transmit losses.

Streamed data from over-the-air packets is put into a respective receive RadioBuffer owned by the radio driver. When a 
packet delimiter (zero) is detected the RadioBufferEntry is complete and ownership is transferred to application code.
If there is more data in the over-the-air packet it is put in the next buffer.

COBS decoding and CRC verification is done when the application code retrieves the buffer data. This completes the 
application payload transmit cycle.

## Multi-Robot Coordination

The sequence of PHY operations depends on whether the radio is running on a base station or a robot. The sections
below outline both operating modes.

### Base Station

The [base station](src/shared/module/radio/radio_base.h) coordinates all communication. It uses time slots to address robots one after another (TDMA/CA).
The normal schedule consists of sending one packet to robot A and then robot A immediately sends back one return packet.
Afterward, the procedure repeats for robot B and so on.
To check if new robots came online a configurable number of slots is used afterward to send packets to robots which
are listed as offline. If a reply is received they are considered online and will be scheduled regularly.
After all robots have been addressed the base station can send up to 8 broadcast packets. This is common data
for all robots and no robot is allowed to reply to this packet.
Overall, the base station uses two radio buffers for each robot (RX+TX) plus a single buffer for broadcasts. The radio
buffers are the interface between the interrupt-driven radio driver and application code running on OS levels.
The base station does not implement any kind of retransmission mechanism on this layer to ensure predictable update rates.

### Robot

The [robot](src/shared/module/radio/radio_bot.h) radio mostly waits passively until it receives a packet. If the packet
is directly addressing this robot (not a broadcast) it will immediately send a return packet. To ensure a reception is
not interrupted by a timeout the robot generally uses longer receive timeouts than the base station.

The robot has one transmit radio buffer to the base station and two receive buffers for each robot. This stems from the
fact that each robot actually receives all data from all other robots as wireless communication is physically
always a broadcast. In addition to that each robot also gets all the data that was directed to other robots from the
base station. Nevertheless, a single robot usually only uses the data directed at it directly or broadcasted.

# Integration Guidelines

Our wireless solution is deeply integrated in our architecture. We think this is also the way to achieve maximum
performance. While it should be possible to put it on a separate PCB/microcontroller, this would come with its own set
of challenges and integration latencies. Nevertheless, our radio code is very modular and has few dependencies.
The following guidelines should help others to adapt the code to their architecture.

1. Make sure you understand your microcontroller's interrupt levels and the interaction of the operating system with it.
   If interrupts are globally disabled for more than a few microseconds it will throw of the radio's strict timings.
2. You will need to interface the FEM IOs (easy), have interrupt lines for the DIO and BUSY lines (medium), and an SPI driver
   with a transfer complete interrupt (difficult). A timer with 1us resolution is required as well.
3. Your microcontroller must be fast. At least 200MHz is recommended. The faster the IRQs execute that better is overall
   wireless throughput and timings can be tighter.
4. The radio interrupts must be the highest priority. This also means other tasks must be ok with some interruption. This is
   usually in conflict with doing motor control on the same microcontroller. In that case think about moving motor control
   to another microcontroller or try out dual-core architectures.
5. Make sure to also read the developer-oriented [readme](src/shared/module/radio/README.md) and understand the
   according code paths in the source files.
6. You can adjust all timeouts and radio settings [here](src/shared/module/radio/radio_settings.c). The current
   parameters have been tested and tuned for our hardware.
7. The [radio_module](src/shared/module/radio/radio_module.c) offers a print function for tracing data. All operations
   in the whole radio architecture are traced and offer great insights into timings and potential issues. If you read
   the above section you will find a lot of references.

Successful wireless communication is hard and always comes with trade-offs. We hope this documentation makes it a bit easier for you.
