# TIGERs Mannheim Radio Communication

The main wireless architecture has four layers:
 - SX1280 Low Level Driver (SX1280LLD, sx1280_lld.c)  
 - Radio PHY (radio_phy.c)
 - Radio Module (radio_module.c)
 - Radio Base (radio_base.c) or Radio Bot (radio_bot.c)

Payload data transfers are done via buffers:
 - Radio Buffer (radio_buf.c)

The following lower layer drivers are used:
 - SPI Low Level Driver (spi_lld.c)
 - Timer Simple Low Level Driver (timer_simple_lld.c)

On top the following application layer is used:
 - Network (network.c)
 - Network Implementation (robot only, network_impl.c)

## Architecture

The following diagram shows how all components are related:
```
                                                                 Network                              
                                                                     ▲                                
                                                                     │                                
                                                                     │                                
                                                                     │                                
Low-Prio IRQ ───────────────────► Radio Base/Bot ───────► OS Event   │      Packet Scheduling         
                                       │    │                        │                                
                                       │    │                        │                                
                                       │    │                        ▼                                
                                       │    └──────In/Out/Broadcast Buffers                           
                                       │                                                              
                                       │                                                              
                                 Radio Module ◄───── Settings               Setup                     
                                  │  │                                                                
                      ┌───────────┘  │                                                                
                      │              │                                                                
                      │              │                                                                
                      │              │                                                                
                      │              │                                                                
                 Power Pin       Radio PHY ◄──── PHY Timeouts               Operations                
                                  │    │                                    RX, TX, MOD, CFG, Idle    
                                  │    │                                                              
High-Prio IRQ ──┐                 │    └──────┐                                                       
                │                 │           │                                                       
                │                 │           │                                                       
Busy IRQ ───────┼───────► SX1280LLD        Front-End Module                                           
                │           │   │                                                                     
                │      ┌────┘   └────┐                                                                
DIO IRQ ────────┘      │             │                                                                
                       │             │                                                                
                    SPILLD        Timer 1us                                 Commands                  
                                                                            All SX1280 commands       
                                                                            +Wait DIO, Timed Idle, NOP
```

## SX1280LLD

Low-level driver, does not use any OS functions. This driver handles direct communication to the SX1280
via SPI. For reliable and precise timeouts (or idle periods) a 1us timer is used.  
This driver is fully interrupt driven. All interrupts operate outside (above) OS levels (aka fast interrupts).
Busy IRQ, DIO IRQ, and timer IRQ call the high-prio IRQ, which contains the main logic.  
Control works by issuing _commands_. Most of the commands listed in the SX1280 are implemented.
Additionally, the driver can also wait for a digital IO (DIO) IRQ, idle for a specific time, or do nothing (NOP).

## Radio PHY

This driver combines an SX1280 and a front-end module. Furthermore, it takes higher-level operations
like transmitting or receiving a packet and breaks it down to smaller SX1280 commands.  
Data flow works only via callbacks. Whenever the SX1280LLD is done with a command, the Radio PHY is
queried for the next command.  
When Radio PHY completes an operation, it calls another callback for the next higher layer.

## Radio Module

This component mainly takes care of setting up the SX1280 and configuring it. Lower level drivers
do not have access to the module settings.

## Radio Base/Bot

Scheduling component on base station or robot. It is called whenever the Radio PHY finishes an operation.  
It can trigger a low priority interrupt to get back to OS interrupt level. This interrupt in turn
uses OS functions to send out events to all interested components.

This component also contains radio buffers. All payload data IO is done via these radio buffers.  
They handle COBS encoding/decoding of payload data and framing via an appended zero byte.

# Radio PHY State Machine

```
┌──►START
│     │ GetOp():NEXT
│     │
│     ├─────────────────────┬────────────────────┬─────────┬────────┐
│     │ [TX]                │ [RX]               │ [MOD]   │ [CFG]  │ [IDLE]   Guards:
│     │                     │ ->CIRQ             │ ->RREG  │ ->CFG  │ ->IDLE   RX:  Receive Operation
│     ├───────────┐         │                    │         │        │          TX:  Transmit Operation
│     │ [DELAY]   │ [else]  │                  WAIT_RREG   │        │          CFG: Set Configuration
│     │ ->IDLE    │         ▼                    │ ->WREG  │        │          MOD: Modify Register (RMW)
│     ▼           │       WAIT_CIRQRX            │         │        │          TO:  Timeout
│   INIT_TX◄──────┘         │ ->SRX              │         │        │
│     │ ->WBUF              │                    │         │        │          Actions:
│     ▼                     │                    │         │        │          ->RREG: Read register
│   WAIT_WBUF               │                    │         │        │          ->WREG: Write register
│     │ ->CIRQ              ▼                    │         │        │          ->CFG:  Send custom low-level command
│     ▼                   WAIT_SRX               │         │        │          ->WBUF: Write buffer
│   WAIT_CIRQTX             │ ->WDIO             │         │        │          ->CIRQ: Clear IRQ on SX1280
│     │ ->STX               ▼                    │         │        │          ->CDIO: Clear Digital IO (IRQ) on host
│     │                   WAIT_WDIORX──┐         │         │        │          ->STX:  Start transmitting
│     │                     │ [TO=0]   │ [TO=1]  │         │        │          ->WDIO: Wait for Digital IO (IRQ)
│     │                     │ ->GPSR   │ ->SFS   │         │        │          ->IDLE: Do nothing for some time
│     ▼                     ▼          │         │         │        │          ->SRX:  Start receiving
│   WAIT_STX              WAIT_GPSR    │         │         │        │          ->GPSR: Get Packet Status
│     │ ->WDIO              │  ->RBUF  │         │         │        │          ->RBUF: Read buffer
│     ▼                     ▼          │         │         │        │          ->SFS:  Switch to FS Mode (abort receiving)
│   WAIT_WDIOTX           WAIT_RBUF    │         │         │        │
│     │                     │          │         │         │        │
│     │                     │◄─────────┘         │         │        │
│     │                     │                    │         │        │
│     ▼                     ▼                    ▼         ▼        ▼
└───FINAL◄──────────────────────────────────────────────────────────┘
```

# SX1280 Data Flow
```
 ENQUEUE_CMD         IRQ_TIMER  Prio 3    IRQ_BUSY_LOW  Prio 2  IRQ_DIO  Prio 2     IRQ_SPI_DONE  Prio 1
 ->SET_QUEUED        ->SET_TIMEOUT_FLAG   ->SET_READY_FLAG      ->SET_DONE_FLAG     ->SET_DONE_FLAG
 ->SET_CMD_ACT       ->SET_IRQ_HIGH       ->SET_IRQ_HIGH        ->SET_IRQ_HIGH      ->SET_IRQ_HIGH
 ->SET_IRQ_HIGH      │                    │                     │                   │
 │                   │                    │                     │                   │
 ▼                   ▼                    ▼                     ▼                   ▼
 ────────────┬───────────────────────────────────────────────────────────────────────
             ▼
             IRQ_HIGH  Prio 0
             │
             ▼
┌────────[N]─CMD_ACT?
│            │
▼           [Y]
->CB_DONE    │
             ▼
       ┌─[N]─QUEUED?
       │     │
       │    [Y]
       │     │->START_TIMER
       │     │->SET_PENDING
       │     ├────────────────┐
       │     │                │
       │     ▼                ▼
       │     [BUSY_HIGH]      [BUSY_LOW]
       │     │                ->SET_READY_FLAG
       │     │                │
       └────►│◄───────────────┘
             │
             ▼
       ┌─[N]─PENDING?
       │     │
       │    [Y]
       │     │       [READY_FLAG]
       │     ├───────────────────┬────────────────────┬─────────────────┐
       │     │                   │                    │                 │
       │     │                   ▼                    ▼                 ▼
       │     │[TIMEOUT_FLAG]     [CMD_TIMED_IDLE]     [CMD_WAIT_DIO]    [else]
       │     │->RES_TIMEDOUT     ->SET_ACTIVE         ->SET_ACTIVE      ->START_XFER
       │     │->SET_DONE         │                    │                 ->SET_ACTIVE
       │     │                   │                    │                 │
       │     │                   │                    │                 │
       │     │                   ▼                    ▼                 │
       └────►│◄─────────────────────────────────────────────────────────┘
             │
             ▼
       ┌─[N]─ACTIVE?
       │     │
       │    [Y]
       │     │
       │     ├──────────────────┐
       │     │                  │
       │     ▼                  ▼
       │     [DONE_FLAG]        [TIMEOUT_FLAG]
       │     ->STOP_TIMER       │
       │     ->RES_OK           ├──────────────────────┐
       │     ->SET_DONE         │                      │
       │     │                  ▼                      ▼
       │     │                  [CMD_TIMED_IDLE]       [else]
       │     │                  ->RES_OK               ->RES_TIMEDOUT
       │     │                  │                      │
       │     │                  │◄─────────────────────┘
       │     │                  │
       │     │                  │->SET_DONE
       └────►│◄─────────────────┘
             │
             ▼
       ┌─[N]─DONE?
       │     │
       │    [Y]
       │     │
       │     │->SET_STATS
       │     │->CLEAR_CMD_ACT
       │     │->CB_DONE
       │     │
       │     ▼
       └────►END
```