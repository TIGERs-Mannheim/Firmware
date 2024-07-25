/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * STM32F76xxI generic setup.
 * 
 * RAM0 - Data, Heap.
 * RAM3 - Main Stack, Process Stack, BSS, NOCACHE, ETH.
 *
 */
MEMORY
{
    flash0 (rx) : org = 0x08020000, len = 384k        /* Flash as AXIM (writable) */
    flash1 (rx) : org = 0x00220000, len = 384k        /* Flash as ITCM */
    flash2 (rx) : org = 0x00000000, len = 0
    flash3 (rx) : org = 0x00000000, len = 0
    flash4 (rx) : org = 0x00000000, len = 0
    flash5 (rx) : org = 0x00000000, len = 0
    flash6 (rx) : org = 0x00000000, len = 0
    flash7 (rx) : org = 0x00000000, len = 0
    ram0   (wx) : org = 0x20020000, len = 384k      /* SRAM1 + SRAM2 */
    ram1   (wx) : org = 0x20020000, len = 368k      /* SRAM1 */
    ram2   (wx) : org = 0x2007C000, len = 16k       /* SRAM2 */
    ram3   (wx) : org = 0x20000000, len = 128k      /* DTCM-RAM */
    ram4   (wx) : org = 0x00000000, len = 16k       /* ITCM-RAM */
    ram5   (wx) : org = 0x40024000, len = 4k        /* BCKP SRAM */
    ram6   (wx) : org = 0x00000000, len = 0
    ram7   (wx) : org = 0x00000000, len = 0
}

INCLUDE common.ld
