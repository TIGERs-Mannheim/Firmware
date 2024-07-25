#pragma once

#include <stdint.h>

#define BOOTLOADER_MSG_TYPE_GET_INFO        0x00
#define BOOTLOADER_MSG_TYPE_ACK             0x01
#define BOOTLOADER_MSG_TYPE_EXIT            0x02
#define BOOTLOADER_MSG_TYPE_CHECK_CRC       0x03
#define BOOTLOADER_MSG_TYPE_ERASE           0x04
#define BOOTLOADER_MSG_TYPE_WRITE           0x05
#define BOOTLOADER_MSG_TYPE_ENTER           0x06
#define BOOTLOADER_MSG_TYPE_CUSTOM          0x7F

#define BOOTLOADER_RESP_ACK                 0x79

#define BOOTLOADER_MAX_PAYLOAD_SIZE         2048

#pragma pack(push, 1)

typedef struct {
    uint8_t magic[2]; // "BL"
    uint16_t length; // length of all message data including this header
    uint8_t type;
    uint8_t headerXOR; // XOR over previous fields
    uint16_t crc; // CRC over all following fields and data bytes
} Bootloader_Header_t;

typedef struct {
    Bootloader_Header_t header;

    uint16_t version;
    uint16_t writeGranularity; // smallest number of bytes that can be written
    uint32_t writeTime_us; // maximum time it takes to write <writeGranularity> bytes
    uint32_t eraseTime_us; // maximum time it takes to erase all available flash
    char deviceName[16]; // null-terminated string
} Bootloader_Info_t;

typedef struct {
    Bootloader_Header_t header;

    uint32_t addr;
    uint32_t length;
    uint32_t crc32;
} Bootloader_CRC_t;

typedef struct {
    Bootloader_Header_t header;

    uint32_t addr;
    uint32_t length;
} Bootloader_Erase_t;

typedef struct {
    Bootloader_Header_t header;

    uint32_t addr;
} Bootloader_Write_t;

typedef struct {
    Bootloader_Header_t header;

    uint8_t ack;
} Bootloader_Ack_t;

#pragma pack(pop)

typedef struct {
    uint16_t state;

    struct {
        uint32_t invalidHeader;
        uint32_t tooLong;
        uint32_t invalidPayload;
        uint32_t invalidMagic;

        uint32_t bytesReceived;
        uint32_t bytesDropped;
    } stats;

    union {
        Bootloader_Header_t header;
        uint8_t buf[BOOTLOADER_MAX_PAYLOAD_SIZE+sizeof(Bootloader_Header_t)];
    };
} Bootloader_Parser_t;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t  BootloaderComputeHeaderChecksum(const Bootloader_Header_t* pHeader);
uint16_t BootloaderComputePayloadChecksum(const void* pMsg);
uint8_t  BootloaderParserStream(Bootloader_Parser_t* pParser, uint8_t data);
void     BootloaderInsertChecksums(void* pMsg);

#ifdef __cplusplus
}
#endif
