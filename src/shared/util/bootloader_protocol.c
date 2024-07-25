#include "bootloader_protocol.h"
#include "crc.h"

uint8_t BootloaderComputeHeaderChecksum(const Bootloader_Header_t* pHeader)
{
    const uint8_t* pData = (const uint8_t*)pHeader;

    uint8_t xor = 0;
    for(uint32_t i = 0; i < sizeof(Bootloader_Header_t)-3; i++)
    {
        xor ^= pData[i];
    }

    return xor;
}

uint16_t BootloaderComputePayloadChecksum(const void* pMsg)
{
    const Bootloader_Header_t* pHeader = (const Bootloader_Header_t*)pMsg;
    const uint8_t* pPayload = (const uint8_t*)pMsg + sizeof(Bootloader_Header_t);
    uint16_t payloadLength = pHeader->length - sizeof(Bootloader_Header_t);

    return CRC16CalcChecksum(pPayload, payloadLength);
}

uint8_t BootloaderParserStream(Bootloader_Parser_t* pParser, uint8_t data)
{
    pParser->stats.bytesReceived++;

    if(pParser->state == 0)
    {
        if(data == 'B')
        {
            pParser->buf[pParser->state] = data;
            pParser->state = 1;
        }
    }
    else if(pParser->state == 1)
    {
        if(data == 'L')
        {
            pParser->buf[pParser->state] = data;
            pParser->state = 2;
        }
        else if(data == 'B')
        {
            pParser->stats.invalidMagic++;
            pParser->stats.bytesDropped++;
        }
        else
        {
            pParser->stats.invalidMagic++;
            pParser->stats.bytesDropped++;
            pParser->state = 0;
        }
    }
    else if(pParser->state < sizeof(Bootloader_Header_t))
    {
        pParser->buf[pParser->state] = data;
        pParser->state++;

        if(pParser->state == sizeof(Bootloader_Header_t))
        {
            if(pParser->header.headerXOR != BootloaderComputeHeaderChecksum(&pParser->header))
            {
                // invalid header checksum
                pParser->stats.invalidHeader++;
                pParser->stats.bytesDropped += pParser->state;
                pParser->state = 0;
            }
            else if(pParser->header.length == sizeof(Bootloader_Header_t))
            {
                // message already complete, no payload
                pParser->state = 0;
                return 1;
            }
            else if(pParser->header.length > sizeof(pParser->buf))
            {
                // message too long
                pParser->stats.tooLong++;
                pParser->stats.bytesDropped += pParser->state;
                pParser->state = 0;
            }
        }
    }
    else if(pParser->state < pParser->header.length)
    {
        pParser->buf[pParser->state] = data;
        pParser->state++;

        if(pParser->state == pParser->header.length)
        {
            // message complete
            if(pParser->header.crc == BootloaderComputePayloadChecksum(pParser->buf))
            {
                pParser->state = 0;
                return 1;
            }
            else
            {
                pParser->stats.invalidPayload++;
                pParser->stats.bytesDropped += pParser->state;
                pParser->state = 0;
            }
        }
    }
    else
    {
        pParser->state = 0;
    }

    return 0;
}

void BootloaderInsertChecksums(void* pMsg)
{
    Bootloader_Header_t* pHeader = (Bootloader_Header_t*)pMsg;
    pHeader->headerXOR = BootloaderComputeHeaderChecksum(pHeader);
    pHeader->crc = BootloaderComputePayloadChecksum(pMsg);
}
