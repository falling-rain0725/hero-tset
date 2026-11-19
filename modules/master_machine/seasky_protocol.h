#ifndef SEASKY_PROTOCOL_H
#define SEASKY_PROTOCOL_H

#include <stdint.h>

#define SEASKY_SOF 0xA5u
#define SEASKY_MAX_FRAME_SIZE 128u
#define SEASKY_MAX_PAYLOAD_SIZE (SEASKY_MAX_FRAME_SIZE - 10u)

#define SEASKY_TX_COMMAND_ID 0x0002u
#define SEASKY_RX_COMMAND_ID 0x0001u

#define SEASKY_TX_FLOAT_COUNT 11u
#define SEASKY_RX_FLOAT_COUNT 8u
#define SEASKY_TX_PAYLOAD_SIZE (SEASKY_TX_FLOAT_COUNT * 4u)
#define SEASKY_RX_PAYLOAD_SIZE (SEASKY_RX_FLOAT_COUNT * 4u)
#define SEASKY_TX_FRAME_SIZE (SEASKY_TX_PAYLOAD_SIZE + 10u)
#define SEASKY_RX_FRAME_SIZE (SEASKY_RX_PAYLOAD_SIZE + 10u)

typedef struct
{
    uint16_t command_id;
    uint8_t flags[2];
    const uint8_t *payload;
    uint16_t payload_size;
} SeaskyFrameView;

typedef struct
{
    uint8_t buffer[SEASKY_MAX_FRAME_SIZE];
    uint16_t used;
    uint8_t frame_payload[SEASKY_MAX_PAYLOAD_SIZE];
    SeaskyFrameView frame;
    uint32_t frames;
    uint32_t crc_errors;
    uint32_t length_errors;
} SeaskyStream;

void SeaskyProtocolInit(void);

/** 将命令、标志字节和负载封装为一帧完整数据。 */
uint8_t SeaskyBuildFrame(uint16_t command_id, const uint8_t flags[2],
                         const uint8_t *payload, uint16_t payload_size,
                         uint8_t *frame_buffer, uint16_t buffer_capacity,
                         uint16_t *frame_size);

/**
 * 将接收数据送入流式解析器；找到完整有效帧时返回帧指针。
 * consumed 返回本次从 bytes 消耗的字节数；帧指针在下一次调用前有效。
 */
const SeaskyFrameView *SeaskyFeed(SeaskyStream *stream,
                                  const uint8_t *bytes, uint16_t size,
                                  uint16_t *consumed);

/** 按小端字节序读取 IEEE-754 单精度浮点数，不依赖地址对齐。 */
float SeaskyReadFloatLE(const uint8_t bytes[4]);

/** 按小端字节序写入 IEEE-754 单精度浮点数，不依赖地址对齐。 */
void SeaskyWriteFloatLE(uint8_t bytes[4], float value);

#endif
