#include "seasky_protocol.h"

#include <string.h>

#include "crc16.h"
#include "crc8.h"

static uint16_t ReadU16LE(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static void WriteU16LE(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)(value & 0xffu);
    bytes[1] = (uint8_t)(value >> 8);
}

static void DropBytes(SeaskyStream *stream, uint16_t count)
{
    if (count >= stream->used) {
        stream->used = 0;
        return;
    }

    stream->used -= count;
    memmove(stream->buffer, stream->buffer + count, stream->used);
}

void SeaskyProtocolInit(void)
{
    static const uint8_t seed = 0u;
    (void)crc_16(&seed, 1u);
}

uint8_t SeaskyBuildFrame(uint16_t command_id, const uint8_t flags[2],
                         const uint8_t *payload, uint16_t payload_size,
                         uint8_t *frame_buffer, uint16_t buffer_capacity,
                         uint16_t *frame_size)
{
    if (flags == NULL || frame_buffer == NULL || frame_size == NULL ||
        (payload_size > 0u && payload == NULL)) {
        return 0u;
    }

    const uint32_t data_length = (uint32_t)payload_size + 2u;
    const uint32_t total_size = data_length + 8u;
    if (total_size > SEASKY_MAX_FRAME_SIZE || total_size > buffer_capacity) {
        return 0u;
    }

    frame_buffer[0] = SEASKY_SOF;
    WriteU16LE(frame_buffer + 1, (uint16_t)data_length);
    frame_buffer[3] = crc_8(frame_buffer, 3u);
    WriteU16LE(frame_buffer + 4, command_id);
    frame_buffer[6] = flags[0];
    frame_buffer[7] = flags[1];
    if (payload_size > 0u) {
        memcpy(frame_buffer + 8, payload, payload_size);
    }

    const uint16_t checksum =
        crc_16(frame_buffer, (uint16_t)(total_size - 2u));
    WriteU16LE(frame_buffer + total_size - 2u, checksum);
    *frame_size = (uint16_t)total_size;
    return 1u;
}

const SeaskyFrameView *SeaskyFeed(SeaskyStream *stream,
                                  const uint8_t *bytes, uint16_t size,
                                  uint16_t *consumed)
{
    if (consumed != NULL) {
        *consumed = 0u;
    }
    if (stream == NULL || consumed == NULL || (size > 0u && bytes == NULL)) {
        return NULL;
    }

    uint16_t input_index = 0u;
    for (;;) {
        if (stream->used > 0u) {
            if (stream->buffer[0] != SEASKY_SOF) {
                DropBytes(stream, 1u);
                continue;
            }

            if (stream->used >= 4u) {
                if (crc_8(stream->buffer, 3u) != stream->buffer[3]) {
                    ++stream->crc_errors;
                    DropBytes(stream, 1u);
                    continue;
                }

                const uint16_t data_length = ReadU16LE(stream->buffer + 1);
                const uint32_t total_size = (uint32_t)data_length + 8u;
                if (data_length < 2u ||
                    total_size > SEASKY_MAX_FRAME_SIZE) {
                    ++stream->length_errors;
                    DropBytes(stream, 1u);
                    continue;
                }

                if (stream->used >= total_size) {
                    const uint16_t expected_crc =
                        ReadU16LE(stream->buffer + total_size - 2u);
                    const uint16_t actual_crc =
                        crc_16(stream->buffer,
                               (uint16_t)(total_size - 2u));
                    if (actual_crc != expected_crc) {
                        ++stream->crc_errors;
                        DropBytes(stream, 1u);
                        continue;
                    }

                    const uint16_t payload_size =
                        (uint16_t)(data_length - 2u);
                    memcpy(stream->frame_payload, stream->buffer + 8,
                           payload_size);
                    stream->frame = (SeaskyFrameView){
                        .command_id = ReadU16LE(stream->buffer + 4),
                        .flags = {stream->buffer[6], stream->buffer[7]},
                        .payload = stream->frame_payload,
                        .payload_size = payload_size,
                    };
                    ++stream->frames;
                    DropBytes(stream, (uint16_t)total_size);
                    return &stream->frame;
                }
            }
        }

        if (input_index >= size) {
            return NULL;
        }

        if (stream->used == SEASKY_MAX_FRAME_SIZE) {
            ++stream->length_errors;
            DropBytes(stream, 1u);
        }
        stream->buffer[stream->used++] = bytes[input_index++];
        *consumed = input_index;
    }
}

float SeaskyReadFloatLE(const uint8_t bytes[4])
{
    const uint32_t bits = (uint32_t)bytes[0] |
                          ((uint32_t)bytes[1] << 8) |
                          ((uint32_t)bytes[2] << 16) |
                          ((uint32_t)bytes[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void SeaskyWriteFloatLE(uint8_t bytes[4], float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    bytes[0] = (uint8_t)(bits & 0xffu);
    bytes[1] = (uint8_t)((bits >> 8) & 0xffu);
    bytes[2] = (uint8_t)((bits >> 16) & 0xffu);
    bytes[3] = (uint8_t)(bits >> 24);
}
