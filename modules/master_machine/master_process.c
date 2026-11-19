#include "master_process.h"

#include <math.h>
#include <string.h>

#include "daemon.h"
#include "robot_def.h"
#include "seasky_protocol.h"
#include "bsp_usb.h"

static Vision_Recv_s vision_recv_data;
static SeaskyStream vision_stream;
static DaemonInstance *vision_daemon_instance;
static uint32_t vision_rejected_frames;
static uint8_t *vision_usb_rx_buffer;
static volatile uint8_t vision_has_valid_frame;

static void AcceptVisionFrame(const SeaskyFrameView *frame)
{
    if (frame->command_id != SEASKY_RX_COMMAND_ID ||
        frame->payload_size != SEASKY_RX_PAYLOAD_SIZE) {
        ++vision_rejected_frames;
        return;
    }

    float values[SEASKY_RX_FLOAT_COUNT];
    for (uint8_t i = 0; i < SEASKY_RX_FLOAT_COUNT; ++i) {
        values[i] = SeaskyReadFloatLE(frame->payload + (uint16_t)i * 4u);
        if (!isfinite(values[i])) {
            ++vision_rejected_frames;
            return;
        }
    }

    const Vision_Recv_s next = {
        .target_state = (Target_State_e)(frame->flags[0] & 0x0fu),
        .pitch = values[0],
        .yaw = values[1],
        .delta_pitch = values[2],
        .delta_yaw = values[3],
        .target_distance = values[4],
        .vel_x = values[5],
        .vel_y = values[6],
        .angle_yaw = values[7],
    };

    vision_recv_data = next;
    vision_has_valid_frame = 1u;

    // 喂狗
    DaemonReload(vision_daemon_instance);
}

static void DecodeVision(uint16_t received_size)
{
    const uint8_t *bytes = vision_usb_rx_buffer;
    uint16_t remaining = received_size;

    while (remaining > 0u) {
        uint16_t consumed = 0u;
        const SeaskyFrameView *frame =
            SeaskyFeed(&vision_stream, bytes, remaining, &consumed);
        bytes += consumed;
        remaining -= consumed;

        if (frame == NULL) {
            break;
        }
        AcceptVisionFrame(frame);
    }
}

static void VisionOfflineCallback(void *owner)
{
    (void)owner;
}

Vision_Recv_s *VisionInit(void)
{
    SeaskyProtocolInit();
    memset(&vision_stream, 0, sizeof(vision_stream));
    memset(&vision_recv_data, 0, sizeof(vision_recv_data));
    vision_rejected_frames = 0u;
    vision_has_valid_frame = 0u;

    Daemon_Init_Config_s daemon_config = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 10u,
    };
    vision_daemon_instance = DaemonRegister(&daemon_config);

    USB_Init_Config_s usb_config = {.rx_cbk = DecodeVision};
    vision_usb_rx_buffer = USBInit(usb_config);
    return &vision_recv_data;
}

void VisionSend(const Vision_Send_s *data)
{
    if (data == NULL) {
        return;
    }

    const uint8_t self_color = (uint8_t)data->self_color;
    const uint8_t work_mode = (uint8_t)data->work_mode;
    const uint8_t bullet_speed = (uint8_t)data->bullet_speed;

    /* 标志字节 0 的低四位为颜色、高四位为工作模式；字节 1 为弹速。 */
    const uint8_t flags[2] = {
        (uint8_t)(self_color + (work_mode << 4)),
        bullet_speed,
    };

    /* 负载第 3、4 个浮点位置为协议保留位，固定发送为零。 */
    const float values[SEASKY_TX_FLOAT_COUNT] = {
        data->yaw,
        data->pitch,
        data->roll,
        0.0f,
        0.0f,
        data->yaw_vel,
        data->pitch_vel,
        data->game_progress,
        data->current_hp,
        data->current_base_hp,
        data->current_enemy_base_hp,
    };

    uint8_t payload[SEASKY_TX_PAYLOAD_SIZE];
    for (uint8_t i = 0; i < SEASKY_TX_FLOAT_COUNT; ++i) {
        SeaskyWriteFloatLE(payload + (uint16_t)i * 4u, values[i]);
    }

    static uint8_t frame_buffer[VISION_SEND_SIZE];
    uint16_t frame_size = 0u;
    if (!SeaskyBuildFrame(SEASKY_TX_COMMAND_ID, flags, payload,
                          (uint16_t)sizeof(payload), frame_buffer,
                          (uint16_t)sizeof(frame_buffer), &frame_size)) {
        return;
    }

    USBTransmit(frame_buffer, frame_size);
}

uint8_t VisionIsOline(void)
{
    return (uint8_t)(vision_has_valid_frame != 0u &&
                     vision_daemon_instance != NULL &&
                     DaemonIsOnline(vision_daemon_instance));
}
