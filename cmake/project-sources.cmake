# 自己维护的源码。
# CubeMX重新生成时不应修改本文件。

set(PROJECT_SOURCES
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s

    bsp/dwt/bsp_dwt.c
    bsp/pwm/bsp_pwm.c
    bsp/gpio/bsp_gpio.c
    bsp/spi/bsp_spi.c
    bsp/iic/bsp_iic.c
    bsp/can/bsp_can.c
    bsp/usart/bsp_usart.c
    bsp/usb/bsp_usb.c
    bsp/log/bsp_log.c
    bsp/flash/bsp_flash.c
    bsp/bsp_tools.c

    modules/algorithm/controller.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/algorithm/crc8.c
    modules/algorithm/crc16.c
    modules/algorithm/user_lib.c

    modules/bluetooth/HC05.c
    modules/BMI088/bmi088.c

    modules/imu/BMI088driver.c
    modules/imu/BMI088Middleware.c
    modules/imu/ins_task.c
    modules/imu/ch_imu.c

    modules/ist8310/ist8310.c

    modules/master_machine/master_process.c
    modules/master_machine/seasky_protocol.c

    modules/motor/DJImotor/dji_motor.c
    modules/motor/HTmotor/HT04.c
    modules/motor/LKmotor/LK9025.c
    modules/motor/DMmotor/dmmotor.c
    modules/motor/step_motor/step_motor.c
    modules/motor/servo_motor/servo_motor.c
    modules/motor/motor_task.c
    modules/motor/power_control.c

    modules/oled/oled.c

    modules/referee/crc_ref.c
    modules/referee/rm_referee.c
    modules/referee/referee_UI.c
    modules/referee/referee_task.c
    modules/referee/referee_custom.c
    modules/referee/referee_vt.c
    modules/referee/referee_vt_task.c

    modules/remote/remote_control.c
    modules/super_cap/super_cap.c
    modules/can_comm/can_comm.c
    modules/message_center/message_center.c
    modules/daemon/daemon.c
    modules/alarm/buzzer.c
    modules/alarm/buzzer_music.c

    application/gimbal/gimbal.c
    application/chassis/chassis.c
    application/shoot/shoot.c
    application/cmd/robot_cmd.c
    application/robot.c
)

set(PROJECT_INCLUDE_DIRS
    Middlewares/ST/ARM/DSP/Include/dsp
    Middlewares/ST/ARM/DSP/Include
    Middlewares/ST/ARM/DSP/Inc
    Middlewares/Third_Party/SEGGER/RTT
    Middlewares/Third_Party/SEGGER/Config

    application
    application/chassis
    application/shoot
    application/gimbal
    application/cmd

    bsp
    bsp/dwt
    bsp/can
    bsp/usart
    bsp/usb
    bsp/gpio
    bsp/spi
    bsp/iic
    bsp/log
    bsp/flash
    bsp/pwm

    modules
    modules/algorithm
    modules/bluetooth
    modules/BMI088
    modules/imu
    modules/ist8310
    modules/master_machine

    modules/motor
    modules/motor/DJImotor
    modules/motor/LKmotor
    modules/motor/HTmotor
    modules/motor/step_motor
    modules/motor/servo_motor
    modules/motor/DMmotor

    modules/oled
    modules/referee
    modules/remote
    modules/super_cap
    modules/can_comm
    modules/message_center
    modules/daemon
    modules/alarm
)
