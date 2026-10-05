#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H

/* Hexadecimal STANDARD CAN identifiers. */
#define GIMBAL_FRIC_LEFT_ID       0x205U
#define GIMBAL_FRIC_RIGHT_ID      0x206U
#define GIMBAL_FRIC_COMMAND_ID    0x1FFU
#define GIMBAL_PITCH_TX_ID        0x201U
#define GIMBAL_PITCH_RX_ID        0x201U /* User confirmed feedback uses same ID. */
#define GIMBAL_MOTOR_TIMEOUT_MS   100U
#define GIMBAL_MOTOR_TEMP_LIMIT   80U

/* DM4310 MIT ranges MUST match PMAX/VMAX/TMAX in the motor tool. */
#define GIMBAL_DM_P_MAX           12.5f
#define GIMBAL_DM_V_MAX           30.0f
#define GIMBAL_DM_T_MAX           10.0f
#define GIMBAL_PITCH_CALIBRATED   0 /* Check mechanical zero/limits, then set 1. */
#define GIMBAL_PITCH_MIN_RAD      (-0.35f)
#define GIMBAL_PITCH_MAX_RAD      0.35f
#define GIMBAL_PITCH_DIRECTION    1.0f
#define GIMBAL_PITCH_RATE_RAD_S   1.0f
#define GIMBAL_PITCH_KP           20.0f
#define GIMBAL_PITCH_KD           1.0f
#define GIMBAL_PITCH_FEEDFORWARD  0.0f /* Nm; tune gravity compensation. */

/* C620 units; speed is rotor rpm, not gearbox output rpm. */
#define GIMBAL_FRIC_RPM           6200.0f
#define GIMBAL_FRIC_LEFT_SIGN     (-1.0f)
#define GIMBAL_FRIC_RIGHT_SIGN    1.0f
#define GIMBAL_FRIC_RAMP_RPM_S    8000.0f
#define GIMBAL_FRIC_READY_RPM     300.0f
#define GIMBAL_FRIC_READY_MS      200U
#define GIMBAL_FRIC_KP            3.3f
#define GIMBAL_FRIC_KI            0.035f /* Per 2 ms control update. */
#define GIMBAL_FRIC_CURRENT_MAX   4000U
#define GIMBAL_FRIC_I_LIMIT       700U
#define GIMBAL_DIAL_RPM           2500.0f
#define GIMBAL_DIAL_INSTALLED     1
#define GIMBAL_DIAL_CALIBRATED    0 /* Verify direction/gear ratio/pockets, then enable. */
#define GIMBAL_DIAL_RX_ID         0x202U
#define GIMBAL_DIAL_COMMAND_ID    0x200U
#define GIMBAL_DIAL_DIRECTION     1.0f
#define GIMBAL_DIAL_GEAR_RATIO    36.0f
#define GIMBAL_DIAL_POCKETS       8U /* Replace with the actual disk pocket count. */
#define GIMBAL_DIAL_KP            3.0f
#define GIMBAL_DIAL_KI            0.02f /* Per 2 ms update. */
#define GIMBAL_DIAL_CURRENT_MAX   3000U /* C610 protocol range: +/-10000. */
#define GIMBAL_DIAL_I_LIMIT       500U
#define GIMBAL_DIAL_STALL_RPM     100.0f
#define GIMBAL_DIAL_STALL_MS      500U

/* ET16S calibration copied from chassis. Channel indices are ZERO based. */
#define GIMBAL_SBUS_MIN           321U
#define GIMBAL_SBUS_MID           992U
#define GIMBAL_SBUS_MAX           1663U
#define GIMBAL_SBUS_DEADZONE      50U
#define GIMBAL_SBUS_TIMEOUT_MS    100U
#define GIMBAL_CH_YAW             0U
#define GIMBAL_CH_FORWARD         1U
#define GIMBAL_CH_PITCH           2U
#define GIMBAL_CH_LATERAL         3U
#define GIMBAL_CH_ARM             4U /* SA: 1/3=safe, 2=armed (chassis NORMAL). */
#define GIMBAL_CH_FRICTION        5U /* SB: high=on. */
#define GIMBAL_CH_FEED            6U /* SC: high=continuous feed. */
#define GIMBAL_CH_UI_REFRESH      7U /* SD: rising high=redraw. */
#define GIMBAL_BOARD_INPUT_SCALE  660.0f
#define GIMBAL_BOARD_PERIOD_MS    10U
#define GIMBAL_BOARD_TIMEOUT_MS   100U
#define GIMBAL_REFEREE_TIMEOUT_MS 500U
#define GIMBAL_SHOT_HEAT          10.0f
#define GIMBAL_HEAT_RESERVE       30.0f
#define GIMBAL_UI_PERIOD_MS       100U
#endif

