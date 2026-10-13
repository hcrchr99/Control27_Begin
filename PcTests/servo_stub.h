/**
 * @file    servo_stub.h
 * @brief   servo 器件层的 host 可控替身（仅 PC 单测）
 *
 * actuator.c 只 include servo.h 调 Servo_SetAngle/Servo_Release 两个口，
 * 本替身记录调用供用例断言（最后发的角度、各通道是否被卸力、调用次数）。
 */
#ifndef SERVO_STUB_H
#define SERVO_STUB_H

#include <stdbool.h>
#include "servo.h"

extern float     Stub_Servo_LastDeg;             /* 最后一次 SetAngle 的角度 */
extern ServoId_t Stub_Servo_LastId;              /* 最后一次 SetAngle 的通道 */
extern int       Stub_Servo_SetCalls;            /* SetAngle 累计次数 */
extern bool      Stub_Servo_Released[SERVO_COUNT]; /* Release 过的通道 */
extern int       Stub_Servo_ReleaseCalls;        /* Release 累计次数 */

void Stub_Servo_Reset(void);                     /* 用例间清记录 */

#endif /* SERVO_STUB_H */
