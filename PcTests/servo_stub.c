/**
 * @file    servo_stub.c
 * @brief   servo 替身实现（真 servo.c 不进 PC 构建，此处提供同名符号）
 */
#include "servo_stub.h"

float     Stub_Servo_LastDeg  = 0.0f;
ServoId_t Stub_Servo_LastId   = SERVO_ID1;
int       Stub_Servo_SetCalls = 0;
bool      Stub_Servo_Released[SERVO_COUNT];
int       Stub_Servo_ReleaseCalls = 0;

void Stub_Servo_Reset(void)
{
    Stub_Servo_LastDeg  = 0.0f;
    Stub_Servo_LastId   = SERVO_ID1;
    Stub_Servo_SetCalls = 0;
    for (int i = 0; i < (int)SERVO_COUNT; i++)
    {
        Stub_Servo_Released[i] = false;
    }
    Stub_Servo_ReleaseCalls = 0;
}

void Servo_SetAngle(ServoId_t id, float deg)
{
    Stub_Servo_LastId = id;
    Stub_Servo_LastDeg = deg;
    Stub_Servo_SetCalls++;
    Stub_Servo_Released[id] = false;    /* 与真件一致：SetAngle 自动重新上力 */
}

bool Servo_Release(ServoId_t id)
{
    if ((int)id >= (int)SERVO_COUNT)
    {
        return false;
    }
    Stub_Servo_Released[id] = true;
    Stub_Servo_ReleaseCalls++;
    return true;
}
