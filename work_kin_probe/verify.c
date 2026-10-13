/* 用标定后的 TOOL 常量验证：建模位是否脱离奇异、NORMAL 模式能否走 70mm */
#include <stdio.h>
#include <math.h>
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static float GX, GZ;

static void eff_link(float roll, float el3_deg, float *bx, float *bz, float *tip_y)
{
    float a3 = el3_deg*D2R, ca3=cosf(a3), sa3=sinf(a3);
    float wxx = ROBOT_ARM_WRIST_X_MM + GX*ca3 - GZ*sa3;
    float wzz = ROBOT_ARM_WRIST_Z_MM + GX*sa3 + GZ*ca3;
    float wy  = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float px=-az, pz=ax, adotw = ax*wxx+az*wzz;
    float cr=cosf(roll), sr=sinf(roll);
    float tx = cr*wxx + sr*wy*px + (1-cr)*adotw*ax;
    float tz = cr*wzz + sr*wy*pz + (1-cr)*adotw*az;
    *bx = ROBOT_ARM_ROLLJ_X_MM + tx;
    *bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
    *tip_y = ROBOT_ARM_ROLLJ_Y_MM + wy*cr + (az*wxx-ax*wzz)*sr;
}
#define UPPER_LEN sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM+ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM)
#define UPPER_ELEV (atan2f(ROBOT_ARM_UPPER_Z_MM,ROBOT_ARM_UPPER_X_MM)*R2D)
static float fore_len(float roll,float el3){float bx,bz,ty;eff_link(roll,el3,&bx,&bz,&ty);return sqrtf(bx*bx+bz*bz);}
static float fore_elev(float roll,float el3){float bx,bz,ty;eff_link(roll,el3,&bx,&bz,&ty);return atan2f(bz,bx)*R2D;}

static void fk(const float deg[3], float roll, float *x, float *z, float *y)
{
    float el1=90-deg[0], el2=90-deg[1], el3=90-deg[2];
    float bx,bz,ty; eff_link(roll*D2R, el3, &bx,&bz,&ty);
    float lb=sqrtf(bx*bx+bz*bz), bb=atan2f(bz,bx);
    float a1=(UPPER_ELEV+el1)*D2R, a2=(bb*R2D+el1+el2)*D2R, la=UPPER_LEN;
    *x = ROBOT_ARM_SHOULDER_X_MM + la*cosf(a1) + lb*cosf(a2);
    *z = ROBOT_ARM_SHOULDER_Z_MM + la*sinf(a1) + lb*sinf(a2);
    *y = ty;
}
static float normal0_elev(void){return atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;}
static void r4v(float *x,float *z){
    *x = ROBOT_ARM_TCP_X_MM - (ROBOT_ARM_SHOULDER_X_MM+ROBOT_ARM_UPPER_X_MM+ROBOT_ARM_ROLLJ_X_MM+ROBOT_ARM_WRIST_X_MM);
    *z = ROBOT_ARM_TCP_Z_MM - (ROBOT_ARM_SHOULDER_Z_MM+ROBOT_ARM_UPPER_Z_MM+ROBOT_ARM_ROLLJ_Z_MM+ROBOT_ARM_WRIST_Z_MM);
}
static int solve2r(float tx,float tz,float la,float lb,float bea,float beb,float sign,float *el1,float *el2)
{
    float dx=tx-ROBOT_ARM_SHOULDER_X_MM, dz=tz-ROBOT_ARM_SHOULDER_Z_MM;
    float r2=dx*dx+dz*dz;
    float c=(r2-la*la-lb*lb)/(2*la*lb);
    if(c>1.0f-ROBOT_ARM_SING_MARGIN||c<-(1.0f-ROBOT_ARM_SING_MARGIN))return 0;
    float bend=sign*acosf(c);
    float th1=atan2f(dz,dx)-atan2f(lb*sinf(bend),la+lb*cosf(bend));
    *el1=(th1-bea*D2R)*R2D;
    *el2=bend*R2D+bea-beb;
    return 1;
}
static int ik_norm(const float cc[2], float dir, float out[3])
{
    float sigma = dir - normal0_elev();
    float sr=sinf(sigma*D2R), cr=cosf(sigma*D2R);
    float r4x,r4z; r4v(&r4x,&r4z);
    float wx=cc[0]-(r4x*cr-r4z*sr), wz=cc[1]-(r4x*sr+r4z*cr);
    float el1,el2;
    if(!solve2r(wx,wz,UPPER_LEN,fore_len(0,0),UPPER_ELEV,fore_elev(0,0),-1.0f,&el1,&el2))
        if(!solve2r(wx,wz,UPPER_LEN,fore_len(0,0),UPPER_ELEV,fore_elev(0,0),1.0f,&el1,&el2)) return 0;
    float el4 = sigma-el1-el2;
    if(fabsf(el1)>90||fabsf(el2)>90||fabsf(el4)>90) return 0;
    out[0]=90-el1; out[1]=90-el2; out[2]=90-el4;
    return 1;
}

int main(void)
{
    for (int pass=0; pass<2; pass++)
    {
        if(pass==0){GX=ROBOT_ARM_TOOL_X_MM; GZ=ROBOT_ARM_TOOL_Z_MM; printf("########## 现状常量 ##########\n");}
        else {GX=70.6730f; GZ=-47.8160f; printf("\n########## 标定后 TOOL=(%.4f, %.4f) ##########\n",GX,GZ);}

        float deg0[3]={90,90,90}, x,z,y;
        fk(deg0,0,&x,&z,&y);
        float fx=ROBOT_ARM_ROLLJ_X_MM+ROBOT_ARM_WRIST_X_MM;
        float fz=ROBOT_ARM_ROLLJ_Z_MM+ROBOT_ARM_WRIST_Z_MM;
        float la=UPPER_LEN, lb=sqrtf(fx*fx+fz*fz);
        float r0=sqrtf(fx*fx+fz*fz);
        float c0=(r0*r0-la*la-lb*lb)/(2*la*lb);
        printf("  FK(90,90,90) 爪心=(%.3f, %.3f)  目标(%.3f, %.3f)  误差 %.3f mm\n",
               x,z,ROBOT_ARM_TCP_X_MM,ROBOT_ARM_TCP_Z_MM,
               sqrtf(powf(x-ROBOT_ARM_TCP_X_MM,2)+powf(z-ROBOT_ARM_TCP_Z_MM,2)));
        printf("  肩→腕俯仰轴=%.3f  可伸最长=%.3f  余量=%+.4f mm\n", r0, la+lb, la+lb-r0);
        printf("  奇异裕度 1-|cos|=%.6f  (阈值 %.4f)  → %s\n",
               1.0f-fabsf(c0), ROBOT_ARM_SING_MARGIN,
               1.0f-fabsf(c0)>ROBOT_ARM_SING_MARGIN?"通过":"**被拒**");

        float nd = normal0_elev() + (90-deg0[0])+(90-deg0[1])+(90-deg0[2]);
        printf("  建模位法向仰角 = %.3f°\n", nd);
        /* 竖直向下 */
        float last=-1e9f, first=1e9f;
        for(float s=-150;s<=150;s+=0.5f){
            float tgt[2]={x, z-s}; float o[3];
            if(ik_norm(tgt,-90.0f,o)){ if(s<first)first=s; if(s>last)last=s; }
        }
        printf("  竖直向下(-90°)可用 s ∈ [%.1f, %.1f] mm  → 下压可用深度 %.1f mm\n",
               first, last, first>=0? first : 0.0f);
        (void)y;
    }
    return 0;
}
