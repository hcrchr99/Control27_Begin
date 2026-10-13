/* 最终核验：两种 TOOL 常量下，建模位能否做竖直下压 */
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
#define LA sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM+ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM)
#define BEA (atan2f(ROBOT_ARM_UPPER_Z_MM,ROBOT_ARM_UPPER_X_MM)*R2D)
static float fore_len(float roll,float el3){float bx,bz,ty;eff_link(roll,el3,&bx,&bz,&ty);return sqrtf(bx*bx+bz*bz);}
static float fore_elev(float roll,float el3){float bx,bz,ty;eff_link(roll,el3,&bx,&bz,&ty);return atan2f(bz,bx)*R2D;}

static void fk(const float deg[3], float roll, float *x, float *z)
{
    float el1=90-deg[0], el2=90-deg[1], el3=90-deg[2];
    float lb=fore_len(roll*D2R, el3), bb=fore_elev(roll*D2R, el3);
    float a1=(BEA+el1)*D2R, a2=(bb+el1+el2)*D2R;
    *x = ROBOT_ARM_SHOULDER_X_MM + LA*cosf(a1) + lb*cosf(a2);
    *z = ROBOT_ARM_SHOULDER_Z_MM + LA*sinf(a1) + lb*sinf(a2);
}
static float normal0_elev(void){return atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;}
static void r4v(float *x,float *z){
    *x = ROBOT_ARM_TCP_X_MM - (ROBOT_ARM_SHOULDER_X_MM+ROBOT_ARM_UPPER_X_MM+ROBOT_ARM_ROLLJ_X_MM+ROBOT_ARM_WRIST_X_MM);
    *z = ROBOT_ARM_TCP_Z_MM - (ROBOT_ARM_SHOULDER_Z_MM+ROBOT_ARM_UPPER_Z_MM+ROBOT_ARM_ROLLJ_Z_MM+ROBOT_ARM_WRIST_Z_MM);
}
static int s2r(float tx,float tz,float la,float lb,float bea,float beb,float sign,float *el1,float *el2)
{
    float dx=tx-ROBOT_ARM_SHOULDER_X_MM, dz=tz-ROBOT_ARM_SHOULDER_Z_MM;
    float r2=dx*dx+dz*dz;
    float c=(r2-la*la-lb*lb)/(2*la*lb);
    if(c>1.0f-ROBOT_ARM_SING_MARGIN||c<-(1.0f-ROBOT_ARM_SING_MARGIN))return 0;
    float bend=sign*acosf(c);
    float th1=atan2f(dz,dx)-atan2f(lb*sinf(bend),la+lb*cosf(bend));
    *el1=(th1-bea*D2R)*R2D; *el2=bend*R2D+bea-beb; return 1;
}
static int ik_norm(const float cc[2], float dir, float out[3])
{
    float sigma = dir - normal0_elev();
    float sr=sinf(sigma*D2R), cr=cosf(sigma*D2R);
    float r4x,r4z; r4v(&r4x,&r4z);
    float wx=cc[0]-(r4x*cr-r4z*sr), wz=cc[1]-(r4x*sr+r4z*cr);
    float lb0=fore_len(0,0), bb0=fore_elev(0,0);
    float el1,el2;
    if(!s2r(wx,wz,LA,lb0,BEA,bb0,-1.0f,&el1,&el2))
        if(!s2r(wx,wz,LA,lb0,BEA,bb0,1.0f,&el1,&el2)) return 0;
    float el4 = sigma-el1-el2;
    if(fabsf(el1)>90||fabsf(el2)>90||fabsf(el4)>90) return 0;
    out[0]=90-el1; out[1]=90-el2; out[2]=90-el4; return 1;
}

int main(void)
{
    for (int pass=0; pass<2; pass++)
    {
        if(pass==0){GX=ROBOT_ARM_TOOL_X_MM; GZ=ROBOT_ARM_TOOL_Z_MM;
                    printf("########## 现状 TOOL=(%.3f, %.3f) ##########\n",GX,GZ);}
        else {GX=70.6730f; GZ=-47.8160f;
              printf("\n########## 标定后 TOOL=(%.3f, %.3f) ##########\n",GX,GZ);}

        float deg0[3]={90,90,90}, x,z;
        fk(deg0,0,&x,&z);
        float lb0=fore_len(0,0), bb0=fore_elev(0,0);
        float r0=sqrtf(powf(x-ROBOT_ARM_SHOULDER_X_MM,2)+powf(z-ROBOT_ARM_SHOULDER_Z_MM,2));
        printf("  FK(90,90,90) 爪心=(%.3f, %.3f)   配置 TCP0=(%.3f, %.3f)   误差 %.3f mm\n",
               x,z,ROBOT_ARM_TCP_X_MM,ROBOT_ARM_TCP_Z_MM,
               sqrtf(powf(x-ROBOT_ARM_TCP_X_MM,2)+powf(z-ROBOT_ARM_TCP_Z_MM,2)));
        printf("  等效连杆 |ROLLJ→爪| = %.3f  基准仰角 %.3f°   可伸最长 = %.3f\n",
               lb0, bb0, LA+lb0);
        printf("  肩→爪心 = %.3f  → 剩余可伸 %.3f mm\n", r0, LA+lb0-r0);
        float c=(r0*r0-LA*LA-lb0*lb0)/(2*LA*lb0);
        printf("  1-|cos|=%.6f (阈值%.4f) → %s\n", 1.0f-fabsf(c), ROBOT_ARM_SING_MARGIN,
               (1.0f-fabsf(c))>ROBOT_ARM_SING_MARGIN?"奇异裕度通过":"**奇异被拒**");

        float nd = normal0_elev();
        printf("  建模位法向仰角 = %.3f°\n", nd);
        for (int k=0;k<2;k++)
        {
            float dir = k==0 ? 50.535f : nd;
            float lo=1e9f, hi=-1e9f;
            for(float s=-120;s<=260;s+=0.5f){
                float tgt[2]={x, z + (k==0? -s : 0)};
                if(k==1){ float ux=cosf(dir*D2R), uz=sinf(dir*D2R); tgt[0]=x+ux*s; tgt[1]=z+uz*s; }
                float o[3];
                if(ik_norm(tgt,dir,o)){ if(s<lo)lo=s; if(s>hi)hi=s; }
            }
            if(k==0) printf("  dir=+50.5 可用 s ∈ [%.1f, %.1f] mm\n", lo, hi);
            else     printf("  沿法向(%.1f°) 可用 s ∈ [%.1f, %.1f] mm\n", dir, lo, hi);
        }
    }
    return 0;
}
