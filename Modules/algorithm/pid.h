/**
 * @file    pid.h
 * @brief   PID 控制器（W2.1，移植自 control-2026 Modules/algorithm/controller，
 *          Wang Hongxi 风格，接口与优化环节位掩码原样保留）
 *
 * 分层与边界（Modules 层开发规划 §3.7）：
 *  - 本模块是纯算法库：除 PID 内部记时间要用 bsp_sys（两次计算的间隔自己
 *    算，调用方不用传）外，不 include 任何 bsp/cmsis，可以在电脑上单独
 *    编译跑测试（PcTests/）；
 *  - 对下零依赖（不碰 PWM/编码器），对上给的是限幅后的控制量——速度环
 *    （W2.3）里 MaxOut=1.0f 时输出可以直接喂给 Motor_SetDuty，不用换算；
 *  - 禁止在中断里调用：每个 PID 实例都记着自己的积分、上次时间、堵转
 *    标志等状态，中断和主程序同时用同一个实例会把这些状态搅乱（坑#9）。
 *
 * 移植时改了什么（原版的五个问题/差异，PC 单测覆盖）：
 *  1. 去掉 arm_math 依赖（原版只是 include 了，实际没用到）；
 *  2. 记时间从 bsp_dwt 换成 bsp_sys：实例自己记上次的微秒数，两次相减
 *     得到间隔；微秒计数器计满归零后，相减的结果依然正确；
 *  3. 积分限幅里原来用了 static 临时变量——所有实例共用一份，几个 PID
 *     一起算会互相覆盖，改成函数内的局部变量；
 *  4. 初始化原来用 memcpy 把配置整体拍进实例，要求两个结构体的内存排列
 *     一模一样，改错字段也不报错——改成按名字逐字段赋值，编译器帮着查；
 *  5. 两次计算落在同一微秒时，强制按最小间隔 1 微秒算：微分项要除以
 *     间隔，间隔为 0 会算出 inf/NaN 这种坏数并污染整个实例。
 *
 * 保留的原版行为（PC 单测把它们固化成规格，改动前先改用例）：
 *  - 误差进入死区的那次计算：P/I/D 三项断供（输出里的 PID 部分清零）、
 *    这次的新增积分清零，但之前已经累积下来的积分不清——误差离开死区后
 *    接着原来的积分继续，不从头再积；前馈项不在此列（见下"本项目新增"），
 *    死区里也照给——不然到点后前馈断供、被控量掉下来、又冲出死区，
 *    围着死区边界来回振荡；
 *  - 堵转检测在"输出接近 0"（根本没给出力）或"指令接近 0"（本来就让它停）
 *    时跳过检查：这两种情况电机不动是正常的，不是堵转；跳过时计数不动
 *    （不加也不清零）；
 *  - 堵转标志一旦置起就一直保持，本模块不会自己清除——什么时候清由
 *    使用方决定（W2.3 速度环上报报警后自己清计数和标志）；
 *  - 输入若混进 NaN（比如 0÷0 的结果）会一路传染到输出，本层不检查，
 *    靠下游 Motor_SetDuty 的 NaN 拦截兜底（电机滑行）。
 *
 * 本项目新增（2026-10-20，前馈 Kff）：
 *  - 输出 = Kp·误差 + 积分 + Kd·微分 + Kff·指令。前三项是"事后补救"——
 *    看到误差才出力；前馈是"事先按小抄出力"——指令说要多少就先垫多少。
 *    速度环里 Kff 单位是"占空比/转"：它就是电机匀速那张小抄（转这个速度
 *    本来就需要这么大占空比），数值上 ≈ 开环增益（rpm/duty）的倒数。
 *  - 为什么要它：没有前馈时，匀速需要的出力全靠积分一项慢慢顶上去（P 只
 *    看得见剩余误差），于是稳态时积分常年顶在高位，指令一来还要等积分
 *    爬坡——实测斜坡恒差 ~15rpm 就是这个原因。前馈把"匀速出力"一次付清，
 *    积分只剩修小偏差的活儿，阶跃瞬间到位也不再逼着加大 P。
 *  - Kff 放在死区分支之外叠加（死区里照给，理由见上）；输出限幅和积分
 *    抗饱和（Integral_Limit）看到的都是含前馈的总出力，饱和判断不漏算。
 *  - Kff=0 时输出与原版逐位一致，旧用例不受影响。标定方法见
 *    robot_config.h 的 ROBOT_MOTOR_SPEED_KFF 注释。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 优化环节开关：按需要把下面几项用 | 组合进 Improve 字段即可生效
 *（原样保留 control-2026 的位定义） */
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,               /* 什么优化都不开 */
    PID_Integral_Limit = 0b00000001,             /* 积分限幅 */
    PID_Derivative_On_Measurement = 0b00000010,  /* 微分先行（只看测量值变化） */
    PID_Trapezoid_Intergral = 0b00000100,        /* 梯形积分 */
    PID_Proportional_On_Measurement = 0b00001000,/* 测量值比例（本项目未用） */
    PID_OutputFilter = 0b00010000,               /* 输出低通滤波 */
    PID_ChangingIntegrationRate = 0b00100000,    /* 变速积分 */
    PID_DerivativeFilter = 0b01000000,           /* 微分低通滤波 */
    PID_ErrorHandle = 0b10000000,                /* 堵转检测 */
} PID_Improvement_e;

/* PID 报错类型：堵转标志置起后保持，清除由使用方负责 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U,
} ErrorType_e;

typedef struct
{
    uint32_t ERRORCount;      /* 连续"指令大却没动"的次数；电机追上指令就归零 */
    ErrorType_e ERRORType;    /* 堵转标志：计数超过 500 次置起，之后一直保持 */
} PID_ErrorHandler_t;

/* PID 实例（调用方静态分配；速度环要 4 份，每路电机一个） */
typedef struct
{
    /* ---------------- 配置参数（PIDInit 时装入，和下面的结构体对应） ------ */
    float Kp;
    float Ki;
    float Kd;
    float Kff;                /* 前馈增益：出力里按指令直接垫的一份（速度环 =
                               * 占空比/转，≈开环增益 rpm/duty 的倒数，0=关闭） */
    float MaxOut;             /* 输出限幅（速度环里 = 占空比满量程 1.0f） */
    float DeadBand;           /* 死区：误差小到这个范围内就当已到位，
                               * 输出清零、不再积累积分 */

    /* 优化环节参数（开哪个优化环节，哪个才有意义） */
    PID_Improvement_e Improve;
    float IntegralLimit;      /* 积分累计量的上限，防止积分无限增长 */
    float CoefA;              /* 变速积分：误差在 CoefB..CoefA+CoefB 之间时积分按比例减速 */
    float CoefB;              /* 变速积分：误差小于 CoefB 全速积，大于 CoefA+CoefB 不积 */
    float Output_LPF_RC;      /* 输出低通滤波的时间常数（秒） */
    float Derivative_LPF_RC;  /* 微分低通滤波的时间常数（秒） */

    /* ---------------- 运行状态（每次计算自动更新，不用管） ---------------- */
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;               /* 本次的比例项输出 */
    float Iout;               /* 积分累计量（历史总和） */
    float Dout;               /* 本次的微分项输出 */
    float Fout;               /* 本次的前馈项输出（= Kff×指令，与误差无关） */
    float ITerm;              /* 本次新增的积分量 */

    float Output;             /* 最终输出 */
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t last_us;         /* 上次计算时的微秒时间戳（用来算两次计算的间隔） */
    float dt;                 /* 本次与上次计算的间隔（秒），实例自己算 */

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于 PID 初始化的结构体（字段与上面实例的配置段一一对应） */
typedef struct
{
    /* basic parameter */
    float Kp;
    float Ki;
    float Kd;
    float Kff;                /* 前馈增益（0 = 关闭前馈） */
    float MaxOut;             /* 输出限幅 */
    float DeadBand;           /* 死区 */

    /* improve parameter */
    PID_Improvement_e Improve;
    float IntegralLimit;      /* 积分累计量上限 */
    float CoefA;              /* 变速积分参数 */
    float CoefB;
    float Output_LPF_RC;      /* 输出低通时间常数 */
    float Derivative_LPF_RC;  /* 微分低通时间常数 */
} PID_Init_Config_s;

/**
 * @brief  初始化 PID 实例：装参数、清历史状态、记下当前时刻
 * @param  pid    PID 实例指针（调用方持有，静态分配）
 * @param  config 初始化配置（只读，逐字段拷入）
 * @note   初始化时记下当前时刻，这样第一次计算也能算出"距初始化过了多久"
 *         当作时间间隔；时间取自 Bsp_GetUs，调用前须已 Bsp_Init（上电
 *         主流程保证）
 */
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);

/**
 * @brief  PID 计算（位置式）
 * @param  pid     PID 实例指针
 * @param  measure 反馈值（电机实际转到哪了）
 * @param  ref     设定值（想让电机到哪）
 * @return 限幅后的控制量
 * @note   两次计算的间隔由实例自己记（不用传）；同微秒重复调用按 1 微秒
 *         算，不会除以 0；禁止在中断里调用
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif /* PID_H */
