#include "application.h"

#include "includes.h"
#include "main.h"

//===============定义优先级===============
#define TASK_START_PRIO      3
#define TASK_SHOWSTATE_PRIO  5
#define TASK_CONTROL_PRIO    8
#define TASK_ATTITUDE_PRIO   9
#define TASK_RCDATAPROC_PROC 10
#define TASK_SENDDATA_PRIO   11
#define TASK_RECEDATA_PRIO   14

//===============定义栈大小================
#define TASK_START_STK_SIZE      512
#define TASK_SHOWSTATE_STK_SIZE  512
#define TASK_CONTROL_STK_SIZE    512
#define TASK_ATTITUDE_STK_SIZE   512
#define TASK_RCDATAPROC_STK_SIZE 512
#define TASK_SENDDATA_STK_SIZE   512
#define TASK_RECEDATA_STK_SIZE   512

//================定义栈===================
CPU_STK TASK_START_STK[TASK_START_STK_SIZE];
CPU_STK TASK_SHOWSTATE_STK[TASK_SHOWSTATE_STK_SIZE];
CPU_STK TASK_CONTROL_STK[TASK_CONTROL_STK_SIZE];
CPU_STK TASK_ATTITUDE_STK[TASK_ATTITUDE_STK_SIZE];
CPU_STK TASK_RCDATAPROC_STK[TASK_RCDATAPROC_STK_SIZE];
CPU_STK TASK_SENDDATA_STK[TASK_SENDDATA_STK_SIZE];
CPU_STK TASK_RECEDATA_STK[TASK_RECEDATA_STK_SIZE];

//==============定义任务控制块=============
OS_TCB TaskStartTCB;
OS_TCB TaskShowStateTCB;
OS_TCB TaskControlTCB;
OS_TCB TaskAttitudeTCB;
OS_TCB TaskRcDataProcTCB;
OS_TCB TaskSendDataTCB;
OS_TCB TaskReceDataTCB;

//================函数声明===================
void Task_Start(void *p_arg);
void Task_ShowState(void *p_arg);
void Task_Control(void *p_arg);
void Task_Attitude(void *p_arg);
void Task_RcDataProc(void *p_arg);
void Task_SendData(void *p_arg);
void Task_ReceData(void *p_arg);

//===============定义内存池================
#define USARTQTY  12
#define USARTSIZE 100

OS_MEM UsartMemory;
CPU_INT08U UsartMemoryStorage[USARTQTY][USARTSIZE];

//=========全局存储区,任务间消息传递===========
u16 Glob_Pwm[CH_NUM] = {1500, 1000, 1500, 1500};
volatile StGy86_t Glob_Gy86;
Spid_t Glob_PID_Tran;
int16_t Glob_motor[4];

//=========互斥信号量，保护全局存储区============
OS_MUTEX MutexPWM;
OS_MUTEX MutexGy86;
OS_MUTEX MutexMotor;

//===========定义信号量用于同步================
OS_SEM SemParaPID;

//===================外部变量===================
extern u16 Tim3_Pwm_Capt[CH_NUM];
extern StrPIDContrl PitCont;
extern StrPIDContrl RolCont;
extern StrPIDContrl YawCont;

//==============与上位机通讯的联合体变量===============
S_Trans_gy86_t Trans_Gy86;
U_Trans_u16_t Trans_Pwm[4];
U_Trans_int16_t Trans_Motor[4];

/**********************************************************/
/*
 * 函数名：Task_Start
 * 描述  ：开始任务，创建其他任务
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_Start(void *p_arg) {
    OS_ERR err;
    CPU_SR_ALLOC();
    p_arg = p_arg;

    CPU_Init();
    OS_CRITICAL_ENTER();  //进入临界区

    //=======================创建任务==============================

    //创建显示状态任务
    OSTaskCreate(
        (OS_TCB *)&TaskShowStateTCB,
        (CPU_CHAR *)"Task_ShowState",
        (OS_TASK_PTR)Task_ShowState,
        (void *)0, (OS_PRIO)TASK_SHOWSTATE_PRIO,
        (CPU_STK *)&TASK_SHOWSTATE_STK[0],
        (CPU_STK_SIZE)TASK_SHOWSTATE_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_SHOWSTATE_STK_SIZE, 
        (OS_MSG_QTY)0, 
        (OS_TICK)0,
        (void *)0, 
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);

    //创建控制任务
    OSTaskCreate(
        (OS_TCB *)&TaskControlTCB,
        (CPU_CHAR *)"Task_Control",
        (OS_TASK_PTR)Task_Control,
        (void *)0,
        (OS_PRIO)TASK_CONTROL_PRIO,
        (CPU_STK *)&TASK_CONTROL_STK[0],
        (CPU_STK_SIZE)TASK_CONTROL_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_CONTROL_STK_SIZE,
        (OS_MSG_QTY)0,
        (OS_TICK)0,
        (void *)0,
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);

    //创建姿态任务
    OSTaskCreate(
        (OS_TCB *)&TaskAttitudeTCB,
        (CPU_CHAR *)"Task_Attitude",
        (OS_TASK_PTR)Task_Attitude, 
        (void *)0,
        (OS_PRIO)TASK_ATTITUDE_PRIO,
        (CPU_STK *)&TASK_ATTITUDE_STK[0],
        (CPU_STK_SIZE)TASK_ATTITUDE_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_ATTITUDE_STK_SIZE, 
        (OS_MSG_QTY)0,
        (OS_TICK)0,
        (void *)0,
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);
                    
    //创建遥控器信号处理任务
    OSTaskCreate(
        (OS_TCB *)&TaskRcDataProcTCB,
        (CPU_CHAR *)"Task_RcDataProc",
        (OS_TASK_PTR)Task_RcDataProc,
        (void *)0,
        (OS_PRIO)TASK_RCDATAPROC_PROC,
        (CPU_STK *)&TASK_RCDATAPROC_STK[0],
        (CPU_STK_SIZE)TASK_RCDATAPROC_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_RCDATAPROC_STK_SIZE,
        (OS_MSG_QTY)0,
        (OS_TICK)0,
        (void *)0,
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);

    //创建数据发送任务
    OSTaskCreate(
        (OS_TCB *)&TaskSendDataTCB,
        (CPU_CHAR *)"Task_SendData",
        (OS_TASK_PTR)Task_SendData, 
        (void *)0,
        (OS_PRIO)TASK_SENDDATA_PRIO, 
        (CPU_STK *)&TASK_SENDDATA_STK[0],
        (CPU_STK_SIZE)TASK_SENDDATA_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_SENDDATA_STK_SIZE, 
        (OS_MSG_QTY)0,
        (OS_TICK)0, 
        (void *)0,
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);

    //创建数据接收处理任务
    OSTaskCreate((OS_TCB *)&TaskReceDataTCB,
        (CPU_CHAR *)"Task_ReceData",
        (OS_TASK_PTR)Task_ReceData, 
        (void *)0,
        (OS_PRIO)TASK_RECEDATA_PRIO, 
        (CPU_STK *)&TASK_RECEDATA_STK[0],
        (CPU_STK_SIZE)TASK_RECEDATA_STK_SIZE / 10,
        (CPU_STK_SIZE)TASK_RECEDATA_STK_SIZE, 
        (OS_MSG_QTY)10,
        (OS_TICK)0, 
        (void *)0,
        (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
        (OS_ERR *)&err);

    //==================创建内存池=============================
    OSMemCreate(
        (OS_MEM *)&UsartMemory,
        (CPU_CHAR *)"UsartMemory",
        (void *)&UsartMemoryStorage[0][0], 
        (OS_MEM_QTY)USARTQTY,
        (OS_MEM_SIZE)USARTSIZE, 
        (OS_ERR *)&err);

    //================创建互斥型信号量========================
    OSMutexCreate(
        (OS_MUTEX *)&MutexPWM,
        (CPU_CHAR *)"MutexPWM",
        (OS_ERR *)&err);

    OSMutexCreate(
        (OS_MUTEX *)&MutexGy86, 
        (CPU_CHAR *)"MutexGy86",
        (OS_ERR *)&err);

    OSMutexCreate(
        (OS_MUTEX *)&MutexMotor,
        (CPU_CHAR *)"MutexMotor",
        (OS_ERR *)&err);

    //=============创建信号量用于同步===========================
    OSSemCreate(
        (OS_SEM *)&SemParaPID,
        (CPU_CHAR *)"SemParaPID",
        (OS_SEM_CTR)0,
        (OS_ERR *)&err);

    OS_TaskSuspend((OS_TCB *)&TaskStartTCB, &err);  //挂起开始任务

    OS_CRITICAL_EXIT();
}

/**********************************************************/
/*
 * 函数名：Task_ShowState
 * 描述  ：通过LED灯显示目前飞行器状态
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_ShowState(void *p_arg) {
    OS_ERR err;
    p_arg = p_arg;
    while (1) {
        OS_TaskSuspend((OS_TCB *)&TaskShowStateTCB, &err);
    }
}

/**********************************************************/
/*
 * 函数名：Task_Control
 * 描述  ：PID控制器
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_Control(void *p_arg) {
    OS_ERR err;

    u8 run_times = 0;
    float myRoll, myPitch, myYaw;
    float mygx, mygy, mygz;
    u16 CH[CH_NUM] = {1500, 1000, 1500, 1500};
    int16_t motor_ts[4];

    CPU_TS ts_Outer[2];
    CPU_TS ts_Inner[2];
    float loop_time_Outer;
    float loop_time_Inner;

    OS_SEM_CTR SemCtr;

    p_arg = p_arg;

    while (1) {
        // OS_TaskSuspend((OS_TCB*)&TaskControlTCB,&err);
        //等待一个任务内建信号量
        SemCtr = OSTaskSemPend(
                    (OS_TICK)0,
                    (OS_OPT)OS_OPT_PEND_BLOCKING,
                    (CPU_TS *)NULL, 
                    (OS_ERR *)&err);
        if (SemCtr != 0) {
            //一定出现了错误，逾期未执行
            OSTaskSemSet(
                (OS_TCB *)&TaskControlTCB, 
                (OS_SEM_CTR)0,
                (OS_ERR *)&err);
        }

        //全局变量为局部变量赋值

        OSMutexPend(
            (OS_MUTEX *)&MutexGy86, 
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);

        myRoll  = Glob_Gy86.Rol;
        myPitch = Glob_Gy86.Pit;
        myYaw   = Glob_Gy86.Yaw;
        mygx    = Glob_Gy86.Gyro.x;
        mygy    = Glob_Gy86.Gyro.y;
        mygz    = Glob_Gy86.Gyro.z;

        //对全局变量进行赋值
        OSMutexPost(
            (OS_MUTEX *)&MutexGy86, 
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        OSMutexPend(
            (OS_MUTEX *)&MutexPWM, 
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);
        //对全局变量进行赋值
        Rc_DataTrans(Glob_Pwm, CH);

        OSMutexPost(
            (OS_MUTEX *)&MutexPWM, 
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        //判断是否PID已被修改
        OSSemPend(
            (OS_SEM *)&SemParaPID, 
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_NON_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);
        if (Glob_PID_Tran.is == 1) {
            switch (Glob_PID_Tran.opt) {
                case XO:
                    PitCont.Outer.P = Glob_PID_Tran.P;
                    PitCont.Outer.I = Glob_PID_Tran.I;
                    PitCont.Outer.D = Glob_PID_Tran.D;
                    PitCont.Outer.Integ = 0;
                    PitCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;
                    break;

                case XI:
                    PitCont.Inner.P = Glob_PID_Tran.P;
                    PitCont.Inner.I = Glob_PID_Tran.I;
                    PitCont.Inner.D = Glob_PID_Tran.D;
                    PitCont.Outer.Integ = 0;
                    PitCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;

                    break;

                case YO:
                    RolCont.Outer.P = Glob_PID_Tran.P;
                    RolCont.Outer.I = Glob_PID_Tran.I;
                    RolCont.Outer.D = Glob_PID_Tran.D;
                    RolCont.Outer.Integ = 0;
                    RolCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;

                    break;

                case YI:
                    RolCont.Inner.P = Glob_PID_Tran.P;
                    RolCont.Inner.I = Glob_PID_Tran.I;
                    RolCont.Inner.D = Glob_PID_Tran.D;
                    RolCont.Outer.Integ = 0;
                    RolCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;

                    break;

                case ZO:
                    YawCont.Outer.P = Glob_PID_Tran.P;
                    YawCont.Outer.I = Glob_PID_Tran.I;
                    YawCont.Outer.D = Glob_PID_Tran.D;
                    YawCont.Outer.Integ = 0;
                    YawCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;

                    break;

                case ZI:
                    YawCont.Inner.P = Glob_PID_Tran.P;
                    YawCont.Inner.I = Glob_PID_Tran.I;
                    YawCont.Inner.D = Glob_PID_Tran.D;
                    YawCont.Outer.Integ = 0;
                    YawCont.Inner.Integ = 0;
                    Glob_PID_Tran.opt = NO;
                    break;
                default:
                    break;
            }

            Glob_PID_Tran.is = 0;
        }

        if (run_times == 1) {
            ts_Outer[0] = ts_Outer[1];
            ts_Outer[1] = OS_TS_GET();
            if (ts_Outer[1] > ts_Outer[0]) {
                loop_time_Outer = (ts_Outer[1] - ts_Outer[0]) / 100000000.0f;
            } else {
                loop_time_Outer =
                    (ts_Outer[1] - ts_Outer[0] + 0xffffffff + 1) / 100000000.0f;
            }

            //外环控制
            Control_Outer(loop_time_Outer, CH, myRoll, myPitch, myYaw);
        }

        ts_Inner[0] = ts_Inner[1];
        ts_Inner[1] = OS_TS_GET();

        if (ts_Inner[1] > ts_Outer[0]) {
            loop_time_Inner = (ts_Inner[1] - ts_Inner[0]) / 100000000.0f;
        } else {
            loop_time_Inner =
                (ts_Inner[1] - ts_Inner[0] + 0xffffffff + 1) / 100000000.0f;
        }

        //内环控制

        Control_Inner(loop_time_Inner, CH, mygx, mygy, mygz, motor_ts);

        OSMutexPend(
            (OS_MUTEX *)&MutexMotor, 
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING,
            (CPU_TS *)NULL,
            (OS_ERR *)&err);

        Glob_motor[0] = motor_ts[0];
        Glob_motor[1] = motor_ts[1];
        Glob_motor[2] = motor_ts[2];
        Glob_motor[3] = motor_ts[3];

        OSMutexPost(
            (OS_MUTEX *)&MutexMotor, 
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        run_times++;
        if (run_times > 1) run_times = 0;
    }
}

/**********************************************************/
/*
 * 函数名：Task_Attitude
 * 描述  ：惯性测量单元的姿态解算
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_Attitude(void *p_arg) {
    OS_ERR err;
    u8 run_times = 0;
    CPU_TS ts[2];
    float loop_time;

    u8 mpu_6050_buf[14];

    float mx0, my0, mz0;
    short ax, ay, az, gx, gy, gz;

    OS_SEM_CTR SemCtr;

    p_arg = p_arg;

    while (1) {
        // OS_TaskSuspend((OS_TCB*)&TaskAttitudeTCB,&err);
        //等待一个任务内建信号量
        SemCtr = OSTaskSemPend(
            (OS_TICK)0, 
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL, 
            (OS_ERR *)&err);

        if (SemCtr != 0) {
            //一定出现了错误，逾期未执行,可以增加错误处理
            OSTaskSemSet(
                (OS_TCB *)&TaskAttitudeTCB, 
                (OS_SEM_CTR)0,
                (OS_ERR *)&err);
        }
        //在这里关一下任务调度器

        MPU_ReadData(mpu_6050_buf);

        //开任务调度器

        MPU_DataProc(mpu_6050_buf, &ax, &ay, &az, &gx, &gy, &gz);

        if (run_times == 3) {
            Read_HMC5883L(&mx0, &my0, &mz0);  //周期为10ms
        } else if (run_times % 2 == 0) {
            //姿态解算
            ts[0] = ts[1];
            ts[1] = OS_TS_GET();

            if (ts[1] > ts[0]) {
                loop_time = (ts[1] - ts[0]) / 100000000.0f;
            } else {
                loop_time = (ts[1] - ts[0] + 0xffffffff + 1) / 100000000.0f;
            }
 
            IMUupdate(loop_time / 2, (float)gx * 0.06103f, (float)gy * 0.06103f,
                      (float)gz * 0.06103f, (float)ax, (float)ay, (float)az,
                      mx0, my0, mz0, &Roll, &Pitch, &Yaw);
        }

        run_times++;
        if (run_times >= 4) run_times = 0;

        OSMutexPend(
            (OS_MUTEX *)&MutexGy86,
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);

        //对全局变量进行赋值
        Glob_Gy86.Acce.x = (float)ax;
        Glob_Gy86.Acce.y = (float)ay;
        Glob_Gy86.Acce.z = (float)az;

        Glob_Gy86.Gyro.x = (float)gx * 0.06103f;
        Glob_Gy86.Gyro.y = (float)gy * 0.06103f;
        Glob_Gy86.Gyro.z = (float)gz * 0.06103f;

        Glob_Gy86.Mag.x = mx0;
        Glob_Gy86.Mag.y = my0;
        Glob_Gy86.Mag.z = mz0;

        Glob_Gy86.Pit = Pitch;
        Glob_Gy86.Rol = Roll;
        Glob_Gy86.Yaw = Yaw;

        OSMutexPost(
            (OS_MUTEX *)&MutexGy86,
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        //        printf("myRoll = %f , myPitch = %f ,myYaw = %f , mygx = %f
        //,mygy = %f , mygz = %f
        //\r\n",Glob_Gy86.Rol,Glob_Gy86.Pit,Glob_Gy86.Yaw,Glob_Gy86.Gyro.x,Glob_Gy86.Gyro.y,Glob_Gy86.Gyro.z);
        // printf("Roll = %.2f ,Pitch = %.2f ,Yaw = %.2f \r\n",Roll,Pitch,Yaw);

        // printf("123 \r\n");
        // printf("g(x,y,z):%d %d %d , a(x,y,z):%d %d %d \r\n",gx, gy, gz,ax,
        // ay, az); printf("Hello\r\n");

        // OSTimeDly(2,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}

/**********************************************************/
/*
 * 函数名：Task_RcDataProc
 * 描述  ：处理遥控器的数据，滤波或纠错处理等
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_RcDataProc(void *p_arg) {
    OS_ERR err;

    u16 Tim3_Pwm[CH_NUM] = {1500, 1000, 1500, 1500};
    u16 Rc_Pwm_Capt[CH_NUM] = {1500, 1000, 1500, 1500};

    p_arg = p_arg;

    while (1) {
        Rc_DataTrans(
            Tim3_Pwm_Capt,
            Tim3_Pwm);  //从中断中接受数据，单独处理。 //方便后期添加各种控制

        rc_data_proc(Tim3_Pwm, Rc_Pwm_Capt);  //滤波，死区控制

        OSMutexPend(
            (OS_MUTEX *)&MutexPWM, 
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);

        // printf("myRoll = %f , myPitch = %f ,myYaw = %f , mygx = %f ,mygy = %f
        // , mygz = %f
        // \r\n",Glob_Gy86.Rol,Glob_Gy86.Pit,Glob_Gy86.Yaw,Glob_Gy86.Gyro.x,Glob_Gy86.Gyro.y,Glob_Gy86.Gyro.z);

        //对全局变量进行赋值
        Rc_DataTrans(Rc_Pwm_Capt, Glob_Pwm);

        OSMutexPost(
            (OS_MUTEX *)&MutexPWM,
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        OSTimeDly(2, OS_OPT_TIME_PERIODIC, &err);
    }
}

/**********************************************************/
/*
 * 函数名：Task_SendData
 * 描述  ：向上位机发送数据
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_SendData(void *p_arg) {
    OS_ERR err;

    u8 run_times = 0;
    StGy86_t Local_Gy86;
    u16 CH[CH_NUM] = {1500, 1000, 1500, 1500};
    int16_t Local_motor[4];

    p_arg = p_arg;

    while (1) {
        DMA_BUF_Count_Init();
        // 10ms周期发送

        OSMutexPend(
            (OS_MUTEX *)&MutexGy86,
            (OS_TICK)0,
            (OS_OPT)OS_OPT_PEND_BLOCKING, 
            (CPU_TS *)NULL,
            (OS_ERR *)&err);

        Local_Gy86 = Glob_Gy86;

        OSMutexPost(
            (OS_MUTEX *)&MutexGy86, 
            (OS_OPT)OS_OPT_POST_NO_SCHED,
            (OS_ERR *)&err);

        //传递函数，写在这里
        ANO_DT_Send_Status(Local_Gy86.Rol, Local_Gy86.Pit, Local_Gy86.Yaw, 0, 0,
                           1);
        ANO_DT_Send_Senser(Local_Gy86.Acce.x, Local_Gy86.Acce.y,
                           Local_Gy86.Acce.z, Local_Gy86.Gyro.x,
                           Local_Gy86.Gyro.y, Local_Gy86.Gyro.z,
                           Glob_Gy86.Mag.x, Glob_Gy86.Mag.y, Glob_Gy86.Mag.z);
        ANO_DT_Send_Speed(PitCont.Inner.out_P, PitCont.Inner.out_I,
                          PitCont.Inner.out_D);  //对应内环的PID值
        if (run_times == 1)                      // 20ms周期发送
        {
            OSMutexPend(
                (OS_MUTEX *)&MutexPWM,
                (OS_TICK)0,
                (OS_OPT)OS_OPT_PEND_BLOCKING,
                (CPU_TS *)NULL,
                (OS_ERR *)&err);
            //对全局变量进行赋值
            Rc_DataTrans(Glob_Pwm, CH);

            OSMutexPost(
                (OS_MUTEX *)&MutexPWM,
                (OS_OPT)OS_OPT_POST_NO_SCHED,
                (OS_ERR *)&err);
            //传递函数，写在这里

            ANO_DT_Send_RCData(CH[THR], CH[YAW], CH[ROL], CH[PIT], 0, 0, 0, 0,
                               0, 0);

            OSMutexPend(
                (OS_MUTEX *)&MutexMotor,
                (OS_TICK)0,
                (OS_OPT)OS_OPT_PEND_BLOCKING,
                (CPU_TS *)NULL,
                (OS_ERR *)&err);

            Local_motor[0] = Glob_motor[0];
            Local_motor[1] = Glob_motor[1];
            Local_motor[2] = Glob_motor[2];
            Local_motor[3] = Glob_motor[3];

            OSMutexPost(
                (OS_MUTEX *)&MutexMotor,
                (OS_OPT)OS_OPT_POST_NO_SCHED,
                (OS_ERR *)&err);

            //传递函数，写在这里

            ANO_DT_Send_MotoPWM(Local_motor[0], Local_motor[1], Local_motor[2],
                                Local_motor[3], 0, 0, 0, 0);
        }

        run_times++;
        if (run_times >= 2) run_times = 0;

        Usart2_DMA_Send_Start();

        OSTimeDly(5, OS_OPT_TIME_PERIODIC, &err);
    }
}

/**********************************************************/
/*
 * 函数名：Task_ReceData
 * 描述  ：接收上位机数据并分析
 * 输入  ：无
 * 输出  ：无
 * 说明  ：无
 */
/**********************************************************/
void Task_ReceData(void *p_arg) {
    OS_ERR err;

    u8 *RxData;
    OS_MSG_SIZE RxSize;

    u8 index = 4;
    u8 index_t = 0;

    u8 CharTemp[10];

    float Get_P, Get_I, Get_D;

    p_arg = p_arg;

    while (1) {
        RxData = (u8 *)OSTaskQPend(
                        (OS_TICK)0,
                        (OS_OPT)OS_OPT_PEND_BLOCKING,
                        (OS_MSG_SIZE *)&RxSize,
                        (CPU_TS *)NULL,
                        (OS_ERR *)&err);

        if (RxData[0] == '<' && RxData[RxSize - 1] == '>') {
            if (RxData[1] == 'P') {
                index = 4;
                // get P
                index_t = 0;
                memset(CharTemp, 0, 10);
                while (RxData[index] != '#') {
                    CharTemp[index_t++] = RxData[index++];
                }
                CharTemp[index_t] = '\0';
                Get_P = atof((const char *)CharTemp);
                // get I
                index++;
                index_t = 0;
                memset(CharTemp, 0, 10);
                while (RxData[index] != '#') {
                    CharTemp[index_t++] = RxData[index++];
                }
                CharTemp[index_t] = '\0';

                Get_I = atof((const char *)CharTemp);

                // get D
                index++;
                index_t = 0;
                memset(CharTemp, 0, 10);
                while (RxData[index] != '#') {
                    CharTemp[index_t++] = RxData[index++];
                }
                CharTemp[index_t] = '\0';

                Get_D = atof((const char *)CharTemp);

                if (Glob_PID_Tran.is == 0) {
                    //判断是哪个？对结构体进行赋值
                    if (RxData[2] == 'X') {
                        if (RxData[3] == 'O') {
                            Glob_PID_Tran.opt = XO;
                        } else if (RxData[3] == 'I') {
                            Glob_PID_Tran.opt = XI;
                        } else
                            Glob_PID_Tran.opt = NO;
                    } else if (RxData[2] == 'Y') {
                        if (RxData[3] == 'O') {
                            Glob_PID_Tran.opt = YO;
                        } else if (RxData[3] == 'I') {
                            Glob_PID_Tran.opt = YI;
                        } else
                            Glob_PID_Tran.opt = NO;
                    } else if (RxData[2] == 'Z') {
                        if (RxData[3] == 'O') {
                            Glob_PID_Tran.opt = ZO;
                        } else if (RxData[3] == 'I') {
                            Glob_PID_Tran.opt = ZI;
                        } else
                            Glob_PID_Tran.opt = NO;
                    } else
                        Glob_PID_Tran.opt = NO;

                    Glob_PID_Tran.P = Get_P;
                    Glob_PID_Tran.I = Get_I;
                    Glob_PID_Tran.D = Get_D;

                    Glob_PID_Tran.is = 1;
                }
            }
        }

        //串口数据处理

        //返还内存块
        OSMemPut((OS_MEM *)&UsartMemory, (void *)RxData, (OS_ERR *)&err);
        RxData = NULL;
        RxSize = 0;
    }
}
