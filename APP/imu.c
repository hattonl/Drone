#include "imu.h"

#include "includes.h"
#include "mymath.h"

//#define Kp 0.6f                
// proportional gain governs rate of
//convergence to accelerometer/magnetometer #define Ki 0.1f
//// 0.001  integral gain governs rate of convergence of gyroscope biases

#define ANGLE_TO_RADIAN 0.01745329f
#define IMU_INTEGRAL_LIM (2.0f * ANGLE_TO_RADIAN)
#define NORM_ACC_LPF_HZ 10  //(Hz)
#define REF_ERR_LPF_HZ 1    //(Hz)

#define Kp 3.5f
#define Ki 0.3f
#define halfT 0.002f

typedef struct {
    float x;
    float y;
    float z;
} xyz_f_t;

typedef struct {
    xyz_f_t err;
    xyz_f_t err_tmp;
    xyz_f_t err_lpf;
    xyz_f_t err_Int;
    xyz_f_t g;

} ref_t;

xyz_f_t reference_v;
ref_t ref;

// xyz_f_t Gravity_Vec;
//解算的重力向量

float Roll, Pitch, Yaw;  //姿态角

float ref_q[4] = {1, 0, 0, 0};
float norm_acc, norm_q;
float norm_acc_lpf;

float mag_norm, mag_norm_xyz;

xyz_f_t mag_sim_3d;

extern u8 fly_ready;

#define _xyz_f_t xyz_f_t
void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in,
                     _xyz_f_t *out)  //小范围内正确。
{
    static s8 pn;
    static float h_tmp_x, h_tmp_y;

    h_tmp_x = my_sqrt(my_pow(ref->z) + my_pow(ref->y));
    h_tmp_y = my_sqrt(my_pow(ref->z) + my_pow(ref->x));

    pn = ref->z < 0 ? -1 : 1;

    out->x = (h_tmp_x * in->x - pn * ref->x * in->z);
    out->y = (pn * h_tmp_y * in->y - ref->y * in->z);

    // out->x = h_tmp_x *in->x - ref->x *in->z;
    //  out->y = ref->z *in->y - ref->y *in->z;

    out->z = ref->x * in->x + ref->y * in->y + ref->z * in->z;
}

void IMUupdate(float half_T, float gx, float gy, float gz, float ax, float ay,
               float az, float mx, float my, float mz, float *rol, float *pit,
               float *yaw) {
    float ref_err_lpf_hz;
    static float yaw_correct;
    float mag_norm_tmp;
    static xyz_f_t mag_tmp;
    static float yaw_mag;

    mag_norm_tmp = 20 * (6.28f * half_T);

    mag_norm_xyz = my_sqrt(mx * mx + my * my + mz * mz);

    if (mag_norm_xyz != 0) {
        mag_tmp.x += mag_norm_tmp * ((float)mx / (mag_norm_xyz)-mag_tmp.x);
        mag_tmp.y += mag_norm_tmp * ((float)my / (mag_norm_xyz)-mag_tmp.y);
        mag_tmp.z += mag_norm_tmp * ((float)mz / (mag_norm_xyz)-mag_tmp.z);
    }

    simple_3d_trans(&reference_v, &mag_tmp, &mag_sim_3d);

    mag_norm =
        my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y * mag_sim_3d.y);

    if (mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 &&
        mag_norm != 0) {
        yaw_mag =
            fast_atan2((mag_sim_3d.y / mag_norm), (mag_sim_3d.x / mag_norm)) *
            57.3f;
    }
    //=============================================================================
    // 计算等效重力向量
    reference_v.x = 2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2]);
    reference_v.y = 2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]);
    reference_v.z =
        1 -
        2 * (ref_q[1] * ref_q[1] +
             ref_q[2] * ref_q[2]);  // ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] -
                                    // ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

    //这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
    //根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
    //所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
    //=============================================================================

    // 计算加速度向量的模
    norm_acc = my_sqrt(ax * ax + ay * ay + az * az);
    norm_acc_lpf += NORM_ACC_LPF_HZ * (6.28f * half_T) *
                    (norm_acc - norm_acc_lpf);  // 10hz *3.14 * 2*0.001

    if (ABS(ax) < 4400 && ABS(ay) < 4400 && ABS(az) < 4400) {
        //把加计的三维向量转成单位向量。
        ax = ax / norm_acc;  // 4096.0f;
        ay = ay / norm_acc;  // 4096.0f;
        az = az / norm_acc;  // 4096.0f;

        if (3800 < norm_acc && norm_acc < 4400) {
            /* 叉乘得到误差 */
            ref.err_tmp.x = ay * reference_v.z - az * reference_v.y;
            ref.err_tmp.y = az * reference_v.x - ax * reference_v.z;
            // ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;

            /* 误差低通 */
            ref_err_lpf_hz = REF_ERR_LPF_HZ * (6.28f * half_T);
            ref.err_lpf.x += ref_err_lpf_hz * (ref.err_tmp.x - ref.err_lpf.x);
            ref.err_lpf.y += ref_err_lpf_hz * (ref.err_tmp.y - ref.err_lpf.y);
            //			 ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  -
            //ref.err_lpf.z );

            ref.err.x = ref.err_lpf.x;  //
            ref.err.y = ref.err_lpf.y;  //
                                        //ref.err.z = ref.err_lpf.z ;
        }
    } else {
        ref.err.x = 0;
        ref.err.y = 0;
        //		ref.err.z = 0 ;
    }
    /* 误差积分 */
    ref.err_Int.x += ref.err.x * Ki * 2 * half_T;
    ref.err_Int.y += ref.err.y * Ki * 2 * half_T;
    ref.err_Int.z += ref.err.z * Ki * 2 * half_T;

    /* 积分限幅 */
    ref.err_Int.x = LIMIT(ref.err_Int.x, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
    ref.err_Int.y = LIMIT(ref.err_Int.y, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
    ref.err_Int.z = LIMIT(ref.err_Int.z, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

    if (reference_v.z > 0.0f) {
        yaw_correct = Kp * 0.2f * To_180_degrees(yaw_mag - Yaw);
    }

    ref.g.x = (gx - reference_v.x * yaw_correct) * ANGLE_TO_RADIAN +
              (Kp * (ref.err.x + ref.err_Int.x));  // IN RADIAN
    ref.g.y = (gy - reference_v.y * yaw_correct) * ANGLE_TO_RADIAN +
              (Kp * (ref.err.y + ref.err_Int.y));  // IN RADIAN
    ref.g.z = (gz - reference_v.z * yaw_correct) * ANGLE_TO_RADIAN;

    /* 用叉积误差来做PI修正陀螺零偏 */

    // integrate quaternion rate and normalise
    ref_q[0] +=
        (-ref_q[1] * ref.g.x - ref_q[2] * ref.g.y - ref_q[3] * ref.g.z) * half_T;
    ref_q[1] +=
        (ref_q[0] * ref.g.x + ref_q[2] * ref.g.z - ref_q[3] * ref.g.y) * half_T;
    ref_q[2] +=
        (ref_q[0] * ref.g.y - ref_q[1] * ref.g.z + ref_q[3] * ref.g.x) * half_T;
    ref_q[3] +=
        (ref_q[0] * ref.g.z + ref_q[1] * ref.g.y - ref_q[2] * ref.g.x) * half_T;

    /* 四元数规一化 normalise quaternion */
    norm_q = my_sqrt(ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1] +
                     ref_q[2] * ref_q[2] + ref_q[3] * ref_q[3]);
    ref_q[0] = ref_q[0] / norm_q;
    ref_q[1] = ref_q[1] / norm_q;
    ref_q[2] = ref_q[2] / norm_q;
    ref_q[3] = ref_q[3] / norm_q;

    *rol = fast_atan2(2 * (ref_q[0] * ref_q[1] + ref_q[2] * ref_q[3]),
                      1 - 2 * (ref_q[1] * ref_q[1] + ref_q[2] * ref_q[2])) *
           57.3f;
    *pit = asin(2 * (ref_q[1] * ref_q[3] - ref_q[0] * ref_q[2])) * 57.3f;
    //Yaw   = ( - fast_atan2(2*(ref_q[1]*ref_q[2] +
    // ref_q[0]*ref_q[3]),ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] -
    // ref_q[2]*ref_q[2] - ref_q[3]*ref_q[3]) )* 57.3;
    *yaw = fast_atan2(2 * (-ref_q[1] * ref_q[2] - ref_q[0] * ref_q[3]),
                      2 * (ref_q[0] * ref_q[0] + ref_q[1] * ref_q[1]) - 1) * 57.3f;
                   //*yaw = yaw_mag;
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                           float az, float *rol, float *pit, float *yaw) {
    float norm;

    float vx, vy, vz;

    float ex, ey, ez;

    norm = my_sqrt(ax * ax + ay * ay + az * az);

    ax = ax / norm;

    ay = ay / norm;

    az = az / norm;

    vx = 2 * (q1 * q3 - q0 * q2);

    vy = 2 * (q0 * q1 + q2 * q3);

    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);

    ey = (az * vx - ax * vz);

    ez = (ax * vy - ay * vx);

    exInt = exInt + ex * Ki;

    eyInt = eyInt + ey * Ki;

    ezInt = ezInt + ez * Ki;

    gx = gx + Kp * ex + exInt;

    gy = gy + Kp * ey + eyInt;

    gz = gz + Kp * ez + ezInt;

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;

    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;

    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;

    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = my_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    q0 = q0 / norm;

    q1 = q1 / norm;

    q2 = q2 / norm;

    q3 = q3 / norm;

    *rol = (float)fast_atan2(2 * q0 * q1 + 2 * q2 * q3,
                             1 - 2 * q1 * q1 - 2 * q2 * q2) *
           57.3f;
    *pit = (float)asin(2 * q3 * q1 - 2 * q0 * q2) * 57.3f;
    *yaw = (float)fast_atan2(-2 * q0 * q3 - 2 * q1 * q2,
                             2 * q0 * q0 + 2 * q1 * q1 - 1) *
           57.3f;

    *yaw += 90;
    if (*yaw >= 180) yaw -= 360;
}
