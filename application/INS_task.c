/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "ahrs.h"
#include "user_lib.h"

#include "calibrate_task.h"
#include "detect_task.h"

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm给定

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f},                  \
        {-1.0f, 0.0f, 0.0f},             \
    {                                    \
        0.0f, 0.0f, 1.0f                 \
    }

#define IST8310_BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f},                   \
        {0.0f, 1.0f, 0.0f},               \
    {                                     \
        0.0f, 0.0f, 1.0f                  \
    }

/**
 * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
 *                 different install derection.
 * @param[out]     gyro: after plus zero drift and rotate
 * @param[out]     accel: after plus zero drift and rotate
 * @param[out]     mag: after plus zero drift and rotate
 * @param[in]      bmi088: gyro and accel data
 * @param[in]      ist8310: mag data
 * @retval         none
 */
/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
 * @brief          control the temperature of bmi088
 * @param[in]      temp: the temperature of bmi088
 * @retval         none
 */
/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp);
/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f; // tast run time , unit s.任务运行的时间 单位 s

// 加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f}; // euler angle, unit rad.欧拉角 单位 rad

/*-------------------- 玺佬的姿态解算代码 --------------------*/
#define TRUE 1
#define FALSE 0
float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};

float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 10000};
// float IMU_QuaternionEKF_K[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
//                                  0.1, 100000, 0.1, 0.1, 0.1, 0.1,
//                                  0.1, 0.1, 100000, 0.1, 0.1, 0.1,
//                                  0.1, 0.1, 0.1, 100000, 0.1, 0.1,
//                                  0.1, 0.1, 0.1, 0.1, 10000, 0.1,
//                                  0.1, 0.1, 0.1, 0.1, 0.1, 10000};
// float IMU_QuaternionEKF_H[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
//                                  0.1, 100000, 0.1, 0.1, 0.1, 0.1,
//                                  0.1, 0.1, 100000, 0.1, 0.1, 0.1,
//                                  0.1, 0.1, 0.1, 100000, 0.1, 0.1,
//                                  0.1, 0.1, 0.1, 0.1, 10000, 0.1,
//                                  0.1, 0.1, 0.1, 0.1, 0.1, 10000};

float IMU_QuaternionEKF_Q[36] = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.000001, 0,
                                 0, 0, 0, 0, 0, 0.000001};
float IMU_QuaternionEKF_R[9] = {1000000, 0, 0,
                                0, 1000000, 0,
                                0, 0, 1000000};
static INS_t INS;
Angle_t angle;       // 欧拉角
Velocity_t velocity; // 角速度
Accel_t accel;       // 加速度

/*-------------------- 欧拉角测量部分 --------------------*/

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
static void IMU_QuaternionEKF_User_Func1(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);

/*-------------------- 角速度测量部分 --------------------*/
/*-------------------- 加速度测量部分 --------------------*/
/*-------------------- 数据指针获取部分 --------------------*/

static void AngleUpdate(void);
static void VelocityUpdate(void);
static void AccelUpdate(void);

/**
 * @brief          imu task, init bmi088, ist8310, calculate the euler angle
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */

void INS_task(void const *pvParameters)
{
    // wait a time
    osDelay(INS_TASK_INIT_TIME);
    while (BMI088_init())
    {
        osDelay(100);
    }
    while (ist8310_init())
    {
        osDelay(100);
    }

    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    // rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    // get the handle of task
    // 获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 1000000, 0.9996);

    while (1)
    {
        // wait spi DMA tansmit done
        // 等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        // rotate and zero drift
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

        // 加速度计低通滤波
        // accel low-pass filter
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

        AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

        // because no use ist8310 and save time, no use
        if (mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);
            //            ist8310_read_mag(ist8310_real_data.mag);
        }

        IMU_QuaternionEKF_Update(bmi088_real_data.gyro[0], bmi088_real_data.gyro[1], bmi088_real_data.gyro[2],
                                 bmi088_real_data.accel[0], bmi088_real_data.accel[1], bmi088_real_data.accel[2],
                                 timing_time);
        AngleUpdate();
    }
}

/*-------------------- 欧拉角测量部分 --------------------*/

/**
 * @brief Quaternion EKF initialization
 * @param[in]       quaternion process noise
 * @param[in]       gyro bias process noise
 * @param[in]       accel measure noise
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda)
{
    INS.Q1 = process_noise1;
    INS.Q2 = process_noise2;
    INS.R = measure_noise;
    INS.ChiSquareTestThreshold = 0.01f;
    INS.ConvergeFlag = 0;
    if (lambda > 1)
        lambda = 1;
    INS.lambda = lambda;
    Kalman_Filter_Init(&INS.IMU_QuaternionEKF, 6, 0, 3);
    INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    INS.IMU_QuaternionEKF.xhat_data[3] = 0;
    INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_User_Func1;
    INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;
    INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    INS.IMU_QuaternionEKF.SkipEq4 = TRUE;
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
    memcpy(INS.IMU_QuaternionEKF.Q_data, IMU_QuaternionEKF_Q, sizeof(IMU_QuaternionEKF_Q));
    memcpy(INS.IMU_QuaternionEKF.R_data, IMU_QuaternionEKF_R, sizeof(IMU_QuaternionEKF_R));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float halfgxdt, halfgydt, halfgzdt;
    static float accelInvNorm;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    INS.dt = dt;

    halfgxdt = 0.5f * (gx - INS.GyroBias[0]) * dt;
    halfgydt = 0.5f * (gy - INS.GyroBias[1]) * dt;
    halfgzdt = 0.5f * (gz - INS.GyroBias[2]) * dt;

    // 初始化F矩阵为单位阵
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    // 设置F矩阵用于过程更新
    INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // 归一化加速度向量作为量测向量
    accelInvNorm = invSqrt(ax * ax + ay * ay + az * az);
    INS.IMU_QuaternionEKF.MeasuredVector[0] = ax * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[1] = ay * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[2] = az * accelInvNorm;

    // 设置Q,R矩阵
    INS.IMU_QuaternionEKF.Q_data[0] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[7] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[14] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[21] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[28] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[35] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.R_data[0] = INS.R;
    INS.IMU_QuaternionEKF.R_data[4] = INS.R;
    INS.IMU_QuaternionEKF.R_data[8] = INS.R;

    // 卡尔曼滤波器更新
    Kalman_Filter_Update(&INS.IMU_QuaternionEKF);

    // 估计结果导出
    INS.q[0] = INS.IMU_QuaternionEKF.FilteredValue[0];
    INS.q[1] = INS.IMU_QuaternionEKF.FilteredValue[1];
    INS.q[2] = INS.IMU_QuaternionEKF.FilteredValue[2];
    INS.q[3] = INS.IMU_QuaternionEKF.FilteredValue[3];
    INS.GyroBias[0] = INS.IMU_QuaternionEKF.FilteredValue[4];
    INS.GyroBias[1] = INS.IMU_QuaternionEKF.FilteredValue[5];
    INS.GyroBias[2] = 0;

    // 四元数反解欧拉角
    INS.Yaw = atan2f(2.0f * (INS.q[0] * INS.q[3] + INS.q[1] * INS.q[2]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[1] * INS.q[1]) - 1.0f);
    INS.Pitch = -atan2f(2.0f * (INS.q[0] * INS.q[1] + INS.q[2] * INS.q[3]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[3] * INS.q[3]) - 1.0f);
    INS.Roll = asinf(-2.0f * (INS.q[1] * INS.q[3] - INS.q[0] * INS.q[2]));
}

static void IMU_QuaternionEKF_User_Func1(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
    static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 四元数归一化
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    // 补充F矩阵
    kf->F_data[4] = q1 * INS.dt / 2;
    kf->F_data[5] = q2 * INS.dt / 2;

    kf->F_data[10] = -q0 * INS.dt / 2;
    kf->F_data[11] = q3 * INS.dt / 2;

    kf->F_data[16] = -q3 * INS.dt / 2;
    kf->F_data[17] = -q0 * INS.dt / 2;

    kf->F_data[22] = q2 * INS.dt / 2;
    kf->F_data[23] = -q1 * INS.dt / 2;
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */

    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];
    // 复位H矩阵为0矩阵
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    // 设置H矩阵
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    // 计算残差方差 inv(H·P'(k)·HT + R)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    // 计算h(xhat'(k))
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算残差z(k) - h(xhat'(k))
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // 卡方检验 计算检验函数r
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // 检测函数
    INS.ChiSquare = kf->temp_matrix.pData[0];
    if (INS.ChiSquare < 0.1f * INS.ChiSquareTestThreshold)
        INS.ConvergeFlag = 1;
    // if (INS.ChiSquare > INS.ChiSquareTestThreshold && enChiSquareTest && INS.ConvergeFlag)//实在没明白enChiSquareTest的定义和作用，就先注释掉了
    if (INS.ChiSquare > INS.ChiSquareTestThreshold && INS.ConvergeFlag)
    {
        // 未通过卡方检验 仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        return;
    }
    else
    {
        // 应用渐消因子
        kf->P_data[28] /= INS.lambda;
        kf->P_data[35] /= INS.lambda;
        kf->SkipEq5 = FALSE;
    }

    // 通过卡方检验，进行量测更新
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
    kf->temp_vector.pData[3] = 0;                                                 // 应用M矩阵
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);      // xhat = xhat'(k) + M·K(k)·(z(k) - h(xhat'(k)))
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    // memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    // memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

static void AngleUpdate(void)
{
    angle.yaw = INS.Yaw;
    angle.pitch = INS.Pitch;
    angle.roll = INS.Roll;
}

/*-------------------- 角速度测量部分 --------------------*/

static void VelocityUpdate(void)
{
    velocity.x = INS_gyro[0];
    velocity.y = INS_gyro[1];
    velocity.z = INS_gyro[2];
}

/*-------------------- 加速度测量部分 --------------------*/

static void AccelUpdate(void)
{
    accel.x = INS_accel[0];
    accel.y = INS_accel[1];
    accel.z = INS_accel[2];
}

/**
 * @brief          获取欧拉角, 单位 rad
 * @param[in]      none
 * @retval         angle的指针
 */
const Angle_t *GetAnglePoint(void)
{
    return &angle;
}
/**
 * @brief          获取角速度, 单位 rad/s
 * @param[in]      none
 * @retval         velocity的指针
 */
const Velocity_t *GetVelocityPoint(void)
{
    return &velocity;
}
/**
 * @brief          获取加速度, 单位 m/s^2
 * @param[in]      none
 * @retval         accel的指针
 */
const Accel_t *GetAccelPoint(void)
{
    return &accel;
}

/*-------------------- 陈旧的姿态解算代码 --------------------*/

/**
 * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
 *                 different install derection.
 * @param[out]     gyro: after plus zero drift and rotate
 * @param[out]     accel: after plus zero drift and rotate
 * @param[out]     mag: after plus zero drift and rotate
 * @param[in]      bmi088: gyro and accel data
 * @param[in]      ist8310: mag data
 * @retval         none
 */
/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}

/**
 * @brief          control the temperature of bmi088
 * @param[in]      temp: the temperature of bmi088
 * @retval         none
 */
/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, get_control_temperature());
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        // 在没有达到设置的温度，一直最大功率加热
        // in beginning, max power
        if (temp > get_control_temperature())
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                // 达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
 * @brief          calculate gyro zero drift
 * @param[out]     gyro_offset:zero drift
 * @param[in]      gyro:gyro data
 * @param[out]     offset_time_count: +1 auto
 * @retval         none
 */
/**
 * @brief          计算陀螺仪零漂
 * @param[out]     gyro_offset:计算零漂
 * @param[in]      gyro:角速度数据
 * @param[out]     offset_time_count: 自动加1
 * @retval         none
 */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
    (*offset_time_count)++;
}

/**
 * @brief          calculate gyro zero drift
 * @param[out]     cali_scale:scale, default 1.0
 * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
 * @param[out]     time_count: time, when call gyro_offset_calc
 * @retval         none
 */
/**
 * @brief          校准陀螺仪
 * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
 * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
 * @retval         none
 */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
    if (*time_count == 0)
    {
        gyro_offset[0] = gyro_cali_offset[0];
        gyro_offset[1] = gyro_cali_offset[1];
        gyro_offset[2] = gyro_cali_offset[2];
    }
    gyro_offset_calc(gyro_offset, INS_gyro, time_count);

    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;
}

/**
 * @brief          get gyro zero drift from flash
 * @param[in]      cali_scale:scale, default 1.0
 * @param[in]      cali_offset:zero drift,
 * @retval         none
 */
/**
 * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂
 * @retval         none
 */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
 * @brief          get the quat
 * @param[in]      none
 * @retval         the point of INS_quat
 */
/**
 * @brief          获取四元数
 * @param[in]      none
 * @retval         INS_quat的指针
 */
const fp32 *get_INS_quat_point(void)
{
    return INS_quat;
}
/**
 * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
 * @param[in]      none
 * @retval         the point of INS_angle
 */
/**
 * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
 * @param[in]      none
 * @retval         INS_angle的指针
 */
const fp32 *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
 * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
 * @param[in]      none
 * @retval         the point of INS_gyro
 */
/**
 * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
 * @param[in]      none
 * @retval         INS_gyro的指针
 */
extern const fp32 *get_gyro_data_point(void)
{
    return INS_gyro;
}
/**
 * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
 * @param[in]      none
 * @retval         the point of INS_accel
 */
/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
 * @param[in]      none
 * @retval         INS_accel的指针
 */
extern const fp32 *get_accel_data_point(void)
{
    return INS_accel;
}
/**
 * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
 * @param[in]      none
 * @retval         the point of INS_mag
 */
/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
 * @param[in]      none
 * @retval         INS_mag的指针
 */
extern const fp32 *get_mag_data_point(void)
{
    return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == DRDY_IST8310_Pin)
    {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {

        // wake up the task
        // 唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // 开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    // 开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }

    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMA2_Stream2_IRQHandler(void)
{

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        // gyro read over
        // 陀螺仪读取完毕
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        // accel read over
        // 加速度计读取完毕
        if (accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        // temperature read over
        // 温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}
