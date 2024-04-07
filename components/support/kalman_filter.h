/**
  ****************************(C) COPYRIGHT 2024 POLARBEAR****************************
  * @file       kalman_filter.c/h
  * @brief      卡尔曼滤波器
  * @note
  * @history
  *  Version    Date         Author        Modification
  *  V1.0.0     2023-11-15   Penguin       1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 POLARBEAR****************************
  */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct
{
    float LastP; // 上次估算协方差
    float out;   // 卡尔曼滤波器输出
    float Q;     // 过程噪声协方差
    float R;     // 观测噪声协方差
} EKF_s;         // Kalman Filter parameter

typedef struct
{
    float LastP; // 上次估算协方差
    float Now_P; // 当前估算协方差
    float out;   // 卡尔曼滤波器输出
    float Kg;    // 卡尔曼增益
    float Q;     // 过程噪声协方差
    float R;     // 观测噪声协方差
} KF_s;          // Kalman Filter parameter

void InitKF(KF_s *kf, float Q, float R);

void InitEKF(EKF_s *ekf, float Q, float R);

float KalmanFilterCalc(KF_s *kfp, float input);

float ExtendedKalmanFilterCalc(EKF_s *ekf, float input1, float input2);

#endif // KALMAN_FILTER_H