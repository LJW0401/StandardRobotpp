/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       leg_model.c/h
  * @brief      轮腿的腿部模型（本部分函数大部分由matlab导出，基本不具备可读性）
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-7-2024     Penguin          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef COMPONENTS_CONTROLLER_LEG_MODEL_H
#define COMPONENTS_CONTROLLER_LEG_MODEL_H

void LegFKine(double phi1, double phi4, double leg_pos[2]);
void LegIKine(double l0, double phi0, double joint_pos[2]);
void LegSpeed(double dphi1, double dphi4, double phi1, double phi4, double speed[2]);
void LegTransform(double F, double Tp, double phi1, double phi4, double T[2]);
double LegFeedforward(double theta);
void L2K(double L0, double K[12]);

#endif // COMPONENTS_CONTROLLER_LEG_MODEL_H
