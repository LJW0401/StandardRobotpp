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

void LegFKine();
void LegIKine();
void LegJacobian();
void LegSpd();
void LegTransform();
void LegFeedforward();
void L2K();

#endif // COMPONENTS_CONTROLLER_LEG_MODEL_H
