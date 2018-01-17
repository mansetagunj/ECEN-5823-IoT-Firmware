/***********************************************************************************************//**
 * \file   pti.h
 * \brief  Functions and data related to PTI
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/
#include "board_features.h"

#if (HAL_PTI_ENABLE == 1) || defined(FEATURE_PTI_SUPPORT)
uint8_t APP_ConfigEnablePti(void);
#endif // HAL_PTI_ENABLE
