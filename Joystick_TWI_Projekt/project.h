/******************************************************************************\
*                                                                              *
* File        : Project.h                                                      *
* Project     : Roboter 2006                                                   *
* Author      :                                                                *
* Initial date:                                                                *
* Release rev.:                                                                *
* Copyright   :                                                                *
* Credits     :                                                                *
* License     :                                                                *
* Description : Definitions spanning this project - except radio communication *
*               interface.                                                     *
*                                                                              *
\******************************************************************************/


#ifndef __PROJECT_H__
#define __PROJECT_H__

#define TWI_JOYSTICK_ADDRESS    0xF4

enum
{ /* TWI commands for PC-joystick */
  // normal operation
  readJoyAll,                           /*   0 */
  readJoy1_X,                           /*   1 */
  readJoy1_Y,                           /*   2 */
  readJoy2_X,                           /*   3 */
  readJoy2_Y,                           /*   4 */
  readJoyPBs,                           /*   5 */
  // (re)centering
  setJoy1UpperLeftCorner = 32,          /*  32 */
  setJoy1LowerRightCorner,              /*  33 */
  setJoy1ConversionFactor,              /*  34 */
  setJoy2UpperLeftCorner,               /*  35 */
  setJoy2LowerRightCorner,              /*  36 */
  setJoy2ConversionFactor,              /*  37 */
  // debugging (optional)
  readJoyAllRaw = 128,                  /* 128 */
  readJoyTrimSetting,                   /* 129 */
};

#endif // #ifndef __PROJECT_H__



/******************************************************************************
 *
 * $Id$
 *
 * $Log$
 *
 *****************************************************************************/
