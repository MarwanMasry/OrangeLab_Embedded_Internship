/******************************************************************************
 *
 * Module: Common - Platform
 *
 * File Name: Std_Types.h
 *
 * Description: General type definitions
 *
 * Author: Marwan Abdelhakim Elmasry
 *
 *******************************************************************************/

#ifndef STD_TYPES_H
#define STD_TYPES_H


/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/
/* Standard Platform types */
#include "Platform_Types.h"

/* Standard Compiler */
#include "Compiler.h"

/*******************************************************************************
 *                             Types Deceleration	                       *
 *******************************************************************************/

/*
 *  Describes the standard Return Type Definitions used in the project
 */
typedef uint8  Std_ReturnType;

/*
 * Structure for the Version of the module.
 * This is requested by calling <Module name>_GetVersionInfo()
 */
typedef struct
{
  uint16  vendorID;
  uint16  moduleID;
  uint8 sw_major_version;
  uint8 sw_minor_version;
  uint8 sw_patch_version;
} Std_VersionInfoType;

/*******************************************************************************
 *                          Pre-Processors Definition                          *
 *******************************************************************************/
#define STD_HIGH        0x01U       /* Standard HIGH */
#define STD_LOW         0x00U       /* Standard LOW */

#define STD_ACTIVE      0x01U       /* Logical state active */
#define STD_IDLE        0x00U       /* Logical state idle */

#define STD_ON          0x01U       /* Standard ON */
#define STD_OFF         0x00U       /* Standard OFF */

#define ZERO            (0U)
#define ONE				(1U)
#define FIVE			(5U)


#define _ENABLE_	STD_HIGH
#define _DISABLE_   STD_LOW

#endif /* STD_TYPES_H */
