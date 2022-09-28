/******************************************************************************
 * File Name: std_types.h
 * Description: Definitions for boolean values and standard data types commonly used
 * Date Created: 14/2/2022
 * Author: Hazem Montasser
 *******************************************************************************/


#ifndef STD_TYPES_H_
#define STD_TYPES_H_
/************ Boolean values ************/
#ifndef TRUE
#define TRUE (1U)
#endif
#ifndef True
#define True (1U)
#endif
#ifndef true
#define true (1U)
#endif

#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef False
#define False (0U)
#endif

#ifndef false
#define false (0U)
#endif

#ifndef LOGIC_HIGH
#define LOGIC_HIGH (1U)
#endif

#ifndef LOGIC_LOW
#define LOGIC_LOW (0U)
#endif

/************ Data types ************/
#define NULL_PTR ((void*)0)

typedef unsigned char 		uint8;			/* 0 .. +255 */
typedef unsigned short 		uint16;			/* 0 .. +65,535*/
typedef unsigned long 		uint32;			/* 0 .. +4,294,967,295*/
typedef unsigned long long 	uint64;			/* 0 .. +18,446,744,073,709,551,615*/
typedef unsigned char 		boolean;		/* 0 .. +255 */

typedef signed char 		sint8;			/* -128 					  .. +127*/
typedef signed short 		sint16;			/* -32,768 					  .. +32,767*/
typedef signed long 		sint32;			/* -2,147,483,648 			  .. +2,147,483,647*/
typedef signed long long 	sint64;			/* -9,223,372,036,854,775,808 .. +9,223,372,036,854,775,807*/

typedef float 				float32;
typedef double 				float64;

#endif /* STD_TYPES_H_ */
