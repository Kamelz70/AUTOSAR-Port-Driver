/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Kamel
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/* 
* Module version 1.0.0                             
*/
#define PORT_SW_MAJOR_VERSION (1U)
#define PORT_SW_MINOR_VERSION (0U)
#define PORT_SW_PATCH_VERSION (0U)

/* 
* Module version 4.0.3                             
*/
#define PORT_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_AR_RELEASE_PATCH_VERSION (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API (STD_ON)

/* Pre-compile option for the Port_SetPinDirection() API */
#define PORT_SET_PIN_DIRECTION_API (STD_ON)

/* Pre-compile option for the Port_SetPinMode() API */
#define PORT_SET_PIN_MODE_API (STD_ON)

/* Number of the configured port PINS */
#define PORT_NUMBER_OF_PORT_PINS (43U)

/* The maximum number for setting a mode*/
#define PORT_MODES_MAX (14)


/*
*       Symbolic pin names
*/
/*Port numbers*/
#define PORT_A_NUM ((uint8)0)
#define PORT_B_NUM ((uint8)1)
#define PORT_C_NUM ((uint8)2)
#define PORT_D_NUM ((uint8)3)
#define PORT_E_NUM ((uint8)4)
#define PORT_F_NUM ((uint8)5)

/*Pin numbers*/
#define PORT_PIN_0_NUM ((uint8)0)
#define PORT_PIN_1_NUM ((uint8)1)
#define PORT_PIN_2_NUM ((uint8)2)
#define PORT_PIN_3_NUM ((uint8)3)
#define PORT_PIN_4_NUM ((uint8)4)
#define PORT_PIN_5_NUM ((uint8)5)
#define PORT_PIN_6_NUM ((uint8)6)
#define PORT_PIN_7_NUM ((uint8)7)

/*Port A pin IDs*/
#define PORT_A_PIN_0_ID ((Port_PinType)0)
#define PORT_A_PIN_1_ID ((Port_PinType)1)
#define PORT_A_PIN_2_ID ((Port_PinType)2)
#define PORT_A_PIN_3_ID ((Port_PinType)3)
#define PORT_A_PIN_4_ID ((Port_PinType)4)
#define PORT_A_PIN_5_ID ((Port_PinType)5)
#define PORT_A_PIN_6_ID ((Port_PinType)6)
#define PORT_A_PIN_7_ID ((Port_PinType)7)

/*Port B pin IDs*/
#define PORT_B_PIN_0_ID ((Port_PinType)8)
#define PORT_B_PIN_1_ID ((Port_PinType)9)
#define PORT_B_PIN_2_ID ((Port_PinType)10)
#define PORT_B_PIN_3_ID ((Port_PinType)11)
#define PORT_B_PIN_4_ID ((Port_PinType)12)
#define PORT_B_PIN_5_ID ((Port_PinType)13)
#define PORT_B_PIN_6_ID ((Port_PinType)14)
#define PORT_B_PIN_7_ID ((Port_PinType)15)

/*Port C pin IDs*/
#define PORT_C_PIN_0_ID ((Port_PinType)16)
#define PORT_C_PIN_1_ID ((Port_PinType)17)
#define PORT_C_PIN_2_ID ((Port_PinType)18)
#define PORT_C_PIN_3_ID ((Port_PinType)19)
#define PORT_C_PIN_4_ID ((Port_PinType)20)
#define PORT_C_PIN_5_ID ((Port_PinType)21)
#define PORT_C_PIN_6_ID ((Port_PinType)22)
#define PORT_C_PIN_7_ID ((Port_PinType)23)

/*Port D pin IDs*/
#define PORT_D_PIN_0_ID ((Port_PinType)24)
#define PORT_D_PIN_1_ID ((Port_PinType)25)
#define PORT_D_PIN_2_ID ((Port_PinType)26)
#define PORT_D_PIN_3_ID ((Port_PinType)27)
#define PORT_D_PIN_4_ID ((Port_PinType)28)
#define PORT_D_PIN_5_ID ((Port_PinType)29)
#define PORT_D_PIN_6_ID ((Port_PinType)30)
#define PORT_D_PIN_7_ID ((Port_PinType)31)

/*Port E pin IDs*/
#define PORT_E_PIN_0_ID ((Port_PinType)32)
#define PORT_E_PIN_1_ID ((Port_PinType)33)
#define PORT_E_PIN_2_ID ((Port_PinType)34)
#define PORT_E_PIN_3_ID ((Port_PinType)35)
#define PORT_E_PIN_4_ID ((Port_PinType)36)
#define PORT_E_PIN_5_ID ((Port_PinType)37)

/*Port F pin IDs*/
#define PORT_F_PIN_0_ID ((Port_PinType)38)
#define PORT_F_PIN_1_ID ((Port_PinType)39)
#define PORT_F_PIN_2_ID ((Port_PinType)40)
#define PORT_F_PIN_3_ID ((Port_PinType)41)
#define PORT_F_PIN_4_ID ((Port_PinType)42)

#endif /*PORT_CFG_H*/