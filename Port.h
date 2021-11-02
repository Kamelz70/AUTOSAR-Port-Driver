/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Kamel
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H
/*vendor id for the port module*/
#define PORT_VENDOR_ID (1000U)

/*port module id*/
#define PORT_MODULE_ID (140U)

/*port instance id*/
#define PORT_INSTANCE_ID (0U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED (1U)
#define PORT_NOT_INITIALIZED (0U)

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

/*
macros for the port initialization STATUS 
*/

#define PORT_INIZTIALIZED (1U)
#define PORT_NOT_INIZTIALIZED (0U)

/*Standard Autosar types*/
#include "Std_Types.h"
/*version check for Std_Types.h*/
#if ((STD_TYPER_AR_RELEASE_MAJOR_VERSION != PORT_PORT_AR_RELEASE_MAJOR_VERSION) || (STD_TYPER_AR_RELEASE_MINOR_VERSION != PORT_PORT_AR_RELEASE_MINOR_VERSION) || (STD_TYPER_AR_RELEASE_PATCH_VERSION != PORT_PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of std_types.h doesn't match the expected version"
#endif

#include "Port_Cfg.h"
/*version check for Port_Cfg.h*/
#if ((STD_TYPER_AR_RELEASE_MAJOR_VERSION != PORT_PORT_AR_RELEASE_MAJOR_VERSION) || (STD_TYPER_AR_RELEASE_MINOR_VERSION != PORT_PORT_AR_RELEASE_MINOR_VERSION) || (STD_TYPER_AR_RELEASE_PATCH_VERSION != PORT_PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of std_types.h doesn't match the expected version"
#endif

/*non-autosar file inclusions*/
#include "Common_Macros.h"
/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS 0x40004000
#define GPIO_PORTB_BASE_ADDRESS 0x40005000
#define GPIO_PORTC_BASE_ADDRESS 0x40006000
#define GPIO_PORTD_BASE_ADDRESS 0x40007000
#define GPIO_PORTE_BASE_ADDRESS 0x40024000
#define GPIO_PORTF_BASE_ADDRESS 0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET 0x3FC
#define PORT_DIR_REG_OFFSET 0x400
#define PORT_ALT_FUNC_REG_OFFSET 0x420
#define PORT_PULL_UP_REG_OFFSET 0x510
#define PORT_PULL_DOWN_REG_OFFSET 0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET 0x51C
#define PORT_LOCK_REG_OFFSET 0x520
#define PORT_COMMIT_REG_OFFSET 0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET 0x528
#define PORT_CTL_REG_OFFSET 0x52C

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT init */
#define PORT_INIT_SID (uint8)0X00

/* Service ID for PORT SET PIN DIRECTION */
#define PORT_SET_PIN_DIRECTION_SID (uint8)0X01

/* Service ID for PORT SET PIN DIRECTION */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0X02

/* Service ID for PORT GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID (uint8)0x03

/* Service ID for PORT SET PIN MODE */
#define PORT_SET_PIN_MODE_SID (uint8)0X04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN 0x0A

/* Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE 0x0B

/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG 0x0C

/* API Port_SetPinMode service called with an invalid mode */
#define PORT_E_PARAM_INVALID_MODE 0x0D

/* API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE 0x0E

/* API service called without module initialization */
#define PORT_E_UNINIT 0x0F

/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER 0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Type definition for Port_PinModeType used by the PORT APIs */
typedef enum
{
	PORT_PIN_MODE_GPIO,
	PORT_PIN_MODE_ADC,
	PORT_PIN_MODE_SPI,
	PORT_PIN_MODE_PWM,
	PORT_PIN_MODE_GPT,
	PORT_PIN_MODE_WDG,
	PORT_PIN_MODE_UART,
	PORT_PIN_MODE_CAN,
	PORT_PIN_MODE_LIN,
	PORT_PIN_MODE_ICU
}Port_PinModeType;

/* Description: Enum to hold PIN direction */
typedef enum 
{
    PORT_PIN_IN,
    PORT_PIN_OUT
} Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,
    PULL_UP,
    PULL_DOWN
} Port_InternalResistorType;


/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct
{
    uint8 port_num;
    uint8 pin_num;
    Port_PinDirectionType direction;
    Port_InternalResistorType resistor;
    Port_PinModeType mode;
    uint8 initial_value;
    uint8 pin_direction_changeable;
    uint8 pin_mode_changeable;
} Port_ConfigPin;

/* Data Structure required for initializing the Port Driver */
typedef struct
{
    Port_ConfigPin pins[PORT_NUMBER_OF_PORT_PINS];
} Port_ConfigType;
/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup each pin as Digital/Analog pin based on passed pointer
*              - Setup each pin direction
*              - Setup the internal resistor for each i/p pin
*              - Setup the mode for each pin
*              - Setup the initial value for each GPIO pin
************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr);

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number
                  Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Set the port pin direction
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)/*only if this api is enabled in port_cfg*/
void Port_SetPinDirection(Port_PinType Pin
                            , Port_PinDirectionType Direction);
#endif


/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to refresh the port direction or all configured pins that don't have the direction changeable attribute
************************************************************************************/
void Port_RefreshPortDirection( void );

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo ->Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to Return the version information of this module.
************************************************************************************/
#if (PORT_GET_VERSION_INFO_API == STD_ON)/*only if this api is enabled in port_cfg*/
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin -> Port Pin ID number
                    ,Mode -> New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): versioninfo ->Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to Set the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)/*only if this api is enabled in port_cfg*/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;


#endif /* PORT_H */
