/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Module */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
|| (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif
#endif

/*PORT module initialization status*/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
STATIC const Port_ConfigPin *Port_Pins = NULL_PTR;
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
void Port_Init(const Port_ConfigType *ConfigPtr)
{
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0; /*used to delay cycles when enabling clock for a port*/
    Port_PinType current_pin = 0; /*used as an index to loop on all pins in our configuration structure*/
    boolean error = FALSE; /*used to Skip service if an error is detected*/
            SYSCTL_REGCGC2_REG |= (1 << 5);

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                        PORT_E_PARAM_CONFIG);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
#endif
	Port_Pins = ConfigPtr->pins; /* address of the first Channels structure --> Channels[0] */
    if (FALSE == error) /*(PORT087) Skip service if an error is detected*/
    {

        while (current_pin < PORT_NUMBER_OF_PORT_PINS) /*Loop on all pins and set them*/
        {

            switch (Port_Pins[current_pin].port_num)
            {
            case 0:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
            case 1:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
            case 2:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
            case 3:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
            case 4:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
            case 5:
                PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
            }

            /* Enable clock for PORT and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1 << Port_Pins[current_pin].port_num);
            delay = SYSCTL_REGCGC2_REG;

            if (((Port_Pins[current_pin].port_num == 3) && (Port_Pins[current_pin].pin_num == 7)) || ((Port_Pins[current_pin].port_num == 5) && (Port_Pins[current_pin].pin_num == 0))) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                                     /* Unlock the GPIOCR register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            else if ((Port_Pins[current_pin].port_num == 2) && (Port_Pins[current_pin].pin_num <= 3)) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
            current_pin++;
                continue;
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }

            if (PORT_ENABLE_ANALOG == Port_Pins[current_pin].analog_mode) /*in case of analog mode for pin*/
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */

                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIODEN register to disnable digital functionality on this pin */
            }
            else /*in case of non-analog mode*/
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */

                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[current_pin].pin_num * 4)); /* Clear the PMCx bits for this pin */
                if (Port_Pins[current_pin].mode != PORT_PIN_MODE_GPIO)                                                                                    /* non GPIO pin */
                {

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[current_pin].pin_num);                                                 /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((0x0000000F & Port_Pins[current_pin].mode) << (Port_Pins[current_pin].pin_num * 4)); /* Set the PMCx bits for this pin as it is in its' mode */
                }
                else if (PORT_PIN_MODE_GPIO == Port_Pins[current_pin].mode) /*GPIO Pin*/
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */

                    if (Port_Pins[current_pin].direction == PORT_PIN_OUT) /*GPIO Pin direction out*/
                    {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                        if (Port_Pins[current_pin].initial_value == STD_HIGH) /*Set initial value for output pin*/
                        {
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                        }
                        else
                        {
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                        }
                    }
                    else if (Port_Pins[current_pin].direction == PORT_PIN_IN)
                    {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                        if (Port_Pins[current_pin].resistor == PULL_UP) /*set internal resistors for input pin*/
                        {
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                        }
                        else if (Port_Pins[current_pin].resistor == PULL_DOWN)
                        {
                            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                        }
                        else
                        {
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[current_pin].pin_num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                        }
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                }
                else
                {
                    /* Do Nothing */
                }
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            }

            current_pin++;
        }
        Port_Status = PORT_INITIALIZED;
        Port_Pins = ConfigPtr->pins; /* address of the first pins structure --> pins[0] */
    }
    else
    {
        /*Do nothing*/
    }
}

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
#if (PORT_SET_PIN_DIRECTION_API == STD_ON) /*only if this api is enabled in port_cfg*/
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE; /*used to Skip service if an error is detected*/

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the Pin id is invalid */
    if (Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
    /* check if the Pin id is invalid */
    if ((error == FALSE) && (Port_Pins[Pin].pin_direction_changeable == FALSE)) /*(PORT077) SKIP next parameter chacking if the first is invalid*/
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
    /* check if the Port module is uninitialized */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }

#endif
    if (FALSE == error) /*(PORT087) Skip service if an error is detected*/
    {

        switch (Port_Pins[Pin].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }
        if (Direction == PORT_PIN_OUT) /*GPIO Pin Direction out*/
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        }
        else if (Direction == PORT_PIN_IN)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        }
        else
        {
            /*do nothing*/
        }
    }
}
#endif /* PORT_SET_PIN_DIRECTION_API CHECK*/

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
void Port_RefreshPortDirection(void)
{
    boolean error = FALSE; /*used to Skip service if an error is detected*/

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /*Check if port module is uninitialized*/
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID,
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
#endif
    if (FALSE == error) /*(PORT087) Skip service if an error is detected*/
    {
        volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
        Port_PinType current_pin = 0; /*used as index to loop on all configured pins*/
        while (current_pin < PORT_NUMBER_OF_PORT_PINS) /*Loop on all pins and set them*/
        {
            if (Port_Pins[current_pin].pin_direction_changeable == FALSE)
            {

                switch (Port_Pins[current_pin].port_num)
                {
                case 0:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                    break;
                case 1:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                    break;
                case 2:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                    break;
                case 3:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                    break;
                case 4:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                    break;
                case 5:
                    PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                    break;
                }
                if (Port_Pins[current_pin].direction == PORT_PIN_OUT) /*GPIO Pin Direction out*/
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                }
                else if (Port_Pins[current_pin].direction == PORT_PIN_IN)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[current_pin].pin_num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                }
                else
                {
                    /*do nothing*/
                }
            }
            else
            {
                /*Do nothing*/
            }

            current_pin++; /*increase index counter*/
        }
    }
}

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

#if (PORT_VERSION_INFO_API == STD_ON) /*CHECK IF API IS ENABLED BY USER IN CFG*/
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
    boolean error = FALSE; /*used to Skip service if an error is detected*/
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if (NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
    /*Check if port module is uninitialized*/
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID,
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
#endif                  /* (DIO_DEV_ERROR_DETECT == STD_ON) */
    if (error == FALSE) /*(PORT087) Skip service if an error is detected*/
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif /* PORT_SGT_VERSION_INFO_API CHECK*/

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
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{

    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE; /*used to Skip service if an error is detected*/

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the Pin id is invalid */
    if (Pin >= PORT_NUMBER_OF_PORT_PINS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                        PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
    /* check if the input mode is invalid */
    if ((error == FALSE)&&(Mode > PORT_MODES_MAX)) /*(PORT077) SKIP next parameter chacking if the first is invalid*/
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                        PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
    /* check if the Pin mode is unchangeable */
    if (Port_Pins[Pin].pin_mode_changeable == FALSE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                        PORT_E_MODE_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /*Do nothing*/
    }
#endif
    if (FALSE == error) /*(PORT087) Skip service if an error is detected*/
    {
        switch (Port_Pins[Pin].port_num)
        {
        case 0:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
        case 1:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
        case 2:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
        case 3:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
        case 4:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
        case 5:
            PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
        }
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[Pin].pin_num * 4)); /* Clear the PMCx bits for this pin */

        if (Mode != PORT_PIN_MODE_GPIO) /* non GPIO pin */
        {

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num);                                   /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((0x0000000F & Port_Pins[Pin].mode) << (Port_Pins[Pin].pin_num * 4)); /* Set the PMCx bits for this pin as it is in its' mode */
        }
        else if (PORT_PIN_MODE_GPIO == Mode) /*GPIO Pin*/
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].pin_num); /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        }
        else
        {
            /*Do nothing*/
        }
    }
}
#endif /* PORT_SET_PIN_MODE_API CHECK*/
