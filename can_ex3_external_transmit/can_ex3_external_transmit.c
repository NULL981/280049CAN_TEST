//#############################################################################
//
// FILE:   can_ex3_external_transmit.c
//
// TITLE:   CAN External Transmit Example
//
//! \addtogroup driver_example_list
//! <h1> CAN-A to CAN-B External Transmit </h1>
//!
//! This example initializes CAN module A and CAN module B for external
//! communication. CAN-A module is setup to transmit incrementing data for "n"
//! number of times to the CAN-B module, where "n" is the value of TXCOUNT.
//! CAN-B module is setup to trigger an interrupt service routine (ISR) when
//! data is received. An error flag will be set if the transmitted data doesn't
//! match the received data. 
//! 
//! \note Both CAN modules on the device need to be
//!       connected to each other via CAN transceivers.
//!
//! \b Hardware \b Required \n
//!  - A C2000 board with two CAN transceivers
//!
//! \b External \b Connections \n
//!  - ControlCARD CANA is on GPIO31 (CANTXA) and GPIO30 (CANRXA)
//!  - ControlCARD CANB is on GPIO8 (CANTXB) and GPIO10 (CANRXB)
//!  - Launchpad CANA is on GPIO32 (CANTXA) and GPIO33 (CANRXA)
//!  - Launchpad CANB is on GPIO8 (CANTXB) and GPIO10 (CANRXB)
//!
//! \b Watch \b Variables \n
//!  - TXCOUNT - Adjust to set the number of messages to be transmitted
//!  - txMsgCount - A counter for the number of messages sent
//!  - rxMsgCount - A counter for the number of messages received
//!  - txMsgData - An array with the data being sent
//!  - rxMsgData - An array with the data that was received
//!  - errorFlag - A flag that indicates an error has occurred
//!
//
//#############################################################################
// $TI Release: F28004x Support Library v1.12.00.00 $
// $Release Date: Fri Feb 12 18:57:27 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Defines
//
#define TXCOUNT  100
#define MSG_DATA_LENGTH    4
#define TX_MSG_OBJ_ID    1
#define RX_MSG_OBJ_ID    1
#define CANA_RX_MSG_OBJ_ID 3
#define CANA_ID 0x66
//
// Globals
//
volatile unsigned long i;
volatile uint32_t txMsgCount = 0;
volatile uint32_t rxMsgCount = 0;
volatile uint32_t errorFlag = 0;
uint16_t txMsgData[4];
uint16_t rxMsgData[4];

//
// Function Prototypes
//
__interrupt void canbISR(void);
__interrupt void canaISR(void);
//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Initialize GPIO and configure GPIO pins for CANTX/CANRX
    // on module A and B
    //
    Device_initGPIO();
    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXA);
    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXA);
    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANRXB);
    //GPIO_setPinConfig(DEVICE_GPIO_CFG_CANTXB);


    //GPIO_5_CANA_RX
    GPIO_setPinConfig(GPIO_5_CANA_RX);
    GPIO_setPinConfig(GPIO_4_CANA_TX);

    //GPIO_setPinConfig(GPIO_33_CANA_RX);
    //GPIO_setPinConfig(GPIO_32_CANA_TX);

    CAN_initModule(CANA_BASE);
    //CAN_initModule(CANB_BASE);

    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);

    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);
    //CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);


    //CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR |CAN_INT_STATUS);

    CAN_enableInterrupt(CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR |CAN_INT_STATUS);


    Interrupt_initModule();


    Interrupt_initVectorTable();


    EINT;
    ERTM;


    Interrupt_register(INT_CANB0, &canbISR);
    Interrupt_register(INT_CANA0, &canaISR);


    Interrupt_enable(INT_CANB0);
    CAN_enableGlobalInterrupt(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    Interrupt_enable(INT_CANA0);
    CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);




    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID, 0x95555555,
                               CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0,
                               CAN_MSG_OBJ_TX_INT_ENABLE, MSG_DATA_LENGTH);

    //CANA_RX_MSG_OBJ_ID
    CAN_setupMessageObject(CANA_BASE, CANA_RX_MSG_OBJ_ID, CANA_ID,
                               CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                               CAN_MSG_OBJ_TX_INT_ENABLE | CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);




    CAN_setupMessageObject(CANB_BASE, RX_MSG_OBJ_ID, 0x95555555,
                           CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0,
                           CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);

    CAN_setupMessageObject(CANB_BASE, 2, 0x95555555,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                               CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);



    txMsgData[0] = 0x12;
    txMsgData[1] = 0x34;
    txMsgData[2] = 0x56;
    txMsgData[3] = 0x78;


    CAN_startModule(CANA_BASE);
    CAN_startModule(CANB_BASE);


    for(i=0;i<10000;i++)
    {
        CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH,txMsgData);
        txMsgData[0]++;

        //CAN_sendMessage(CANB_BASE, 2, MSG_DATA_LENGTH,txMsgData);
        while(CAN_getNewDataFlags(CANA_BASE) == 1 && CAN_getTxRequests(CANA_BASE) == 1);
    }
    while(1);


}

//
// CAN B ISR - The interrupt service routine called when a CAN interrupt is
//             triggered on CAN module B.
//
__interrupt void
canbISR(void)
{
    uint32_t status;

    //
    // Read the CAN-B interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANB_BASE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANB_BASE);

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            errorFlag = 1;
        }
    }
    //
    // Check if the cause is the CAN-B receive message object 1
    //
    else if(status == RX_MSG_OBJ_ID)
    {
        //
        // Get the received message
        //
        CAN_readMessage(CANB_BASE, RX_MSG_OBJ_ID, rxMsgData);

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(CANB_BASE, RX_MSG_OBJ_ID);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANB_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


__interrupt void
canaISR(void)
{
    uint32_t status;
    static int led = 1;
    //
    // Read the CAN-B interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANA_BASE);
    GPIO_writePin(23,led);
    led = 1-led;
    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(CANA_BASE);

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            errorFlag = 1;
        }
    }
    //
    // Check if the cause is the CAN-B receive message object 1
    //
    else if(status == CANA_RX_MSG_OBJ_ID)
    {
        //
        // Get the received message
        //
        CAN_readMessage(CANA_BASE, CANA_RX_MSG_OBJ_ID, rxMsgData);

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        //CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);  //问题1
        CAN_clearInterruptStatus(CANA_BASE, CANA_RX_MSG_OBJ_ID);

        //
        // Increment a counter to keep track of how many messages have been
        // received. In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        rxMsgCount++;

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//
// End of File
//
