//这是发不出去的那份代码
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

#if 0
    //GPIO_5_CANA_RX
    GPIO_setPinConfig(GPIO_5_CANA_RX);
    GPIO_setPinConfig(GPIO_4_CANA_TX);

#else
    GPIO_setPinConfig(GPIO_33_CANA_RX);//DEVICE_GPIO_CFG_CANRXA
    GPIO_setPinConfig(GPIO_32_CANA_TX);//   DEVICE_GPIO_CFG_CANTXA
#endif


    CAN_initModule(CANA_BASE);
    CAN_initModule(CANB_BASE);


    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);
    CAN_setBitRate(CANB_BASE, DEVICE_SYSCLK_FREQ, 500000, 20);


    CAN_enableInterrupt(CANB_BASE, CAN_INT_IE0 | CAN_INT_ERROR |CAN_INT_STATUS);

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
                               CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);

    //CANA_RX_MSG_OBJ_ID
    CAN_setupMessageObject(CANA_BASE, CANA_RX_MSG_OBJ_ID, CANA_ID,
                               CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0,
                               CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);




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


    for(;;)
    {
        CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID, MSG_DATA_LENGTH,txMsgData);

        //CAN_sendMessage(CANB_BASE, 2, MSG_DATA_LENGTH,txMsgData);
        //while(CanaRegs.CAN_NDAT_21 == 1 && CanaRegs.CAN_TXRQ_21 == 1);
    }


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

    //
    // Read the CAN-B interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(CANA_BASE);

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
        CAN_clearInterruptStatus(CANA_BASE, RX_MSG_OBJ_ID);

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
