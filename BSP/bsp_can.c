#include "bsp_can.h"
#include "dm_motor.h"
#include "string.h"
#include "math.h"


// FDCAN config
void BSP_FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;//接收所有标准帧
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;//掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0000; //ID
    sFilterConfig.FilterID2 = 0x0000; // Mask=0, 接收所有

    //FDCAN1
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // FDCAN2 is unused. Keep it stopped: an unconnected transceiver can
    // receive noise and generate needless RX interrupts.
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    // FDCAN3 is unused; leave it stopped for the same reason.
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}



// DJI motor current control command
void DJI_Motor_Send_Command(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t tx_data[8];

    TxHeader.Identifier = std_id; 
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    tx_data[0] = (uint8_t)(current1 >> 8);
    tx_data[1] = (uint8_t)(current1 & 0xFF);
    tx_data[2] = (uint8_t)(current2 >> 8);
    tx_data[3] = (uint8_t)(current2 & 0xFF);
    tx_data[4] = (uint8_t)(current3 >> 8);
    tx_data[5] = (uint8_t)(current3 & 0xFF);
    tx_data[6] = (uint8_t)(current4 >> 8);
    tx_data[7] = (uint8_t)(current4 & 0xFF);

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
		{
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
    }
}



// CAN RX interrupt: push frame into queue
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // CAN RX interrupt: push frame into queue
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            // CAN RX interrupt: push frame into queue
            if(rx_header.IdType == FDCAN_STANDARD_ID)
            {
                CAN_Rx_Message_t can_msg;
                can_msg.hfdcan = hfdcan;
                can_msg.can_id = rx_header.Identifier;
                memcpy(can_msg.data, rx_data, 8);
                
                osMessageQueuePut(canRxQueueHandle, &can_msg, 0, 0);
            }
        }
    }
}





// CAN RX decode task
void Can_Decode_Task(void const *argument)
{
    CAN_Rx_Message_t rx_msg;

    for(;;)
    {
        if(osMessageQueueGet(canRxQueueHandle, &rx_msg, NULL, osWaitForever) == osOK)
        {
            if(rx_msg.hfdcan->Instance == FDCAN1)
            {
                // DM motor feedback (Master ID = 0x11~0x17)
                DM_Motor_ParseFeedback(&hfdcan1, rx_msg.can_id, rx_msg.data);
            }
            else if(rx_msg.hfdcan->Instance == FDCAN2) 
            {

            }
             else if(rx_msg.hfdcan->Instance == FDCAN3)
            {

            }
        }
    }
}

