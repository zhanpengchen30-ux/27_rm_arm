#include "dm_fdcan.h"

#if DM_CANFD_5M_ENABLE

void DM_FDCAN_ConfigureFD(FDCAN_HandleTypeDef *hfdcan)
{
    if(hfdcan == NULL) return;

    /* 回到 INIT/CCE 才能改位时序；对已停住的句柄是空操作。
       注意 HAL_FDCAN_Stop 只在 State==BUSY 时真正执行，
       所以启动前调用两次也是安全的。 */
    (void)HAL_FDCAN_Stop(hfdcan);

    hfdcan->Init.FrameFormat       = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.DataPrescaler     = DM_CANFD_DATA_PRESCALER;
    hfdcan->Init.DataSyncJumpWidth = DM_CANFD_DATA_SJW;
    hfdcan->Init.DataTimeSeg1      = DM_CANFD_DATA_TSEG1;
    hfdcan->Init.DataTimeSeg2      = DM_CANFD_DATA_TSEG2;

    /* State 已是 READY，MspInit 不会再跑，时钟/GPIO 保持原样 */
    if(HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }

#if DM_CANFD_TDC_ENABLE
    /* 必须在 READY 状态调用：此时外设还在 INIT 里且 CCE=1 */
    (void)HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, DM_CANFD_TDC_OFFSET, DM_CANFD_TDC_FILTER);
    (void)HAL_FDCAN_EnableTxDelayCompensation(hfdcan);
#endif
}

#else

void DM_FDCAN_ConfigureFD(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
}

#endif

void DM_FDCAN_FillTxHeader(FDCAN_TxHeaderTypeDef *header)
{
#if DM_CANFD_5M_ENABLE && !DM_CANFD_TX_CLASSIC
    header->BitRateSwitch = FDCAN_BRS_ON;
    header->FDFormat      = FDCAN_FD_CAN;
#else
    header->BitRateSwitch = FDCAN_BRS_OFF;
    header->FDFormat      = FDCAN_CLASSIC_CAN;
#endif
}
