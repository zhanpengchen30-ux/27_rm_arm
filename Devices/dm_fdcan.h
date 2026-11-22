#ifndef __DM_FDCAN_H__
#define __DM_FDCAN_H__

#include "main.h"
#include <stdint.h>

/* ============================================================================
 * FDCAN 参数唯一入口。
 * 以后要调数据域速率、位时序、收发器延迟补偿，只改这个文件。
 * DM_CANFD_5M_ENABLE 改成 0 就整体回到 classic CAN 1 Mbps：
 * dm_motor.c / bsp_can.c / arm.c 里的分支会跟着一起切回去。
 * ==========================================================================*/
#define DM_CANFD_5M_ENABLE      1

/* 2026-11-19 已用 DM 调试工具复验：电机在 5M 数据域下能正常接收
   FD 加速使能与 0x307 PVT 帧并持续回帧，主控发送也统一为 FD+BRS。
   如需退回 classic 命令做 A/B，只把本开关改 1；arm.c 会据此自动把
   发帧分频恢复为 2，避免 1M 总线继续按 500Hz 发送。 */
#define DM_CANFD_TX_CLASSIC     0u

/* 仲裁段保持 1 Mbps（fdcan.c 里 120 MHz / (10 × 12tq)），这里不动。
   数据段 5 Mbps：120 MHz / (1 × 24tq)，24 = 1 + 17 + 6，采样点 75%。 */
#define DM_CANFD_DATA_PRESCALER 1u
#define DM_CANFD_DATA_SJW       4u
#define DM_CANFD_DATA_TSEG1     17u
#define DM_CANFD_DATA_TSEG2     6u

/* 5 Mbps 下必须开收发器延迟补偿（TDC）。TDCO 用 ST 高速 BRS 示例的
   基线 DataTimeSeg1 × DataPrescaler = 17 × 1 = 17 mtq；0 只表示测得
   环路延迟、不包含额外偏移，不能作为 5M 的可靠配置。
   若仍有数据段错误，结合 PSR.TDCV / DLEC / TEC 实测调整，勿盲调。 */
#define DM_CANFD_TDC_ENABLE     1
#define DM_CANFD_TDC_OFFSET     17u
#define DM_CANFD_TDC_FILTER     0u

/* 把已在 classic 模式下初始化过的 FDCAN 实例切成 FD + BRS。
   调用时机：HAL_FDCAN_ConfigFilter 之前、HAL_FDCAN_Start 之前。
   内部会 Stop → 改 Init → 重新 Init，所以会清掉已配置的过滤器，
   过滤器必须在它之后再配。 */
void DM_FDCAN_ConfigureFD(FDCAN_HandleTypeDef *hfdcan);

/* 按当前开关填发送帧头的 FD 相关两个字段。 */
void DM_FDCAN_FillTxHeader(FDCAN_TxHeaderTypeDef *header);

#endif
