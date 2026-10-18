#include "ws2812.h"
#include "spi.h"

#define WS2812_0_CODE   (0xC0 >> 1)  // 0码 (短高长低)
#define WS2812_1_CODE   (0xF0 >> 1)  // 1码(长高短低)

#define RESET_BUF_LEN   50

#define WS2812_SPI_HANDLE  (&hspi6)

void WS2812_Set_Colour(uint8_t r, uint8_t g, uint8_t b)
{
    //24位RGB颜色需要24个字节来模拟
    uint8_t tx_buf[24];
    
    //复位码缓冲区
    uint8_t reset_buf[RESET_BUF_LEN] = {0};

    //将RGB值翻译成SPI要发送的“魔法数字”
    for(int i = 0; i < 8; i++)
    {
        tx_buf[i]     = ((g >> (7 - i)) & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
        tx_buf[i + 8] = ((r >> (7 - i)) & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
        tx_buf[i + 16]= ((b >> (7 - i)) & 0x01) ? WS2812_1_CODE : WS2812_0_CODE;
    }

    //发送颜色数据
    HAL_SPI_Transmit(WS2812_SPI_HANDLE, tx_buf, 24, 100);
    
    //发送复位码
    HAL_SPI_Transmit(WS2812_SPI_HANDLE, reset_buf, RESET_BUF_LEN, 100);
}
