/**
 * @file CRC.h
 * @author Miraggio (w1159904119@gmail)
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __DRIVER_CRC_H 
#define __DRIVER_CRC_H 

#include <stdint.h>
#include <stddef.h>
extern    uint16_t check;
/*CRC8ะฃั้*/
unsigned char Checksum_CRC8(uint8_t *buf,uint16_t len);


uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);



uint16_t crc16_ccitt_false(const uint8_t* data, size_t size);


#endif	// __DRIVER_CRC_H
/*-----------------------------------file of end------------------------------*/


