#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"
#include "tim.h"

#define BUZZER_CONTINUE_TIMES 2
#define BUZZER_VAL 250
#define LOW_BUZZER_VAL 5
typedef enum
{
    ZERO_TIME  = 0,
    ONE_TIME   = 1,
	HEAPS_TIME = 2,
	THREE_TIME = 3
} buzzer_mode_u;
typedef enum
{
	OFF = 0,
	ON = 1
}buzzer_status_e;
typedef struct
{
    buzzer_mode_u mode;
    uint16_t buzzer_value;
    uint16_t continue_times;
    uint16_t times;
	buzzer_status_e  status;
} Buzzer_t;
//extern Buzzer_t Buzzer_Control;

void Buzzer_Processing(void);
void Buzzer_Init(void);

void Set_BuzzerMode(buzzer_mode_u buzzer_mode);
buzzer_mode_u Get_BuzzerMode(void);

#endif
