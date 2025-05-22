#ifndef _STM_SETTINGS_H_
#define _STM_SETTINGS_H_

#include "main.h"

/**
  * @brief  нумерация STM-сигналов, используемых в данном устройстве
*/
typedef enum stm_list
{
	KPBE, NKBE, AMKO, RZHM,
	NEK5, NEK6, STM_DV1, STM_DV2, 
	STM_DV3, STM_DV4, 
	STM_NUM
} stm_list;

#define STM_CM_IO_NUM 		    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
#define STM_DEFAULT_VAL 	  	{1, 0, 0, 0, 0, 0, 1, 1, 1, 1}

#endif

