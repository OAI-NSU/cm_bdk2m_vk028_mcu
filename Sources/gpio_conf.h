/**
 * @file gpio_conf.h
 * @author Alexey Styuf
 * @brief настройки для платы OAI_CM
 * @version 0.1
 * @date 2024-02-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef _GPIO_CONF_H_
#define _GPIO_CONF_H_

#include "main.h"

#define GPIO_ON 1
#define GPIO_OFF 0

#define IN 0
#define OUT 1
#define OPEN_DRAIN 2

#define GPIO_IRQ_ON 1
#define GPIO_IRQ_OFF 0


/**
  * @brief  раскрашивание используемых каналов для Out (согласно именованию в схеме)
  * OUT_NU псевдоканал, которые ничего не делает. Нужен как заглушка при не использовании некоторого функционала
*/
enum output_enum
{
	OUT_0, OUT_1, OUT_2, OUT_3,
  OUT_4, OUT_5, OUT_6, OUT_7,
  OUT_8, OUT_9, OUT_10, OUT_11, 
  OUT_12, OUT_13, OUT_14, OUT_15,
  OUT_16, OUT_17, OUT_18, OUT_19,
  OUT_20, OUT_21, OUT_NU,
  OUT_NUM
};

/**
 * @brief раскрашивание ног по функциональному назначению для БДК2М
 * 
 */
enum output_func_enum
{
	OUT_DDII_OFF, OUT_DDII_ON, OUT_BE_OFF, OUT_DEP_OFF,
  OUT_MPP1_OFF, OUT_MPP2_OFF, OUT_MPP3_OFF, OUT_MPP4_OFF, 
  OUT_MPP5_OFF, OUT_MPP6_OFF, OUT_MPP7_OFF,
};


/**
  * @brief  раскрашивание используемых каналов для СТМ
*/
enum stm_enum
{
	STM_0, STM_1, STM_2, STM_3, 
	STM_EXT_0, STM_EXT_1, STM_EXT_2, STM_EXT_3,
  STM_EXT_4, STM_EXT_5, STM_EXT_6, STM_EXT_7,
  STM_EXT_8, STM_EXT_9, STM_EXT_10, STM_EXT_11,
  STM_EXT_12, STM_EXT_13, STM_EXT_14, STM_EXT_15,
  STM_EXT_16, STM_EXT_17, STM_EXT_18, STM_EXT_19,
  STM_EXT_20, STM_EXT_21, STM_EXT_22, STM_EXT_23,
  STM_EXT_24, STM_EXT_25, STM_EXT_26, STM_EXT_27,
  STM_IO_NUM
};

/**
 * @brief раскрашивание ног по функциональному назначению для БДК2М
 * 
 */
enum stm_func_enum
{
  STM_KPBE, STM_NKBE, STM_AMKO
};

// специальная переменная, при передачи в функции управления GPIO делает ничего

// сопоставление имени порта со сквозным номером универсального IO
#define GPIO_PORT { GPIOL, GPIOL, GPIOL, GPIOL, \
                    GPIOL, GPIOL, GPIOL, GPIOL, \
                    GPIOL, GPIOL, GPIOL, GPIOM, \
                    GPIOM, GPIOM, GPIOM, GPIOM, \
                    GPIOM, GPIOM, GPIOM, GPIOM, \
                    GPIOM, GPIOM, NULL \
                  }

// сопоставление сквозного номера вывода с номером линии внутри порта
#define GPIO_LINE { 0, 1, 2, 3,   \
                    4, 5, 6, 7,   \
                    8, 9, 10, 0,  \
                    1, 2, 3, 4,   \
                    5, 6, 7, 8,   \
                    9, 10, 0      \
                  }     

// настройки конфигурации управляющих сигналов для IO: OUT - выход, IN - вход
#define GPIO_IN_OUT { OUT, OUT, OUT, OPEN_DRAIN, \
                      OPEN_DRAIN, OPEN_DRAIN, OPEN_DRAIN, OPEN_DRAIN, \
                      OPEN_DRAIN, OPEN_DRAIN, OPEN_DRAIN, IN,  \
                      IN, IN, IN, IN,     \
                      IN, IN, IN, IN,     \
                      IN, IN, IN          \
                    }

// Настройка применения прерываний по входным сигналам: 0 - не используется, 1 - используется
#define GPIO_IRG_SET  { 0, 0, 0, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0 \
                      }

// настройки начального состояния выводов
#define GPIO_DEF_STATE {0, 0, 0, 1,\
                        1, 1, 1, 1,\
                        1, 1, 1, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0, 0,\
                        0, 0, 0 \
                      }

// настройки конфигурации управляющих сигналов для СТМ
#define STM_PORT {  GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB,\
                    GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC, GPIOC}
#define STM_LINE {  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,\
                    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}
#define STM_IN_OUT {  OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT,\
                      OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT, OUT}
#define STM_IRG_SET {   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define STM_DEF_STATE {  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#endif
