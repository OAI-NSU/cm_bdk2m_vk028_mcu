
/**
 * @brief Настройки количества и типов каналов питания, а также обвязки измерения тока
 * 
 */

#ifndef _POWER_MANAGER_SETTINGS_H_
#define _POWER_MANAGER_SETTINGS_H_


#include "main.h"
#include "pwr_channel.h"

/**
  * @brief  определяем номера каналов, последнее поле - количество каналов
*/
typedef enum PWR_CH
{
  PWR_BE, PWR_CM1, PWR_CM2, PWR_MPP1, 
  PWR_MPP2, PWR_MPP3, PWR_MPP4, PWR_MPP5, 
  PWR_MPP6, PWR_MPP7, PWR_DEP, PWR_DDII,
  PWR_CH_NUMBER
} type_PWR_CH;

/**
  * @brief  распределение каналов в соответствии с enum PWR_CH
  * @note   длина совпадает с PWR_CH_NUMBER
  */
#define PWR_CAL_RES_SHUNT_OHM   {0.25f,    1.0f,    1.0f,     2.0f,     2.0f,    2.0f,     2.0f,     2.0f,     2.0f,      2.0f,     1.0f,    0.25f}
#define PWR_CAL_FB_SHUNT_OHM    {5.11E3f, 10E3f, 10E3f, 5.6E4f, 5.6E4f, 5.6E4f, 5.6E4f, 5.6E4f, 5.6E4f, 5.6E4f, 5.9E3f, 5.11E3f}

#define PWR_CURRENT_BOUND       {3*650, 3*60, 3*60, 3*60, 3*60, 3*60, 3*60, 3*60, 3*60, 3*60, 3*60, 3*290} // граница тока в мА для каждого канала //TODO: уточнить границы тока для каждого канала

#define PWR_CHANNELS_TYPES      {PWR_CH_CTRL_NU, PWR_CH_CTRL_NU, PWR_CH_CTRL_NU, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_FLAG, PWR_CH_PULSE} // типы каналов по способу включения/отключения: NU, FLAG, PULSE, DV_CTRL
#define PWR_AUTO_CTRL           {0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1}  // указываем те каналы, которые мы может отключать или включать автоматически
#define PWR_DOUBLE_OUT          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // указываем те каналы, которые имеют дублированный выход для полукомплектов с холодным резервированием

#define PWR_DEFAULT_STATE       {PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON, PWR_CH_ON}  // изначальное состояние каналов
#define PWR_DEFAULT_HALF_SET    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}   // изначальное состояние каналов
#define PWR_INIT_TIMEOUT_MS     (10000)
#define PWR_DEFAULT_DELAY       {0, 0, 0, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 200}  // задержка на включение каждого следующего устройства при включении питания после PWR_MAIN_PWR_DELAY_MS (мс)

// Конфигурация GPIO - это массив из 5ти GPIO: 
// PWR_CH_CTRL_NU - не используются
// PWR_CH_FLAG - используются 2 GPIO: ena и inh
// PWR_CH_PULSE - используются 2 GPIO: on и off
// PWR_CH_DV_CTRL - используются 5 GPIO: A, B, C, D, E

// Для каждого канала выделяется 5ти элементный массив, в котором используются только те элементы, которые нужны для работы с данным каналом
// GPIO определяются номером в общей структуре управления io CM

#define PWR_GPIO_PORT_CFG {\
                            {OUT_NU, OUT_NU, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, OUT_NU, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, OUT_NU, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 4, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 5, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 6, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 7, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 8, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 9, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 10, OUT_NU, OUT_NU, OUT_NU}, \
                            {OUT_NU, 3, OUT_NU, OUT_NU, OUT_NU}, \
                            {1, 0, OUT_NU, OUT_NU, OUT_NU} \
                          }

#define PWR_ADC_CFG       {\
                            {6},  \
                            {18}, \
                            {2}, \
                            {11}, \
                            {10}, \
                            {9}, \
                            {8}, \
                            {7}, \
                            {5}, \
                            {3},  \
                            {1},  \
                            {0}  \
                          }

#endif
