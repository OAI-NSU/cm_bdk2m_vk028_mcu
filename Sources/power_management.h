#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_


#include "main.h"
#include "power_management_settings.h"
#include "pwr_channel.h"
#include "adc.h"
#include "gpio.h"
#include "cyclogramma.h"
#include "clock.h"

#define PWR_PROCESS_PERIOD      200

#define PWR_CH_HS_NOT_CHANGED   0xFF

#define PWR_CMD_FIFO_SIZE			  (64)  //TODO: проверить размер FIFO

#pragma pack(push, 2)

typedef union{
  struct{
    uint16_t delay_ms;
    uint8_t state;
    uint8_t num;
    uint8_t half_set;
    uint8_t gap;
  }field;
  uint8_t raw[6];
} typeCMD;

#pragma pack(pop)

typedef struct{
	typeCMD cmd_array[PWR_CMD_FIFO_SIZE];
	typeCMD last_cmd;
	uint8_t wr_ptr, rd_ptr;
	uint8_t rec_num, rec_num_max;
	uint32_t rec_full;
	uint32_t rec_lost;
	uint8_t ena;
} type_PWR_CMD_Fifo;


/**
  * @brief  структура управления каналом питания
  */
typedef struct
{
  // структуры управления отдельными каналами
  typePowerCh ch[PWR_CH_NUMBER];
  uint32_t global_busy;
  // отчетные структуры
  uint32_t status;
  uint32_t state;
  uint32_t half_set;
  // отчет по потреблению
  float curr_report_fp[PWR_CH_NUMBER];
  // организация очереди команд
  type_PWR_CMD_Fifo cmd_fifo;
  uint16_t queue_delay_ms;
  uint16_t queue_delay_cnter_ms;
  uint8_t queue_delay_flag;
  uint8_t gap;
  //
  uint32_t error_cnter;
  //
  uint64_t call_interval_us;
  uint64_t last_call_time_us;
}typePower;


//
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr, type_GPIO_OAI_cm *io_ptr);
// task planer function
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
//
void pwr_step_process(typePower* pwr_ptr, uint32_t interval_ms);
void pwr_synchronize_state_and_status(typePower* pwr_ptr);
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state);
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num);
void pwr_set_state(typePower* pwr_ptr, uint32_t state);
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound);
void pwr_create_report(typePower* pwr_ptr);
//
uint8_t pwr_queue_process(typePower* pwr_ptr, uint32_t interval_ms);
uint8_t pwr_queue_get_cmd(typePower* pwr_ptr, typeCMD *cmd);
void pwr_queue_put_cmd(typePower* pwr_ptr, uint16_t delay_ms, uint8_t pwr_ch_num, uint8_t state, uint8_t half_set);
// static
void __pwr_calc_current_coefficients(float r_sh, float r_fb, float* curr_a_ptr, float* curr_b_ptr);
//
int8_t fifo_init(type_PWR_CMD_Fifo* fifo_ptr);
int8_t fifo_write(type_PWR_CMD_Fifo* fifo_ptr, typeCMD* cmd);
int8_t fifo_read(type_PWR_CMD_Fifo* fifo_ptr, typeCMD* cmd);
void fifo_update(type_PWR_CMD_Fifo* fifo_ptr);
#endif
