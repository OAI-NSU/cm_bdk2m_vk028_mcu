#ifndef _CM_H_
#define _CM_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
#include "power_management.h"
#include "internal_bus.h"
#include "timers.h"
#include "frames.h"
#include "frame_mem.h"
#include "task_planner.h"
#include "mko_rt.h"
#include "mko_bc.h"
#include "stm.h"

#define CM_HANDLER_INTERVAL_MS   500

/**
	* @brief  данная нумерация является общей для всей периферии и позволяет в общих списках понять смещение данного устройства 
	* @note  данный список также используется для нумерации источников/обработчиков событий от/к периферии (необходимо следить, что бы количество источников не превышало количество доступных событий)
*/
typedef enum devices_list
{
	CM, 	MPP1, 	MPP2, 	MPP3,
	MPP4, 	MPP5, 	MPP6,   RP1,
	RP2, 	DEP, 	DIR,	BDD1,
	BDD2, 	DDII,
	DEV_NUM
} device_list;

//! Управление разрешением работы ускоренного режима для различных устройств
#define CM_DEVICE_MEAS_MODE_MASK 0x01BFE //! модули, работающие по измерительному интервалу
#define CM_DEVICE_SPEEDY_MODE_MASK 0x01BFE //! Ускоренный режим разрешен для всего, кроме ДИР и ЦМ

enum cm_interval_list
{
	CM_INTERV_SYS, CM_INTERV_MEAS, CM_INTERVAL_SPEED, CM_INTERVAL_DBG,
	CM_INTERV_DIR, CM_INTERV_DDII, CM_1S_INTERVAL,
	CM_INTERV_NUMBER
};
#define DEFAULT_CM_INTERV_VALUES_S					{3600, 600, 2, 30, 120, 720, 1}
#define DEFAULT_CM_DEFAULT_START_TIME_S				{5, 5, 0, 6, 5, 60, 6}

#define CM_EVENT_MEAS_INTERVAL_START       			(1<<0)
#define CM_EVENT_MEAS_INTERVAL_DATA_READY  			(1<<1)
#define CM_EVENT_SYS_INTERVAL_START       			(1<<2)
#define CM_EVENT_SYS_INTERVAL_DATA_READY  			(1<<3)
#define CM_EVENT_SPEEDY_INTERVAL_START       		(1<<4)
#define CM_EVENT_SPEEDY_INTERVAL_DATA_READY  		(1<<5)

#define CM_FRAMES_FIFO_DEPTH 						8
#define CM_CFG_FRAME_TYPE							15

//MKO sub_address that are used
#define CM_MKO_SA_SYS								(CM+1)
#define CM_MKO_SA_CMD								17
#define CM_MKO_SA_ARCH_REQUEST_CM					18
#define CM_MKO_SA_ARCH_REQUEST_BDD					19
#define CM_MKO_SA_ARCH_READ_CM						20
#define CM_MKO_SA_ARCH_READ_BDD						21
#define CM_MKO_SA_TECH_RD							29
#define CM_MKO_SA_TECH_CMD							30

// debug interface base don IB
#define CM_SELF_MB_ID  (1)  // для отладочных команд

// Half-set GPIO setting
#define HALF_SET_IO		(OUT_17)  //TODO: уточнить номер вывода
 
/**
  * @brief  список отладочных команд через ВШ
*/
enum cm_dbg_cmd_list
{
	CM_DBG_CMD_SWITCH_ON_OFF, CM_DBG_CMD_CM_RESET, CM_DBG_CMD_CM_CHECK_MEM, CM_DBG_CMD_CM_INIT,
	CM_DBG_CMD_ARCH_REQUEST,
	CM_DBG_CMD_NUMBER
};

/**
  * @brief  список команд МКО
*/
enum mko_cmd_list
{
	CMD_TEST, CMD_SYNCH_TIME, CMD_INIT, CMD_SET_INTERVAL,
	CMD_SET_READ_PTR, CMD_JUMP_TO_DEFENCE_AREA, CMD_SET_MPP_OFFSET, CMD_CONST_MODE,
	CMD_CURRENT_LVL, CMD_PWR_CH_CTRL, CMD_SPEEDY_MODE, CMD_CM_RESET,
	CMD_NUMBER
};

/**
  * @brief  список команд технологического подадреса МКО
*/
enum mko_tech_cmd_list
{
	TCMD_CHECK_MIRROR, TCMD_CHECK_MEM, TCMD_ANY_FRAME_READ, TCMD_SET_OPERATION_TIME,
	TCMD_SET_STM, TCMD_SET_IB, TCMD_SET_MKO_BC, TCMD_WD_MCU_RESET,
	TCMD_NUMBER
};

// статусы ЦМ
#define CM_STATUS_WORK 					(1<<0)
#define CM_STATUS_CFG_HALF_SET	 		(1<<1)
#define CM_STATUS_FULL_MEM_READ	 		(1<<2)
#define CM_STATUS_CFG_LOADED	 		(1<<3)
#define CM_STATUS_FULL			 		(0xFFFF)  //! переменная для полной установки/очистки статуса

// Настройки каналов МПП
#define MPP_DEV_NUM (8)
// !!настройки уставкии МПП (offset) количество должно совпадать с MPP_DEV_NUM!!
#define MPP_DEFAULT_OFFSET {0xF02, 0xF03, 0xF04, 0xF05, 0xF06, 0xF07, 0xF08, 0xF09}
// адреса МПП на внутренней шине
#define MPP_ID {4, 5, 6, 7, 8, 9, 3, 3}
// номер канала, используемый устройством МПП
#define MPP_CHANNENUM_ID {0, 0, 0, 0, 0, 0, 0, 1}

//структуры кадров
#pragma pack(push, 2)
/** 
  * @brief  структура с данными для кадра ЦМ (!! длина обрежется по 52 байта)
  * @note  соседние байтовые поля в МКО меняются местами внутри одного слова
  */
typedef  struct
{
	// питание
	uint16_t currents[PWR_CH_NUMBER];   //+0
	uint16_t power_status;				//+24
	uint16_t power_state;				//+26
	//
	uint8_t nans_counter;  				//+28
	uint8_t nans_status;  				//+29
	uint8_t rst_cnter;  				//+30
	uint8_t cm_status;		  			//+31
	//
	uint32_t operation_time;  			//+32
	//
	int16_t diff_time;					//+36
	uint8_t diff_time_fractional;		//+38
	uint8_t sync_num;					//+39
	uint32_t sync_time_s;				//+40
	//
	int8_t mcu_temp;					//+44
	int8_t stm_val;						//+45
	//
	uint16_t read_ptr;					//+46
	uint16_t write_ptr; 				//+48
	//
	uint16_t sw_version;				//+50
}typeCMFrameReport;	//! 52 - общая длина

/**
 * @brief объединение для связки уровней кадров и полезных данных
 */
typedef union{
	typeFrameStruct row;
	struct{
		uint16_t header[5];
		typeCMFrameReport body;
		uint16_t crc16;
	} sys;
}typeSysFrameUnion;

/** 
  * @brief  структура с конфигурацией ЦМ
  */
typedef  struct
{
	// питание
	uint16_t power_status;		//+0
	uint16_t power_state;		//+2
	//
	uint8_t rst_cnter;  		//+4
	uint8_t gup;				//+5
	//
	uint32_t operation_time;  	//+6
	//
	uint32_t write_ptr;			//+10
	uint32_t read_ptr;			//+14
	//
	uint8_t reserve[34];  		//+18
}typeCfgReport;      //52

typedef union{
	typeFrameStruct row;
	struct{
		uint16_t header[5];
		typeCfgReport body;
		uint16_t crc16;
	} cfg;
}typeCfgFrameUnion;

/** 
  * @brief  структура с переменными управления ЦМ
  */
typedef  struct
{
	uint16_t speedy_mode_state;
	uint16_t speedy_mode_mask;
	uint64_t speedy_mode_timeout;
	//
	uint64_t intervals_us[CM_INTERV_NUMBER];
	uint64_t last_call_interval_times[CM_INTERV_NUMBER];
	uint64_t first_call_time[CM_INTERV_NUMBER];
	uint8_t first_call_status[CM_INTERV_NUMBER];
	//
	uint8_t meas_event, speedy_event;
	//
	uint32_t frame_end_to_end_number;
	//
	uint32_t sync_time_s;
	uint16_t rst_cnter, sync_num;
	int16_t diff_time;
	int8_t diff_time_fractional;
	//
	uint8_t status;  //! status работы ЦМ
	//
	uint32_t operation_time;
}typeCMControlStruct;

/**
  * @brief  общая структура программной модели ЦМ
  */
typedef struct
{
	//
	uint16_t id;			          	//! id на внутренней шине
	uint16_t self_num;          		//! номер устройства с точки зрения ЦМ
	uint16_t half_set_num;          	//! номер полукомплекта (актуально для приборов в с холодным резервированием)
	uint16_t device_number, frame_type; //! параметры прибора, в котором он используется
	//
	typeMKORTStruct* mko_rt_ptr;
	typeMKOBCStruct* mko_bc_ptr;
	typeIBStruct *ib_ptr;
	typePower *pwr_ptr;
	typeFRAME_MEM mem;
	type_SINGLE_GPIO *hs_io_ptr;
	type_STM_Model *stm_ptr;
	//
	uint16_t sw_version;
	//
	typeSysFrameUnion frame;  //!системный кадр
	typeCfgFrameUnion current_cfg, loaded_cfg;  //! кадры с параметрами для сохранения
	uint8_t frame_data_ready;
	//
	typeFrameStruct frames_fifo[CM_FRAMES_FIFO_DEPTH];
	uint8_t frames_fifo_num, frames_fifo_num_max;
	uint32_t fifo_error_cnt;
	uint32_t global_frame_num;  //сквозной номер для формируемых кадров
	uint8_t const_mode;
	uint8_t ddii_pwr_on_flag;
	//
	typeCMControlStruct ctrl;
	//
	uint64_t last_call_time_us, call_interval_us;
	//
}typeCMModel;

#pragma pack(pop)
//
void cm_init(
				typeCMModel* cm_ptr, 
				uint8_t self_num, 
				uint8_t id, 
				typeMKORTStruct *mko_rt_ptr, 
				typeMKOBCStruct *mko_bc_ptr,
				typeIBStruct *ib_ptr, 
				typePower *pwr_ptr,
				type_SINGLE_GPIO *hs_io_ptr,
				type_STM_Model *stm_ptr,
				uint16_t device_number, 
				char* ver_str, 
				uint16_t frame_type
			);
void cm_reset_parameters(typeCMModel* cm_ptr);
uint8_t cm_load_cfg(typeCMModel* cm_ptr);
void cm_save_cfg(typeCMModel* cm_ptr);
void cm_set_cfg(typeCMModel* cm_ptr);
void cm_get_cfg(typeCMModel* cm_ptr);
int8_t cm_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface);
int8_t cm_frame_receive(typeCMModel* cm_ptr, uint8_t* data);
void cm_frame_handling(typeCMModel* cm_ptr);
int8_t cm_write_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame);
int8_t cm_read_fifo(typeCMModel* cm_ptr, typeFrameStruct* frame);
int8_t cm_interval_processor(typeCMModel* cm_ptr, uint8_t interval_id, uint64_t time_us);
// Управление настройками работы ЦМ
void cm_set_interval_value(typeCMModel* cm_ptr, uint16_t interval_number, uint16_t interval_value_s);
void cm_set_speedy_mode(typeCMModel* cm_ptr, uint16_t speedy_mask, uint16_t time_s);
//работа с системном кадром
void cm_frame_forming(typeCMModel* cm_ptr);
// Отладка через ВШ
__weak void cm_dbg_ib_command_handler(typeCMModel* cm_ptr);
// обработка командных сообщений МКО
__weak void cm_mko_command_interface_handler(typeCMModel *cm_ptr);
void cm_mko_cmd_synch_time(typeCMModel* cm_ptr, uint16_t h_time, uint16_t l_time);
void cm_constant_mode_ena(typeCMModel* cm_ptr, uint8_t mode);
uint8_t cm_set_clear_status(typeCMModel* cm_ptr, uint8_t status, uint8_t set_clear);
// Внутренние рабочие функции
void _buff_rev16(uint16_t *buff, uint8_t leng_16);
uint8_t uint16_to_log2_uint8_t(uint16_t var);
uint16_t get_val_from_bound(uint16_t val, uint16_t min, uint16_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница
uint16_t check_val_in_bound(uint16_t val, uint16_t min, uint16_t max);
uint16_t get_version_from_str(char* var_str);

#endif
