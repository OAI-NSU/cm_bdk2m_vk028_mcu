/**
  ******************************************************************************
	* @file           	: power_management.c
	* @version        	: v1.0
	* @brief        	: библиотека для работы с питанием ЦМ
	* @author			: Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
	* @date				: 2021.09.05
  ******************************************************************************
  */

#include "power_management.h"

/**
	* @brief  инициализация модуля управления питанием
	* @param  pwr_ptr указатель на структуру управления модулем питания
  */
void pwr_init(typePower* pwr_ptr, typeADCStruct* adc_ptr, type_GPIO_OAI_cm *io_ptr)
{
	uint8_t i = 0;
	float curr_a = 0, curr_b = 0;
	char alias[32] = {0};
	//
	uint8_t type_arr[PWR_CH_NUMBER] = PWR_CHANNELS_TYPES;
	uint8_t auto_control_arr[PWR_CH_NUMBER] = PWR_AUTO_CTRL;
	uint8_t double_out[PWR_CH_NUMBER] = PWR_DOUBLE_OUT;
	//
	float r_sh_arr[PWR_CH_NUMBER] = PWR_CAL_RES_SHUNT_OHM;
	float r_fb_arr[PWR_CH_NUMBER] = PWR_CAL_FB_SHUNT_OHM;
	uint16_t bound_arr[PWR_CH_NUMBER] = PWR_CURRENT_BOUND;
	//
	uint8_t io_cfg[PWR_CH_NUMBER][5] = PWR_GPIO_PORT_CFG;
	uint8_t adc_cfg[PWR_CH_NUMBER][1] = PWR_ADC_CFG;
	// начальное состояние
	uint8_t def_state[PWR_CH_NUMBER] = PWR_DEFAULT_STATE;
	uint8_t def_hs[PWR_CH_NUMBER] = PWR_DEFAULT_HALF_SET;
	uint16_t def_delay[PWR_CH_NUMBER] = PWR_DEFAULT_DELAY;
	//
	pwr_ptr->state = 0x0000;
	pwr_ptr->status = 0x0000;
	pwr_ptr->half_set = 0x0000;
	//
	pwr_ptr->global_busy = 0x00;
	//
	fifo_init(&pwr_ptr->cmd_fifo);
	pwr_ptr->queue_delay_ms = 0;
	pwr_ptr->queue_delay_cnter_ms = 0;
	pwr_ptr->queue_delay_flag = 0;
	//
	pwr_ptr->error_cnter = 0;
	//
	for (i=0; i<PWR_CH_NUMBER; i++){
		__pwr_calc_current_coefficients(r_sh_arr[i], r_fb_arr[i], &curr_a, &curr_b);
		switch(type_arr[i]){
			case PWR_CH_CTRL_NU:
			break;
			case PWR_CH_FLAG:
			break;
			case PWR_CH_PULSE:
			break;
			case PWR_CH_DV_CTRL:
			break;
		}
		sprintf(alias, "pwr ch %d", i);
		pwr_ch_init(&pwr_ptr->ch[i], alias,  type_arr[i], auto_control_arr[i], double_out[i], bound_arr[i], curr_a, curr_b, io_ptr, adc_ptr, io_cfg[i], adc_cfg[i]);
	}
	// задание очереди инициализации каналов питания
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_queue_put_cmd(pwr_ptr, 0, i, 0, PWR_CH_HS_NOT_CHANGED);
	}
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_queue_put_cmd(pwr_ptr, def_delay[i], i, def_state[i], def_hs[i]);
	}
	//
	printf("%s: pwr init finish: ch_num <%d>\n",  now(),PWR_CH_NUMBER);
	//
	pwr_ptr->call_interval_us = 0;
	pwr_ptr->last_call_time_us = 0;
}

// Task planner process handler

/**
  * @brief  функция для запуска в планировщике задач обработки каналов питания
	* @param  ctrl_struct указатель на програмную модель устройства
	* @param  time_ms глобальное время
  */
int8_t pwr_process_tp(void* ctrl_struct, uint64_t time_us, typeProcessInterfaceStruct* interface)
{
	typePower* pwr_ptr = (typePower*)ctrl_struct;
	if ((time_us - pwr_ptr->last_call_time_us) > (PWR_PROCESS_PERIOD*1000)) {
		pwr_ptr->call_interval_us = time_us - pwr_ptr->last_call_time_us;
		pwr_ptr->last_call_time_us = time_us;
		// USER CODE BEGIN
		//
		pwr_synchronize_state_and_status(pwr_ptr);
		pwr_create_report(pwr_ptr);
		//
		pwr_step_process(pwr_ptr, pwr_ptr->call_interval_us/1000);
		// USER CODE END
		return 1;
	}
	else {
		return 0;
	}
}

/**
	* @brief  обработка всех каналов измерения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_step_process(typePower* pwr_ptr, uint32_t interval_ms)
{
	typeCMD queue_cmd;
	volatile uint8_t status;
	// обработка состояний каналов
	for(uint8_t num=0; num<PWR_CH_NUMBER; num++){
		status = pwr_ch_process(&pwr_ptr->ch[num], interval_ms);
		if (status == 0){
			pwr_queue_put_cmd(pwr_ptr, 200, num, 0, PWR_CH_HS_NOT_CHANGED);
		}
		else{
			//
		}
	}
	// проверка занятости блоков
	pwr_ptr->global_busy = 0;
	for(uint8_t num=0; num<PWR_CH_NUMBER; num++){
		pwr_ptr->global_busy |= (pwr_ch_get_busy(&pwr_ptr->ch[num])) << num;
	}
	// проверка таймаута взятия новой команды из fifo
	if (pwr_queue_process(pwr_ptr, interval_ms)){
		if(pwr_ptr->global_busy == 0){
			if(pwr_queue_get_cmd(pwr_ptr, &queue_cmd)){
				//
				if (queue_cmd.field.num < PWR_CH_NUMBER){
					if (queue_cmd.field.half_set != PWR_CH_HS_NOT_CHANGED) pwr_ch_half_set_choosing(&pwr_ptr->ch[queue_cmd.field.num], queue_cmd.field.half_set);
					pwr_ch_on_off(&pwr_ptr->ch[queue_cmd.field.num], queue_cmd.field.state);
				}
				else pwr_ptr->error_cnter += 1;
			}
		}
	}
}

/**
 * @brief синхронизация статусов и событий из каналов в общий управляющий модуль
 * 
 * @param pwr_ptr управляющая структура
 */
void pwr_synchronize_state_and_status(typePower* pwr_ptr)
{
	uint32_t state=0, status=0, half_set=0;
	uint8_t ch;
	for (ch=PWR_BE; ch<PWR_CH_NUMBER; ch++){
		state |= (pwr_ptr->ch[ch].state & 0x01) << ch;
		status |= (pwr_ptr->ch[ch].status & 0x01) << ch;
		half_set |= (pwr_ptr->ch[ch].half_set & 0x01) << ch;
	}
	pwr_ptr->state = state;
	pwr_ptr->status = status;
	pwr_ptr->half_set = half_set;
}

/**
  * @brief  включение/отключение канала питания по номеру программно (с учетом флага автоматического контроля)
	* @param  pwr_ch_ptr указатель на структуру управления
	* @param  num номер канала
	* @param  state  1: on, 0: off
  */
void pwr_on_off_by_num(typePower* pwr_ptr, uint8_t num, uint8_t state)
{
	//pwr_ch_on_off(&pwr_ptr->ch[num], state);
	pwr_queue_put_cmd(pwr_ptr, 200, num, state, PWR_CH_HS_NOT_CHANGED);
}

/**
  * @brief  сброс статуса модуля питания
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_status_reset_by_num(typePower* pwr_ptr, uint8_t num)
{
	pwr_ptr->ch[num].status = 0;
}

/**
  * @brief  установка состояния модулей с одновременным включением, отключением
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_set_state(typePower* pwr_ptr, uint32_t state)
{
	uint8_t i=0;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_on_off_by_num(pwr_ptr, i, (pwr_ptr->state >> i) & 0x01);
	}
}

/**
  * @brief  установка уровня токовой защиты
	* @param  pwr_ptr указатель на структуру управления
	* @param  bound указатель на структуру управления
  */
void pwr_set_bound(typePower* pwr_ptr, uint8_t num, uint16_t bound)
{
	pwr_ptr->ch[num].current_bound_mA = (float)bound;
}

/**
  * @brief  создание отчета для удобства отображения
	* @param  pwr_ptr указатель на структуру управления
  */
void pwr_create_report(typePower* pwr_ptr)
{
	uint8_t i;
	for (i=0; i<PWR_CH_NUMBER; i++){
		pwr_ptr->curr_report_fp[i] = pwr_ptr->ch[i].current_fp_mA;
	}
}

/**
 * @brief обработка очереди
 * 
 * @param pwr_ptr 
 * @param interval_ms 
 * @return uint8_t 1 - можно забирать следующую команду, 0 - ждем
 */
uint8_t pwr_queue_process(typePower* pwr_ptr, uint32_t interval_ms)
{
	if(pwr_ptr->queue_delay_flag){
		pwr_ptr->queue_delay_cnter_ms += interval_ms;
		if(pwr_ptr->queue_delay_cnter_ms >= pwr_ptr->queue_delay_ms){
			pwr_ptr->queue_delay_cnter_ms = 0;
			pwr_ptr->queue_delay_ms = 0;
			pwr_ptr->queue_delay_flag = 0;
		}
		return 0;
	}
	else{
		return 1;
	}
}

/**
 * @brief взятие команды из очереди и запуск ожидания следующего шага
 * 
 * @param pwr_ptr 
 * @param cmd 
 * @return uint8_t 
 */
uint8_t pwr_queue_get_cmd(typePower* pwr_ptr, typeCMD *cmd)
{
	typeCMD q_cmd;
	if(fifo_read(&pwr_ptr->cmd_fifo, &q_cmd)){
		pwr_ptr->queue_delay_cnter_ms = 0;
		pwr_ptr->queue_delay_ms = q_cmd.field.delay_ms;
		pwr_ptr->queue_delay_flag = 1;
		//
		// printf("pwr_queue_get_cmd: ch_num<%d> hs<%d> state<%d>\n", q_cmd.field.num, q_cmd.field.half_set, q_cmd.field.state);
		*cmd = q_cmd;
		return 1;
	}
	else{
		return 0;
	}
}

/**
 * @brief установка команды в очередь
 * 
 * @param pwr_ptr 
 * @param delay_ms  задержка до выполнения следующей команды
 * @param pwr_ch_num 
 * @param cmd 
 * @param half_set 
 */
void pwr_queue_put_cmd(typePower* pwr_ptr, uint16_t delay_ms, uint8_t pwr_ch_num, uint8_t state, uint8_t half_set)
{
	typeCMD q_cmd;
	q_cmd.field.delay_ms = delay_ms;
	q_cmd.field.state = state;
	q_cmd.field.num = pwr_ch_num;
	q_cmd.field.half_set = half_set;
	if (fifo_write(&pwr_ptr->cmd_fifo, &q_cmd)){
		//
	}
	else pwr_ptr->error_cnter += 1;
	//
	// printf("pwr_queue_put_cmd: ch_num<%d> hs<%d> state<%d>\n", q_cmd.field.num, q_cmd.field.half_set, q_cmd.field.state);
}

// static
/**
  * @brief  подсчет коэффициентов перевод и сопротивлений в калибровочные коэффициенты  I(mA)=curr_a*ADC_U(V)+curr_b
	* @param  r_sh сопротивление для измерения тока
	* @param  r_fb сопротивление обратной связи
	* @param  curr_a_ptr указатель на переменную adc_a
	* @param  curr_b_ptr указатель на переменную adc_b
  */
void __pwr_calc_current_coefficients(float r_sh, float r_fb, float* curr_a_ptr, float* curr_b_ptr)
{
	float volt_to_curr_coeff = ((float)(1E6)/(r_sh*r_fb));
	*curr_a_ptr = volt_to_curr_coeff;
	*curr_b_ptr = 0;
}

//

/**
 * @brief Инициализация fifo команд управления
 * 
 * @param fifo_ptr 
 * @return int8_t 
 */
int8_t fifo_init(type_PWR_CMD_Fifo* fifo_ptr)
{
	memset((uint8_t*)fifo_ptr, 0x00, sizeof(type_PWR_CMD_Fifo));
	return 0;
}

/**
 * @brief запись данных в fifo для принятых данных
 * @param cmd_ptr 
 * @param rx_frame_ptr 
 * @return int8_t 1 - ОК, 0 - записано, но с потерей пакета из-за переполнения
 */
int8_t fifo_write(type_PWR_CMD_Fifo* fifo_ptr, typeCMD* cmd)
{
	uint8_t ret_val = 1;
	memcpy((uint8_t*)&fifo_ptr->cmd_array[fifo_ptr->wr_ptr], (uint8_t *)cmd, sizeof(typeCMD));
	fifo_ptr->rec_full++;
	//
	if((++fifo_ptr->wr_ptr) >= PWR_CMD_FIFO_SIZE) fifo_ptr->wr_ptr = 0;
	//
	if (fifo_ptr->wr_ptr == fifo_ptr->rd_ptr){
		if(++fifo_ptr->rd_ptr >= PWR_CMD_FIFO_SIZE) fifo_ptr->rd_ptr = 0;
		fifo_ptr->rec_lost++;
		ret_val = 0;
	}
	//
	fifo_update(fifo_ptr);
	return ret_val;
}

/**
 * @brief чтение данных из fifo для принятых данных
 * 
 * @param cmd_ptr 
 * @param rx_frame_ptr 
 * @return int8_t 1 - ОК, 0 - нет данных
 */
int8_t fifo_read(type_PWR_CMD_Fifo* fifo_ptr, typeCMD* cmd)
{
	if(fifo_ptr->rec_num){
		memcpy((uint8_t *)cmd, (uint8_t*)&fifo_ptr->cmd_array[fifo_ptr->rd_ptr], sizeof(typeCMD));
		if(++fifo_ptr->rd_ptr >= PWR_CMD_FIFO_SIZE) fifo_ptr->rd_ptr = 0;
		fifo_update(fifo_ptr);
		return 1;
	}
	else{
		return 0;
	}
}

/**
 * @brief обновление количества записей в FIFO по состояниям указателя чтения и записи.
 * 
 * @param cmd_ptr 
 */
void fifo_update(type_PWR_CMD_Fifo* fifo_ptr)
{
	fifo_ptr->rec_num = (fifo_ptr->wr_ptr >= fifo_ptr->rd_ptr) 	? (fifo_ptr->wr_ptr - fifo_ptr->rd_ptr) 
																: (PWR_CMD_FIFO_SIZE - (fifo_ptr->rd_ptr - fifo_ptr->wr_ptr));
	fifo_ptr->rec_num_max = (fifo_ptr->rec_num > fifo_ptr->rec_num_max) ? fifo_ptr->rec_num : fifo_ptr->rec_num_max;
}

