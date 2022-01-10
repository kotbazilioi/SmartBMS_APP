/*
 * bmsapp.h
 *
 *  Created on: 7 џэт. 2022 у.
 *      Author: ivanov
 */

#ifndef MAIN_BMSAPP_H_
#define MAIN_BMSAPP_H_



#endif /* MAIN_BMSAPP_H_ */

#define CELL_LENGTH 120
#define BUFFER_LENGTH 2*CELL_LENGTH+16
#define low_volt 2650
#define Hi_volt  3650

#define step_diag 10
#define step_volt (Hi_volt-low_volt)/step_diag

typedef struct SCommands {
	uint8_t sign[8];
	uint8_t id;
	uint8_t addr;
	uint8_t n_slave;
	uint16_t in_data;
	uint16_t out_data[CELL_LENGTH];
	uint16_t crc;
} SCommands_t;

typedef struct Cell_data {
	uint8_t n_slave;
	uint16_t u_raw[CELL_LENGTH];
	uint16_t u_raw_pole[step_diag];
	uint16_t u_raw_max;
	uint16_t u_raw_min;
	uint16_t u_mv[CELL_LENGTH];
	uint16_t u_mv_pole[10];
	uint16_t u_mv_max;
	uint16_t u_mv_min;
	uint16_t status[CELL_LENGTH];
	uint8_t shunt[CELL_LENGTH];
	int termo[CELL_LENGTH];
	uint8_t out_swich;

} Cell_data_t;
typedef enum {
	ENULL,
	ADC_GO,
	ADC_READ,
	LED_ERR_SET,
	LED_ERR_CLR,
	LED_RUN_SET,
	LED_RUN_CLR,
	RES_UP_SET,
	RES_DW_SET,
	RES_UP_CLR,
	RES_DW_CLR
} event_type_t;


extern Cell_data_t cell_sost;
void app_bms(void *pvParameters);
