/*
 * app.h
 *
 *  Created on: 22 сент. 2021 г.
 *      Author: ivanov
 */

#ifndef MAIN_APP_H_
#define MAIN_APP_H_
//typedef struct
//{
// uint16_t U_elem[ALLCELL];
// uint8_t B_elem[ALLCELL];
//}E_data_t;



extern uint8_t chipid[6];
extern uint32_t  serial_id;

extern char SNMP_COMMUNITY[32];
extern char SNMP_COMMUNITY_WRITE[32];
//extern E_data_t E_data;

 void start_task(void *pvParameters);


#endif /* MAIN_APP_H_ */
