/*
 * http_var.h
 *
 *  Created on: 18 ����. 2021 �.
 *      Author: ivanov
 */

#ifndef MAIN_HTML_HTTP_VAR_H_
#define MAIN_HTML_HTTP_VAR_H_

#include "../main/config_pj.h"
#include "esp_http_server.h"
#define http_ver 1
#define http_rev 2


typedef struct
{
	char name[32];
	uint8_t rsv1[8];
	uint16_t delay;
	uint8_t dir;
	uint8_t level_out;
	uint8_t rsv2;
	uint8_t pulse_dur;
}io_set_t;

extern uint8_t page_sost;
extern char buf[size_1k_buff * 1044];

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
__attribute__((used)) void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
__attribute__((used)) void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data);


#endif /* MAIN_HTML_HTTP_VAR_H_ */
