/* UART asynchronous example, that uses separate RX and TX tasks

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "bmsapp.h"


static const int RX_BUF_SIZE = 2 * BUFFER_LENGTH;
#define POLYNOM 0x8005  //x16 + x15 + x2 + 1 (reversed)
#define CRC_INIT 0xFFFF
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

#define CONFIG_BLINK_GPIO 2
#define BLINK_GPIO CONFIG_BLINK_GPIO
static const char *RX_TASK_TAG = "RX";
uint8_t sig[8] = { 0x12, 0x45, 0x76, 0x33, 0x57, 0xae, 0xfb, 0xe8 };

SCommands_t frame, frame_in;
Cell_data_t cell_sost={0};
uint8_t real_frame_buf[BUFFER_LENGTH];
uint16_t real_frame_indx;
uint8_t flag_start_frame;
uint8_t flag_frame_detect;

unsigned short crc16(uint16_t crc_in, uint8_t *data, uint16_t size)
//uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
	uint16_t out = 0;
	int bits_read = 0, bit_flag;

	/* Sanity check: */
	if (data == NULL)
		return 0;

	while (size > 0) {
		bit_flag = out >> 15;

		/* Get next bit: */
		out <<= 1;
		out |= (*data >> (7 - bits_read)) & 1;

		/* Increment bit counter: */
		bits_read++;
		if (bits_read > 7) {
			bits_read = 0;
			data++;
			size--;
		}

		/* Cycle check: */
		if (bit_flag)
			out ^= crc_in;
	}

	return out;
}

void UART0_swich_app(uint8_t *inb) {
	uint16_t crc_calc;
	memcpy((uint8_t*) (&frame_in.sign[0]), (uint8_t*) inb, BUFFER_LENGTH);

	crc_calc = crc16(CRC_INIT, (uint8_t*) (&frame_in.sign[0]),
	BUFFER_LENGTH - 3);

	if (frame_in.crc == crc_calc) {

		switch (frame_in.id) {
		case 20: {
			if (frame_in.addr == 0) {
				cell_sost.n_slave = frame_in.n_slave;
				ESP_LOGI(RX_TASK_TAG,
						"*******************Get addr cell ********************* \n\r ");
//				ESP_LOGI(RX_TASK_TAG, "Id 0x%x\n\r ", frame_in.id);
//				ESP_LOGI(RX_TASK_TAG, "IN_CRC16 0x%x\n\r ", frame_in.crc);
//				ESP_LOGI(RX_TASK_TAG, "N_SLAVE 0x%x\n\r  ", frame_in.n_slave);
//				ESP_LOGI(RX_TASK_TAG, "IN_DATA 0x%x\n\r  ", frame_in.in_data);
//				ESP_LOGI(RX_TASK_TAG,
//						"*******************************************\n\r\n\r  ");
			}

		}
			break;
		case 21: {
			uint8_t ct_cell;
			uint8_t ct_st;
			uint8_t lenc;
			lenc = cell_sost.n_slave * 2;
			memcpy(cell_sost.u_raw, frame_in.out_data, lenc);
			cell_sost.u_raw_max = 0;
			cell_sost.u_raw_min = 4096;
			ESP_LOGI(RX_TASK_TAG,
					"********************* Get raw U cell**********************\n\r  ");
//        	  for(ct_st=0;ct_st<step_diag;ct_st++)
//        	    {
//        		  cell_sost.u_raw_pole[ct_st]=0;
//        	    }
			for (ct_cell = 0; ct_cell < cell_sost.n_slave; ct_cell++) {
				if (cell_sost.u_raw[ct_cell] > cell_sost.u_raw_max) {
					cell_sost.u_raw_max = cell_sost.u_raw[ct_cell];
				}
				if (cell_sost.u_raw[ct_cell] < cell_sost.u_raw_min) {
					cell_sost.u_raw_min = cell_sost.u_raw[ct_cell];
				}


//				ESP_LOGI(RX_TASK_TAG, "U CELL#%d=%d\n\r  ", ct_cell,
//						cell_sost.u_raw[ct_cell]);
			}

//			ESP_LOGI(RX_TASK_TAG, "U min=%d\n\r  ", cell_sost.u_raw_min);
//			ESP_LOGI(RX_TASK_TAG, "U max=%d\n\r  ", cell_sost.u_raw_max);
//			ESP_LOGI(RX_TASK_TAG,
//					"*******************************************\n\r  ");
		}
			break;
		case 22: {
			uint8_t ct_cell;
			uint8_t ct_st;
			uint8_t lenc;
			lenc = cell_sost.n_slave * 2;
			memcpy(cell_sost.u_mv, frame_in.out_data, lenc);
			cell_sost.u_mv_max = low_volt;
			cell_sost.u_mv_min = Hi_volt;
			ESP_LOGI(RX_TASK_TAG,
					"********************Get U cell ******************* \n\r ");
			for (ct_st = 0; ct_st < step_diag; ct_st++) {
				cell_sost.u_mv_pole[ct_st] = 0;
			}
			for (ct_cell = 0; ct_cell < cell_sost.n_slave; ct_cell++) {
				if (cell_sost.u_mv[ct_cell] > cell_sost.u_mv_max) {
					cell_sost.u_mv_max = cell_sost.u_mv[ct_cell];
				}
				if (cell_sost.u_mv[ct_cell] < cell_sost.u_mv_min) {
					cell_sost.u_mv_min = cell_sost.u_mv[ct_cell];
				}

				for (ct_st = 0; ct_st < step_diag; ct_st++) {
					if ((cell_sost.u_mv[ct_cell]
							>= (low_volt + step_volt * ct_st))
							&& (cell_sost.u_mv[ct_cell]
									< (low_volt + step_volt * (ct_st + 1)))) {
						cell_sost.u_mv_pole[ct_st]++;
					}

				}
				if (cell_sost.u_mv[ct_cell] < low_volt) {
					cell_sost.u_mv_pole[0]++;
				}
				if (cell_sost.u_mv[ct_cell] > Hi_volt) {
					cell_sost.u_mv_pole[step_diag - 1]++;
				}
				ESP_LOGI(RX_TASK_TAG, "U CELL#%d=%dmV\n\r  ", ct_cell,
						cell_sost.u_mv[ct_cell]);
			}
			for (ct_st = 0; ct_st < step_diag; ct_st++) {
				if (cell_sost.u_mv_pole[ct_st]!=0)
				{
				ESP_LOGI(RX_TASK_TAG, "U %d-%d = %d cell \n\r  ",
						low_volt+step_volt*ct_st, low_volt+step_volt*(ct_st+1),
						cell_sost.u_mv_pole[ct_st]);
				}
			}
			ESP_LOGI(RX_TASK_TAG, "U min=%dmV\n\r  ", cell_sost.u_mv_min);
			ESP_LOGI(RX_TASK_TAG, "U max=%dmV\n\r  ", cell_sost.u_mv_max);
//			ESP_LOGI(RX_TASK_TAG,
//					"*******************************************\n\r");
		}
			break;

		case 23: {
			ESP_LOGI(RX_TASK_TAG,
					"********************* Power shunt get **********************\n\r ");

		}
			break;
		case 24: {
			ESP_LOGI(RX_TASK_TAG,
					"********************* Err  set **********************\n\r");
		}
			break;
		case 25: {
			uint8_t ct_cell;

			uint16_t termo_v;
			uint16_t termo_z;
			ESP_LOGI(RX_TASK_TAG,
					"******************Get termo ********************\n\r");
			for (ct_cell = 0; ct_cell < cell_sost.n_slave / 2; ct_cell++) {
				termo_v = frame_in.out_data[ct_cell * 2] / 10;
				termo_z = frame_in.out_data[(ct_cell * 2) + 1];
//				ESP_LOGI(RX_TASK_TAG, "Termo #%d=%dC*\n\r  ", ct_cell,
//						termo_v);
//				ESP_LOGI(RX_TASK_TAG, "Termo #%d=%dC*\n\r  ", ct_cell,
//						termo_z);
				if (termo_v < 1000) {
					if (termo_z == 0) {
						cell_sost.termo[ct_cell] = termo_v;
					} else {
						cell_sost.termo[ct_cell] = termo_v * (-1);
					}
				}
				if (cell_sost.termo[ct_cell]!=0){
				ESP_LOGI(RX_TASK_TAG, "Termo #%d=%dC*\n\r  ", ct_cell,
						cell_sost.termo[ct_cell]);
				}
			}

//			ESP_LOGI(RX_TASK_TAG,
//					"*******************************************\n\r\n\r  ");
		}
			break;
		case 26: {

			ESP_LOGI(RX_TASK_TAG,
					"********************* Power shunt set ********************** \n\r ");
		}
			break;

		default:

			break;
		}
	} else {
		ESP_LOGE(RX_TASK_TAG, "CRC ERR %x=%x \n\r  ", frame_in.crc, crc_calc);

	}

}

void restore_frame(uint8_t *inb) {
	int in_ct;

	for (in_ct = 0; in_ct < BUFFER_LENGTH; in_ct++) {
		if ((inb[in_ct] == sig[0]) && (inb[in_ct + 1] == sig[1])
				&& (inb[in_ct + 2] == sig[2]) && (inb[in_ct + 3] == sig[3])
				&& (inb[in_ct + 4] == sig[4]) && (inb[in_ct + 5] == sig[5])
				&& (inb[in_ct + 6] == sig[6]) && (inb[in_ct + 7] == sig[7])
				&& (flag_start_frame == 0)) {
			flag_start_frame = 1;
			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

			real_frame_buf[real_frame_indx] = inb[in_ct];
			in_ct += 1;
			real_frame_indx++;

		}

		if (flag_start_frame == 1) {
			if (real_frame_indx < BUFFER_LENGTH) {
				real_frame_buf[real_frame_indx] = inb[in_ct];
				real_frame_indx++;
//          if(real_frame_indx==BUFFER_LENGTH+1)
//          {
//           in_ct=-1;
//            flag_start_frame=0;
//            real_frame_indx=0;
//            UART0_swich_app(real_frame_buf,outb);
//            txBufferFull  = true;
//          }
			} else {
				in_ct = -1;
				flag_start_frame = 0;
				real_frame_indx = 0;
				UART0_swich_app(real_frame_buf);

				flag_frame_detect = 1;

			}

		}

	}

	if ((flag_frame_detect == 0) && (in_ct == BUFFER_LENGTH)) {
		//all_events.led_err_ext=LED_ERR_SET;
	}

}

void init(void) {
	const uart_config_t uart_config = { .baud_rate = 19200, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.source_clk = UART_SCLK_APB, };
	// We won't use a buffer for sending data.
	uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
	UART_PIN_NO_CHANGE);
}

int sendData(const char *logName, uint8_t *data) {

	int len, lenp;
	uint16_t ct_b;
	len = BUFFER_LENGTH;
	lenp = 1;
	int txBytes;
//	for(ct_b=0;ct_b<len;ct_b=ct_b+lenp)
//	{
//     txBytes = uart_write_bytes(UART_NUM_1,data+ct_b,lenp);
//     vTaskDelay(1 / portTICK_PERIOD_MS);
//	}

	txBytes = uart_write_bytes(UART_NUM_1, data, BUFFER_LENGTH);
	txBytes = len;


	ESP_LOGI(logName, "Wrote %d bytes\n\r", txBytes);
	return txBytes;
}

static void tx_task(void *arg) {
	static const char *TX_TASK_TAG = "TX_TASK";
	esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
	uint8_t data[BUFFER_LENGTH + 2];

	while (1) {

		memcpy(frame.sign, sig, 8);
		frame.id = 20;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;

//    	ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = 0x0ff0;
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		memcpy(frame.sign, sig, 8);
		frame.id = 21;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;

		//        ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = 0x0ff0;
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		memcpy(frame.sign, sig, 8);
		frame.id = 22;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;

//        ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = 0x0ff0;
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		memcpy(frame.sign, sig, 8);
		frame.id = 25;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;

		//        ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = 0x0ff0;
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		memcpy(frame.sign, sig, 8);
		frame.id = 26;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;
		//        ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = cell_sost.shunt[ct];
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		memcpy(frame.sign, sig, 8);
		frame.id = 24;
		frame.addr = 0;
		frame.n_slave = 0;
		frame.in_data = 0x0;
		//        ESP_LOGI(TX_TASK_TAG, "CRC16 0x%x ",frame.crc);
		for (uint8_t ct = 0; ct < CELL_LENGTH; ct++) {
			frame.out_data[ct] = cell_sost.status[ct];
		}
		frame.crc = crc16(CRC_INIT, &frame.sign, BUFFER_LENGTH - 3);
		sendData(TX_TASK_TAG, (uint8_t*) (&frame.sign));
		vTaskDelay(2000 / portTICK_PERIOD_MS);

	}
	//   free(data);
}

static void rx_task(void *arg) {

	esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	uint8_t *data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
	while (1) {
		const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE,
				1000 / portTICK_RATE_MS);
		if (rxBytes > 0) {

			UART0_swich_app(data);
			//	restore_frame(data);
			data[rxBytes] = 0;
//			ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'\n\r", rxBytes, data);
			//   ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
		}
	}
	free(data);
}

static void ball_cell(void *arg) {
	uint8_t cell_ct;

	while (1) {
		vTaskDelay(10000 / portTICK_PERIOD_MS);
		for (cell_ct = 0; cell_ct < cell_sost.n_slave; cell_ct++) {
			if ((cell_sost.u_mv[cell_ct] > 3450)&&(cell_sost.u_mv[cell_ct] < 3550)) {
				cell_sost.shunt[cell_ct] = 25;
				ESP_LOGW(RX_TASK_TAG, "cell_sost.shunt[%d] =25\n\r  ",cell_ct);
			}else if ((cell_sost.u_mv[cell_ct] > 3550)&&(cell_sost.u_mv[cell_ct] < 3600)) {
				cell_sost.shunt[cell_ct] = 50;
				ESP_LOGW(RX_TASK_TAG, "cell_sost.shunt[%d] =50\n\r  ",cell_ct);
			}else if (cell_sost.u_mv[cell_ct] > 3600) {
				cell_sost.shunt[cell_ct] = 100;
				ESP_LOGW(RX_TASK_TAG, "cell_sost.shunt[%d] =100\n\r  ",cell_ct);
			}
			else
			{
				cell_sost.shunt[cell_ct] = 0;
			}

			if (cell_sost.u_mv[cell_ct] < low_volt) {
			 cell_sost.status[cell_ct]=LED_ERR_SET;
			 cell_sost.out_swich=1;

			}
			else if (cell_sost.u_mv[cell_ct] > Hi_volt) {
				 cell_sost.status[cell_ct]=LED_ERR_SET;
				 cell_sost.out_swich=1;

				}
			else
			{
				cell_sost.status[cell_ct]=LED_ERR_CLR;
				cell_sost.out_swich=0;
			}

		}

	}

}
//static void led_task(void *arg) {
//	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
//
//	while (1) {
//		gpio_set_level(BLINK_GPIO, 0);
//		ESP_LOGI("LED",
//							"********************* LED CLR  ********************** \n\r ");
//		vTaskDelay(700 / portTICK_PERIOD_MS);
//		/* Blink on (output high) */
//		//    printf("Turning on the LED\n");
//		gpio_set_level(BLINK_GPIO, 1);
//		ESP_LOGI("LED",
//									"********************* LED SET  ********************** \n\r ");
//		vTaskDelay(700 / portTICK_PERIOD_MS);
//	}
//}

void app_bms(void *pvParameters) {
	ESP_LOGI(RX_TASK_TAG,
						"********************* Start  ********************** \n\r ");
	init();

//	cell_sost.status[0] = LED_ERR_SET;
//	cell_sost.shunt[0] = 50;
//	cell_sost.shunt[1] = 100;
//
//	cell_sost.shunt[2] = 10;
	xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES,
	NULL);
	xTaskCreate(ball_cell, "ball_cell", 1024 * 4, NULL, configMAX_PRIORITIES,
	NULL);
	xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL,
	configMAX_PRIORITIES - 1, NULL);
	vTaskSuspend(NULL);
//	xTaskCreate(led_task, "led_task", 1024*4, NULL, configMAX_PRIORITIES - 1,
//	NULL);
}
