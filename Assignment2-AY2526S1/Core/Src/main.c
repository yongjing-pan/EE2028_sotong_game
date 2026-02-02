// ssd1306_2d_any.c
#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>   // for UINT16_MAX
#include <stdbool.h>
#include "math.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"


#define SSD1306_BUFFER_SIZE  ((SSD1306_WIDTH * SSD1306_HEIGHT) / 8)
#define WINDOW_SIZE 10
#define BUZZER_PIN GPIO_PIN_3
#define BUZZER_PORT GPIOA  //buzzer
#define IR_SENSOR_PORT GPIOB
#define IR_SENSOR_PIN  GPIO_PIN_2   // D8 pin

const uint8_t epd_bitmap_R[128*64/8]={0};
static void MX_GPIO_Init(void);
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
static void UART1_Init(void);
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;


uint32_t time_press = 0;
uint8_t button_press_count = 0;
bool is_game_1 = true;
uint32_t time_colour = 0;
uint32_t time_read = 0;
uint32_t time_toggle = 0;
bool is_green_light = true;
int num_green_light = 0;

uint32_t time_env = 0;
uint32_t time_blink = 0;
uint32_t time_prox = 0;
float mag_strength = 0.0f;

bool player_near = false;
bool escaped = false;
uint32_t blink_interval = 500;

#define W 128
#define H 64
// ====== OLED config ======
#define SSD1306_WIDTH     128
#define SSD1306_HEIGHT     64
#define SSD1306_I2C_ADDR  (0x3C << 1)

void uart_send(const char *msg){
	if (!msg) return;
	if (strlen(msg) == 0) return;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg),0xFFFF);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        uint32_t now = HAL_GetTick();
        if ((button_press_count == 0) || (now-time_press>1000)) {
            button_press_count = 1;
            time_press = now;
            if(!is_game_1){
            	 if (!escaped && player_near && (HAL_GetTick() - time_prox <= 3000)) {
            		 time_prox = HAL_GetTick();
            		 uart_send("\"Player captured, good job!\"\r\n");
            	  }
            }
        } else  {//((now - time_press) <= 1000)
            is_game_1 = !is_game_1;
            if (is_game_1) {
                uart_send("\"Entering Red Light, Green Light as Enforcer\"\r\n========\r\n");
                uart_send("\r\n\"Green Light!\"\r\n\r\n");
                time_colour = HAL_GetTick();
                time_read = time_colour;
                is_green_light = true;
                num_green_light = 0;
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
//                _show_rowmajor(epd_bitmap_R, /*msb_first=*/1);
            } else { // game 2
                uart_send("\"Entering Catch & Run as Enforcer\"\r\n========\r\n");
                time_press = now;
                HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
//                _show_rowmajor(epd_bitmap_R, /*msb_first=*/1);
            }
            button_press_count = 0; // reset
        }
    }
}

float buffer[WINDOW_SIZE] = {0}; float buffer2[WINDOW_SIZE] = {0};
int ind = 0; int ind2 = 0;
int count = 0, count2 = 0;
float sum = 0, sum2 = 0;

float avg_lin(float new_value) {
    sum -= buffer[ind];
    buffer[ind] = new_value;
    sum += new_value;

    ind = (ind + 1) % WINDOW_SIZE;

    if (count < WINDOW_SIZE) count++;

    return sum / count;
}
float avg_ang(float new_value) {
    sum2 -= buffer2[ind2];
    buffer2[ind2] = new_value;
    sum2 += new_value;

    ind2 = (ind2 + 1) % WINDOW_SIZE;

    if (count2 < WINDOW_SIZE) count2++;

    return sum2 / count2;
}

static uint8_t fb[SSD1306_WIDTH * SSD1306_HEIGHT / 8];  // 1024 bytes

static HAL_StatusTypeDef ssd1306_cmd(uint8_t c) {
  uint8_t buf[2] = {0x00, c};
  return HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef ssd1306_data(uint8_t *p, uint16_t n) {
  uint8_t chunk[17]; chunk[0] = 0x40;
  while (n) {
    uint16_t k = (n > 16) ? 16 : n;
    memcpy(&chunk[1], p, k);
    if (HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, chunk, k + 1, HAL_MAX_DELAY) != HAL_OK)
      return HAL_ERROR;
    p += k; n -= k;
  }
  return HAL_OK;
}

static void ssd1306_update_full(void){
  ssd1306_cmd(0x21); ssd1306_cmd(0x00); ssd1306_cmd(SSD1306_WIDTH-1);
  ssd1306_cmd(0x22); ssd1306_cmd(0x00); ssd1306_cmd((SSD1306_HEIGHT/8)-1);
  HAL_StatusTypeDef ret = ssd1306_data(fb, (uint16_t)SSD1306_BUFFER_SIZE);
}

void oled_show_ssd1306_framebuffer(const uint8_t *pagebuf)
{
    memcpy(fb, pagebuf, sizeof(fb));
    ssd1306_update_full();
}

static void ssd1306_init(void){

  if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 100) != HAL_OK) {
  }

  ssd1306_cmd(0xAE); // OFF
  ssd1306_cmd(0xD5); ssd1306_cmd(0x80);
  ssd1306_cmd(0xA8); ssd1306_cmd(0x3F);
  ssd1306_cmd(0xD3); ssd1306_cmd(0x00);
  ssd1306_cmd(0x40);
  ssd1306_cmd(0x8D); ssd1306_cmd(0x14);
  ssd1306_cmd(0x20); ssd1306_cmd(0x00);
  ssd1306_cmd(0xA1);
  ssd1306_cmd(0xC8);
  ssd1306_cmd(0xDA); ssd1306_cmd(0x12);
  ssd1306_cmd(0x81); ssd1306_cmd(0x8F);
  ssd1306_cmd(0xD9); ssd1306_cmd(0xF1);
  ssd1306_cmd(0xDB); ssd1306_cmd(0x40);
  ssd1306_cmd(0xA4);
  ssd1306_cmd(0xA6);
  ssd1306_cmd(0x2E);
  ssd1306_cmd(0xAF); // ON

}
void oled_show_bitmap(const uint8_t img[SSD1306_HEIGHT][SSD1306_WIDTH]) {
  memset(fb, 0, sizeof(fb));
  for (int y = 0; y < SSD1306_HEIGHT; ++y) {
    int page = y >> 3;
    uint8_t bit = 1u << (y & 7);
    uint32_t rowBase = page * SSD1306_WIDTH;
    for (int x = 0; x < SSD1306_WIDTH; ++x) {
      if (img[y][x]) fb[rowBase + x] |= bit;
    }
  }
  ssd1306_update_full();
}

void oled_checker(void) {
  ssd1306_init();
  static uint8_t checker[SSD1306_HEIGHT][SSD1306_WIDTH];
  for (int y = 0; y < SSD1306_HEIGHT; ++y)
    for (int x = 0; x < SSD1306_WIDTH; ++x)
      checker[y][x] = ((x >> 3) ^ (y >> 3)) & 1;

  oled_show_bitmap(checker);

}

static void _show_rowmajor(const uint8_t *bmp, int msb_first)
{
    static uint8_t img[SSD1306_HEIGHT][SSD1306_WIDTH];
    for (int y = 0; y < SSD1306_HEIGHT; ++y) {
        const int row_off = y * (SSD1306_WIDTH/8);
        for (int x = 0; x < SSD1306_WIDTH; ++x) {
            int byte_idx = row_off + (x >> 3);
            int bit_idx  = msb_first ? (7 - (x & 7)) : (x & 7);
            img[y][x] = (bmp[byte_idx] >> bit_idx) & 1u;
        }
    }
    oled_show_bitmap(img);
}

static void _show_pageorder(const uint8_t *pagebuf, int invert)
{
    if (!invert) {
        oled_show_ssd1306_framebuffer(pagebuf);
    } else {
        for (int i = 0; i < (SSD1306_WIDTH*SSD1306_HEIGHT/8); ++i)
            fb[i] = (uint8_t)~pagebuf[i];
        ssd1306_update_full();
    }
}

void oled_show_packed_auto(const uint8_t *bmp)
{
    _show_pageorder(bmp, /*invert=*/0);
}



static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  	// Enable NVIC EXTI line 13
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	//initialise LED
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	//BUZZER
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BUZZER_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  //OLED
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitStruct.Pin = IR_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // or GPIO_PULLUP if your sensor requires it
  HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);
}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C0216C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    while (1);
  }
}



int main(void)
{
	// in main.c
	initialise_monitor_handles();
	HAL_Init();
	MX_GPIO_Init();
	MX_I2C1_Init();
	UART1_Init();
	BSP_TSENSOR_Init();
	BSP_LED_Init(LED2);
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();


    float hum = 60.0;
	int16_t mag[3] = {0,0,0};
	float ang_accel[3] = {0.0, 0.0, 0.0}; float ang_thres = 3600.0, ang_change = 0.0;
	float prev_ang_accel = -1.0, ang_accel_mag; //0.28->0.3
	float temp = 25.0;
	float pres = 0.0;
	float accel_data[3]; float lin_thres = 1.0, lin_change = 0.0;
	float prev_accel = -1.0, accel_mag; // 229.89 --> 253
	int wait_reads = 80, cur_counter = 0;
	int16_t accel_data_i16[3] = { 0 };
	float temp_thres = 30.0, hum_thres = 80.0, pres_thres = 1020.0;
	char msg[100];
//	float temp_thres = 29.0, hum_thres = 56.0, pres_thres = 1000.0;

	time_colour = HAL_GetTick();
	time_read = time_colour;
	is_green_light = true;
	bool ir_covered = false;

	uart_send("\"Entering Red Light, Green Light as Enforcer\"\r\n========\r\n");
	uart_send("\r\n\"Green Light!\"\r\n\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

	_show_rowmajor(epd_bitmap_R, 1);
	while (1)
	{
		if(is_game_1){

			if (HAL_GetTick() - time_colour >= 10000){ //Switch colour
				time_colour = HAL_GetTick();
				is_green_light = !is_green_light;
				if (is_green_light){
					uart_send("\r\n\"Green Light!\"\r\n\r\n");
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) != GPIO_PIN_SET){
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					}
				}else{
					uart_send("\r\n\"Red Light!\"\r\n\r\n");
					time_toggle = HAL_GetTick();
				}
			}

			if(is_green_light){

				if(HAL_GetTick() - time_read >= 2000){
					time_read = HAL_GetTick();
					char msg[60];
					sprintf(msg, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f%%\r\n",
							temp, pres, hum);
					uart_send(msg);
				}else{// read readings
					temp = BSP_TSENSOR_ReadTemp();
					hum = BSP_HSENSOR_ReadHumidity();
					pres = BSP_PSENSOR_ReadPressure();

				}

			}else{//red light
				if(HAL_GetTick() - time_read >= 2000){
					time_read = HAL_GetTick();

					sprintf(msg, "Linear Accel X, Y, Z (m/s^2): %.2f, %.2f, %.2f; \r\n", accel_data[0], accel_data[1], accel_data[2]);
					uart_send(msg);

					sprintf(msg, "Angular Accel X, Y, Z (millideg/s): %.2f, %.2f, %.2f; \r\n", ang_accel[0]*0.07, ang_accel[1]*0.07, ang_accel[2]*0.07);
					uart_send(msg);

				}else if(HAL_GetTick() - time_toggle >= 500){//toggle led
					time_toggle = HAL_GetTick();
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

				}else{//read readings
					BSP_ACCELERO_AccGetXYZ(accel_data_i16);
					accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
					accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
					accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
					accel_mag = sqrt(accel_data[0]*accel_data[0]+
									accel_data[1]*accel_data[1]+
									accel_data[2]*accel_data[2]);

					accel_mag = avg_lin(accel_mag);

					if(prev_accel != -1.0){
						lin_change = accel_mag-prev_accel;
						if(lin_change<0.0) lin_change = -lin_change;
					}
					prev_accel = accel_mag;

					BSP_GYRO_GetXYZ(ang_accel);
					ang_accel_mag = sqrt(ang_accel[0]*ang_accel[0]+
										ang_accel[1]*ang_accel[1]+
										ang_accel[2]*ang_accel[2]);
					ang_accel_mag = avg_ang(ang_accel_mag);

					if(prev_ang_accel != -1.0){
						ang_change = ang_accel_mag-prev_ang_accel;
						if(ang_change<0.0) ang_change = -ang_change;
					}
					prev_ang_accel = ang_accel_mag;
					if (cur_counter > 0) {
						cur_counter--;
					}
					else if(cur_counter == 0 && (lin_change > lin_thres || ang_change > ang_thres)){
						uart_send("\"Player Out!\"\r\n");
						cur_counter = wait_reads;
						 HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
					}else if(HAL_GPIO_ReadPin(BUZZER_PORT, BUZZER_PIN) == GPIO_PIN_SET){//stop buzzer after 80 rounds
						 HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
					}
				}
			}
			if (!ir_covered && HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_RESET) {
				oled_checker();
				ir_covered = true;
			}else if (ir_covered && HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_SET){
				_show_rowmajor(epd_bitmap_R,  1);
				ir_covered = false;
			}

		}else{//game 2
			uint32_t now = HAL_GetTick();
			BSP_MAGNETO_GetXYZ(mag);
	        mag_strength = sqrtf((float)mag[0]*mag[0] +
	                             (float)mag[1]*mag[1] +
	                             (float)mag[2]*mag[2]);

	        if (mag_strength > 2600.0f) {  // Adjust threshold
	            if (!player_near) {
	                player_near = true;
	                time_prox = now;
	                escaped = false;
	                uart_send("\"Player is Nearby! Move faster.\"\r\n");
	            }

	            if (mag_strength < 4000.0) blink_interval = 1200; //first speed
	            else if (mag_strength < 6500.0f) blink_interval = 300;//second speed
	            else blink_interval = 80; //third speed

	            if (now - time_blink >= blink_interval) {
	                time_blink = now;
	                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	                if(!ir_covered) HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN); // Buzzer toggle
	            }
				if (!escaped && (now - time_prox > 3000)) {
	                uart_send("\"Player escaped! Keep trying.\"\r\n");
	                player_near = false;
	                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	            }
	        } else { //mag_stregnth < 2600
	            player_near = false;
	            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	            if(!ir_covered) HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
	        }

	        if (now - time_env >= 1000) {
	            time_env = now;

	            temp = BSP_TSENSOR_ReadTemp();
	            hum  = BSP_HSENSOR_ReadHumidity();
	            pres = BSP_PSENSOR_ReadPressure();

	            if (temp > temp_thres) {
	                sprintf(msg, "\"Temperature spike detected! T:%.1fC. Dangerous environment!\"\r\n", temp);
	                uart_send(msg);
	            }
	            if (hum > hum_thres) {
	                sprintf(msg, "\"Humidity spike detected! H:%.1f%%. Dangerous environment!\"\r\n", hum);
	                uart_send(msg);
	            }
	            if (pres > pres_thres) {
	                sprintf(msg, "\"Pressure spike detected! P:%.1f hPa. Dangerous environment!\"\r\n", pres);
	                uart_send(msg);
	            }
	        }

	        if (!ir_covered && HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_RESET) {
	        	HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
	        	ir_covered = true;
	        }else if (ir_covered && HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_SET){
	            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
	        	ir_covered = false;
	        }


		}
	}

}




