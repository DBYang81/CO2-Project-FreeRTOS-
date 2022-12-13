#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "heap_lock_monitor.h"
#include "retarget_uart.h"
#include "ModbusRegister.h"
#include "DigitalIoPin.h"
#include "LiquidCrystal.h"
#include <string>



#define QUEUE_LENGTH1 10
#define QUEUE_LENGTH2 10
#define BINARY_SEMAPHORE_LENGTH 1
#define COMBINE_LENGTH (QUEUE_LENGTH1 + QUEUE_LENGTH2 + BINARY_SEMAPHORE_LENGTH)

typedef enum {
	temperatureSensor,
	humiditySensor,
	CO2Sensor
}Sensors;

typedef struct {
	int sensorValue;
	Sensors sensor;
}Data;

static QueueHandle_t sensorQueue = NULL, ISRQueue = NULL;
static QueueSetHandle_t xQueueSet;
QueueHandle_t xSemaphore;

SemaphoreHandle_t xMutex;

// TODO: insert other definitions and declarations here

//ISR for rotary encoder turn (siga only)
extern "C" {
	void PIN_INT0_IRQHandler(void)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		int inc = 1;
		int dec = -inc;

		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));

		if (Chip_GPIO_GetPinState(LPC_GPIO, 0, 6)) {
			xQueueSendFromISR(ISRQueue, (void*)&inc, &xHigherPriorityTaskWoken);
		}
		else {
			xQueueSendFromISR(ISRQueue, (void*)&dec, &xHigherPriorityTaskWoken);
		}

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

//ISR for rotary encoder button
extern "C" {
	void PIN_INT1_IRQHandler(void)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));

		int val = 0;

		xQueueSendFromISR(ISRQueue, (void*)&val, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

/* The following is required if runtime statistics are to be collected
 * Copy the code to the source file where other you initialize hardware */
extern "C" {

	void vConfigureTimerForRunTimeStats(void) {
		Chip_SCT_Init(LPC_SCTSMALL1);
		LPC_SCTSMALL1->CONFIG = SCT_CONFIG_32BIT_COUNTER;
		LPC_SCTSMALL1->CTRL_U = SCT_CTRL_PRE_L(255) | SCT_CTRL_CLRCTR_L; // set prescaler to 256 (255 + 1), and start timer
	}

}
/* end runtime statictics collection */

static void idle_delay()
{
	vTaskDelay(1);
}

/*
void task1(void* params)
{
	(void)params;

	retarget_init();

	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister RH(&node3, 256, true);

	DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay
	relay.write(0);

	DigitalIoPin sw_a2(1, 8, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a3(0, 5, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a4(0, 6, DigitalIoPin::pullup, true);
	DigitalIoPin sw_a5(0, 7, DigitalIoPin::pullup, true);


	while (true) {
		float rh;
		char buffer[32];

		vTaskDelay(2000);

		rh = RH.read() / 10.0;
		snprintf(buffer, 32, "RH=%5.1f%%", rh);
		printf("%s\n", buffer);
		lcd->setCursor(0, 1);
		// Print a message to the LCD.
		lcd->print(buffer);

	}
}

*/

/* Temp&humidity sensor reading by modus - high priority */
static void vSensorReadTask(void* pvParameters) {
	TickType_t xLastWakeTime;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	xLastWakeTime = xTaskGetTickCount();

	//Temperature sensor
	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister TE(&node3, 257, true);

	//Humidity sensor
	ModbusRegister RH(&node3, 256, true);

	DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay
	relay.write(1);

	//CO2 Value reading
	xLastWakeTime = xTaskGetTickCount();
	ModbusMaster node4(240);
	node4.begin(9600); // all nodes must operate at the same speed!
	node4.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister CO2(&node4, 256, true);


	for (;; )
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
		//xSemaphoreTake(xMutex, portMAX_DELAY);
		Data CO2Value = { CO2.read(),CO2Sensor };
		vTaskDelay(5);
		Data TEValue = { TE.read() / 10, temperatureSensor };
		vTaskDelay(5);
		Data RHValue = { RH.read() / 10, humiditySensor };
		vTaskDelay(5);
		//xSemaphoreGive(xMutex);
		xStatus = xQueueSendToBack(sensorQueue, &TEValue, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Temperature sensor could not send to the queue.\r\n");
		}

		xStatus = xQueueSendToBack(sensorQueue, &RHValue, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Humidity sensor could not send to the queue.\r\n");
		}
		xStatus = xQueueSendToBack(sensorQueue, &CO2Value, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Co2 sensor could not send to the queue.\r\n");
		}

	}
}

#if 0
/* C02 sensor reading by modus controlled by relay - high priority*/
static void vC02Detected(void* pvParameters) {

	DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay
	relay.write(10);

	TickType_t xLastWakeTime;
	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
	xLastWakeTime = xTaskGetTickCount();
	ModbusMaster node4(240);
	node4.begin(9600); // all nodes must operate at the same speed!
	node4.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister CO2(&node4, 256, true);

	for (;; )
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
		xSemaphoreTake(xMutex, portMAX_DELAY);
		Data CO2Value = { CO2.read(),CO2Sensor };
		xStatus = xQueueSendToBack(sensorQueue, &CO2Value, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Temperature sensor could not send to the queue.\r\n");
		}
		xSemaphoreGive(xMutex);
	}
}
#endif

/* LCD display thread - medium priority */
static void vLcdDisplay(void* pvParameters) {
	DigitalIoPin* rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
	DigitalIoPin* en = new DigitalIoPin(0, 9, DigitalIoPin::output);
	DigitalIoPin* d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
	DigitalIoPin* d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
	DigitalIoPin* d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
	DigitalIoPin* d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
	LiquidCrystal* lcd = new LiquidCrystal(rs, en, d4, d5, d6, d7);
	// configure display geometry
	lcd->begin(16, 2);

	const TickType_t xBlockTime = pdMS_TO_TICKS(100);
	Data sensorData;
	QueueHandle_t xQueueThatContainsData;

	int isr_val;
	bool edit_state = false;
	int co2_target = 500;
	int co2_level = co2_target;

	int counter = 0;
	char buffer[50];
	while (1) {
		counter++;
		if(counter > 30){
			lcd->clear();
			counter = 0;
		}

		lcd->setCursor(14, 0);
		DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay
		sprintf(buffer, "R%d", relay.read());
		lcd->print(buffer);

		xQueueThatContainsData = (QueueHandle_t)xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

		if (xQueueThatContainsData == sensorQueue) {
			xQueueReceive(xQueueThatContainsData, &sensorData, 0);
			// set the cursor to column 0, line 1
			// (note: line 1 is the second row, since counting begins with 0):

			if (sensorData.sensor == temperatureSensor) {
				lcd->setCursor(0, 0);
				sprintf(buffer, "T:%d C", sensorData.sensorValue);
				lcd->print(buffer);

			}
			else if (sensorData.sensor == humiditySensor) {
				lcd->setCursor(7, 0);
				sprintf(buffer, "H:%d %%", sensorData.sensorValue);
				lcd->print(buffer);

			}
			else {
				lcd->setCursor(0, 1);
				co2_level = sensorData.sensorValue;
				sprintf(buffer, "CO2:%d ppm", sensorData.sensorValue);
				lcd->print(buffer);
				lcd->setCursor(12, 1);
				sprintf(buffer, "%d", co2_target);
				lcd->print(buffer);
			}
		}
		if (xQueueThatContainsData == ISRQueue) {

			if(xQueueReceive(xQueueThatContainsData, &isr_val, 0) == pdPASS){
				vTaskDelay(10);
				xQueueReset(ISRQueue);
				if(isr_val == 0){
					edit_state = !edit_state;
				}else if(edit_state){
					co2_target += isr_val;
				}
			};

		}

		if(co2_level < co2_target){
			relay.write(1);
		}else{
			relay.write(0);
		}
	}
	vTaskDelay(xBlockTime);
}

/* Uart output thread - low priority */
static void vUARTTask(void* pvParameters) {

	Data sensorData;
	//	int count = 0;
	//	char str[60];
	char buff[100];

	retarget_init();


	QueueHandle_t xQueueThatContainsData;

	while (1) {
		//			int bytes = dbgu->read(str+count, 60-count);
		//			if(bytes > 0){
		//				count += bytes;
		//				str[count] = '\0';
		//				dbgu->write(str+count-bytes, bytes);

		xQueueThatContainsData = (QueueHandle_t)xQueueSelectFromSet(xQueueSet, portMAX_DELAY);
		if (xQueueThatContainsData == sensorQueue) {
			xQueueReceive(xQueueThatContainsData, &sensorData, 0);
			if (sensorData.sensor == temperatureSensor) {
				snprintf(buff, 100, "Temp: %d \r\n", sensorData.sensorValue);
				printf("%s", buff);
			}
		}
		vTaskDelay(configTICK_RATE_HZ / 10);
	}
}


///* Rotary encoder thread - GPIO interrupt*/
//static void vRotaryEncoder(void* pvParameters) {
//}
//
///* WiFi and Mqtt thread */
//static void vMqtt(void* pvParamenters) {
//}

extern "C" {
	void vStartSimpleMQTTDemo(void); // ugly - should be in a header
}

int main(void) {
#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
	Board_LED_Set(0, true);
#endif
#endif
	//Create queue for read sensor values
	xQueueSet = xQueueCreateSet(COMBINE_LENGTH);
	sensorQueue = xQueueCreate(QUEUE_LENGTH1, sizeof(Data));
	ISRQueue = xQueueCreate(QUEUE_LENGTH2, sizeof(int));
	xSemaphore = xSemaphoreCreateBinary();
	xMutex = xSemaphoreCreateMutex();

	xQueueAddToSet(sensorQueue, xQueueSet);
	xQueueAddToSet(ISRQueue, xQueueSet);
	xQueueAddToSet(xSemaphore, xQueueSet);
	vQueueAddToRegistry(sensorQueue, "sensorQ");

	heap_monitor_setup();

	// initialize RIT (= enable clocking etc.)
	//Chip_RIT_Init(LPC_RITIMER);
	// set the priority level of the interrupt
	// The level must be equal or lower than the maximum priority specified in FreeRTOS config
	// Note that in a Cortex-M3 a higher number indicates lower interrupt priority
	//NVIC_SetPriority( RITIMER_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1 );

	/* ISR SETUP START */
	//rotary encoder pin setup
	DigitalIoPin sw_a2(1, 8, DigitalIoPin::pullup, true); //button
	DigitalIoPin sw_a3(0, 5, DigitalIoPin::pullup, true); //siga
	DigitalIoPin sw_a4(0, 6, DigitalIoPin::pullup, true); //sigb



	/* Initialize PININT driver */
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	/* Enable PININT clock */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PININT);

	/* Reset the PININT block */
	Chip_SYSCTL_PeriphReset(RESET_PININT);

	/* Configure interrupt channel for the GPIO pin in INMUX block */
	Chip_INMUX_PinIntSel(0, 0, 5);
	Chip_INMUX_PinIntSel(1, 1, 8);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0));

	//NVIC_SetPriority(PIN_INT0_IRQn, );
	//NVIC_SetPriority(PIN_INT1_IRQn, );

	/* Enable interrupt in the NVIC */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	/* ISR SETUP END */


	xTaskCreate(vUARTTask, "Debug",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);


	xTaskCreate(vLcdDisplay, "LcdDisplay",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);

	//Create task for reading temperature and humidity sensors value
	xTaskCreate(vSensorReadTask, "sensorReadTask",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);

	//Create task for reading CO2 sensor value
//	xTaskCreate(vC02Detected, "CO2Task",
//		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 2UL),
//		(TaskHandle_t*)NULL);

	//vStartSimpleMQTTDemo();

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
