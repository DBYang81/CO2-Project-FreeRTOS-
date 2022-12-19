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
#include "timers.h"
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
#define EEPROM_ADDRESS 0x00000100


typedef enum {
	temperatureSensor,
	humiditySensor,
	CO2Sensor
}Sensors;

typedef struct {
	int sensorValue;
	Sensors sensor;
}Data;

typedef enum {
	co2,
	relativehumidity,
	temperature,
	valveOpenPerc,
	co2SetPoint
}DataTypes;

typedef struct {
	int value;
	DataTypes DataType;
}mqttData;

typedef struct {
	int co2;
	int relativehumidity;
	int temperature;
	int valveOpenPerc;
	int co2SetPoint;
}mqttStrData;


static QueueHandle_t sensorQueue = NULL, ISRQueue = NULL, mqttQueue = NULL, strQueue = NULL;
static QueueSetHandle_t xQueueSet;
QueueHandle_t xSemaphore;

SemaphoreHandle_t xMutex;

TimerHandle_t manual_reset_timer;
int reset_count = 2;

TimerHandle_t manual_relay_timer;
bool manual_relay = false;
int relay_count = 9;

void vTimerCallbackReset( TimerHandle_t xTimer ) {
	reset_count--;
	if (!Chip_GPIO_GetPinState(LPC_GPIO, 1, 8)) {
		xTimerStop(manual_reset_timer, portMAX_DELAY);
		reset_count = 2;
	}
	if (reset_count < 0) {
		xTimerStop(manual_reset_timer, portMAX_DELAY);
		//reset_count = 2;
		//relay_count = 0;
		NVIC_SystemReset();
	}
}

void vTimerCallbackRelay( TimerHandle_t xTimer ) {
	relay_count--;
	if (relay_count < 0) {
		manual_relay = false;
		xTimerStop(manual_relay_timer, portMAX_DELAY);
		relay_count = 9;
	}
}

TimerHandle_t relay_60s_limit;
TimerHandle_t relay_3s_on_timer;
//bool relay_60s_enable = true;
bool relay_enable = true;

void vTimerCallback60s( TimerHandle_t xTimer ) {
	relay_enable = true;
	//xTimerStart(relay_3s_on_timer, portMAX_DELAY);
	//relay_60s_enable = true;
}

void vTimerCallback3s( TimerHandle_t xTimer ) {
	relay_enable = false;
	xTimerStart(relay_60s_limit, portMAX_DELAY);
}

// TODO: insert other definitions and declarations here

//ISR for rotary encoder turn (siga only)
extern "C" {
	void PIN_INT0_IRQHandler(void)
	{
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		int inc = 1;
		int dec = -inc;

		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));

		//NVIC_DisableIRQ(PIN_INT0_IRQn);

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
	//relay.write(1);

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
		vTaskDelay(50);
		Data TEValue = { TE.read() / 10, temperatureSensor };
		vTaskDelay(50);
		Data RHValue = { RH.read() / 10, humiditySensor };
		vTaskDelay(50);
		//xSemaphoreGive(xMutex);
		
		mqttData Co2 = { CO2.read(),co2 };
		vTaskDelay(5);
		mqttData te = { TE.read() / 10,temperature };
		vTaskDelay(5);
		mqttData rh = { RH.read() / 10,relativehumidity };
		vTaskDelay(5);
		
		xStatus = xQueueSendToBack(sensorQueue, &TEValue, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Temperature sensor could not send to the queue.\r\n");
		}

		xStatus = xQueueSendToBack(mqttQueue, &te, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Temperature could not send to the queue.\r\n");
		}

		xStatus = xQueueSendToBack(sensorQueue, &RHValue, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Humidity sensor could not send to the queue.\r\n");
		}
		
		xStatus = xQueueSendToBack(mqttQueue, &rh, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Humidity could not send to the queue.\r\n");
		}
		
		xStatus = xQueueSendToBack(sensorQueue, &CO2Value, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("Co2 sensor could not send to the queue.\r\n");
		}
		
		xStatus = xQueueSendToBack(mqttQueue, &Co2, xTicksToWait);
		if (xStatus != pdPASS)
		{
			printf("CO2 could not send to the queue.\r\n");
		}
	}
}

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

	const TickType_t xBlockTime = pdMS_TO_TICKS(10);
	Data sensorData;
	QueueHandle_t xQueueThatContainsData;

	int isr_val;
	//uint8_t *ptr;
	int co2_target;// = 800;
	vTaskSuspendAll();
	Chip_EEPROM_Read(EEPROM_ADDRESS, (uint8_t *)&co2_target, sizeof(uint32_t));
	xTaskResumeAll();
	//co2_target = ptr[0] + (ptr[1] << 8) + (ptr[2] << 16) + (ptr[3] << 24);
	int co2_level = co2_target;
	int val_mult = 20;
	//bool manual_relay = false;
	int manual_relay_count = 0;
	bool started = false;

	DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay

	int counter = 0;
	char buffer[50];

	while (1) {
		counter++;
		lcd->setCursor(13, 0);
		if (manual_relay) {
			sprintf(buffer, "R:%d", relay_count);
		}
		else {
			sprintf(buffer, "R:%d", relay.read());
		}
		lcd->print(buffer);

		xQueueThatContainsData = (QueueHandle_t)xQueueSelectFromSet(xQueueSet, portMAX_DELAY);

		if (xQueueThatContainsData == sensorQueue) {
			xQueueReceive(xQueueThatContainsData, &sensorData, 0);
			// set the cursor to column 0, line 1
			// (note: line 1 is the second row, since counting begins with 0):

			if (sensorData.sensor == temperatureSensor) {
				lcd->setCursor(0, 0);
				snprintf(buffer, 6, "T:%dC      ", sensorData.sensorValue);
				lcd->print(buffer);

			}
			else if (sensorData.sensor == humiditySensor) {
				lcd->setCursor(6, 0);
				snprintf(buffer, 7, "H:%d%%      ", sensorData.sensorValue);
				lcd->print(buffer);

			}
			else {
				lcd->setCursor(0, 1);
				co2_level = sensorData.sensorValue;
				snprintf(buffer, 17, "CO2:%d|%dppm         ", sensorData.sensorValue, co2_target);
				lcd->print(buffer);
				//lcd->setCursor(12, 1);
				//sprintf(buffer, " %d", co2_target);
				//lcd->print(buffer);
			}
		}
		if (xQueueThatContainsData == ISRQueue) {
			//receive value from isr queue
			if(xQueueReceive(xQueueThatContainsData, &isr_val, 0) == pdPASS){
				//"filtering" multiple isrs from single turn/button press: wait a bit and discard all interrupts in queue
				vTaskDelay(100);
				xQueueReset(ISRQueue);
				//value from isr is 0: button press
				if(isr_val == 0){
					//toggle increase/decrease value
					if (val_mult == 20) {
						val_mult = 1;
					}
					else {
						val_mult = 20;
					}
					//increment count for manual relay open functionality
					if (!manual_relay) {// and !relay.read()) {
						manual_relay_count++;
					}
					//start timer for button hold for reset functionality
					//xTimerStart(manual_reset_timer, portMAX_DELAY);
				}
				//value from isr is 1 or -1: rotary encoder
				else {
					//add value co2 target
					co2_target += (isr_val * val_mult);
					//lower limit
					if (co2_target < 0) {
						co2_target = 0;
					}
					//reset manual relay count
					manual_relay_count = 0;
					//write co2 target to eeprom
					vTaskSuspendAll();
					Chip_EEPROM_Write(EEPROM_ADDRESS, (uint8_t *)&co2_target, sizeof(uint32_t));
					xTaskResumeAll();
				}
			};
			//NVIC_EnableIRQ(PIN_INT0_IRQn);

		}

		//3 button presses -> manual relay open
		if ((manual_relay_count > 2) and (co2_target > co2_level)) {// and relay_enable) {
			manual_relay = true;
			manual_relay_count = 0;
			relay_count = 9;
			xTimerStart(manual_relay_timer, 100);
		}

		//write relay
		if (relay_enable or manual_relay) {
			//if measured co2 is lower than co2 target -> open valve
			//bool started: vaoid starting the timer multiple times
			if((co2_level < co2_target) and !started){
				//start timer: disable relay after 3s
				xTimerStart(relay_3s_on_timer, portMAX_DELAY);
				//open relay
				relay.write(1);
				started = true;
			}
			//if manual relay functionality -> open valve
			else if (manual_relay) {
				relay.write(1);
			}
		}
		//else close relay
		else {//if (!relay_enable) {
			relay_count = 0;
			started = false;
			relay.write(0);
		}
		//else if (co2_level > co2_target) {
		//	relay.write(0);
		//}

		//if(counter > 30){
		//	lcd->clear();
		//	counter = 0;
		//}
	}
	vTaskDelay(xBlockTime);
}


/* WiFi and Mqtt thread */
static void vMqttFormat(void* pvParamenters) {
	//SemaphoreHandle_t xBinarySemaphore;
	mqttData mD;
	mqttStrData mSD;

	const TickType_t xBlockTime = pdMS_TO_TICKS(100);
	char mqttStr[60];
	char strBackUp[60];

	while (1) {
		xQueueReceive(mqttQueue, &mD, 0);

		if (mD.DataType == co2) {
			mSD.co2 = mD.value;
		}
		else if (mD.DataType == temperature) {
			mSD.temperature = mD.value;
		}
		else if (mD.DataType == relativehumidity) {
			mSD.relativehumidity = mD.value;
		}
		else if (mD.DataType == valveOpenPerc) {
			mSD.valveOpenPerc = mD.value;
		}
		else if (mD.DataType == co2SetPoint) {
			mSD.co2SetPoint = mD.value;
		}

		snprintf(mqttStr, 60, "field1=%d&field2=%d&field3=%d&field4=%d&field5=%d", mSD.co2, mSD.relativehumidity, mSD.temperature, mSD.valveOpenPerc, mSD.co2SetPoint);

		if (mqttStr != NULL) {
			xQueueReceive(strQueue, &strBackUp, 0);
			xQueueSendToBack(strQueue, &mqttStr, portMAX_DELAY);
			xSemaphoreGive(xMutex);
		}
	}
	vTaskDelay(xBlockTime);
}

static void vMqttPublish(void* pvParamenters) {
	char publishStr[60];
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	xQueueReceive(strQueue, &publishStr, 0);
	vStartSimpleMQTTDemo(publishStr);

	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300000));
}


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

	manual_relay_timer = xTimerCreate("manual_relay_timer",
						 pdMS_TO_TICKS(1000),
						 pdTRUE,
						 (void*) 0,
						 vTimerCallbackRelay);

	manual_reset_timer = xTimerCreate("manual_reset_timer",
					     pdMS_TO_TICKS(1000),
					     pdTRUE,
					     (void*) 0,
					     vTimerCallbackReset);

	relay_60s_limit = xTimerCreate("relay_60s_timer",
					  pdMS_TO_TICKS(60000),
					  pdFALSE,
					  (void*) 0,
					  vTimerCallback60s);

	relay_3s_on_timer = xTimerCreate("relay_3s_timer",
						pdMS_TO_TICKS(1000),
						pdFALSE,
						(void*) 0,
						vTimerCallback3s);

	heap_monitor_setup();
//	SysTick_Config(SystemCoreClock / TICKRATE_HZ); //not know if needed
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_EEPROM);
	Chip_SYSCTL_PeriphReset(RESET_EEPROM);

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

	retarget_init();


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



	xTaskCreate(vLcdDisplay, "LcdDisplay",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);

	//Create task for reading temperature and humidity sensors value
	xTaskCreate(vSensorReadTask, "sensorReadTask",
		configMINIMAL_STACK_SIZE * 5, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);
	
	xTaskCreate(vMqttFormat, "mqttFormatTask",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);

	xTaskCreate(vMqttPublish, "mqttPublishTask",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);


	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
