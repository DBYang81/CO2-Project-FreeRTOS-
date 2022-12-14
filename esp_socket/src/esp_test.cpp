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
#include "mqtt_demo/demo_config.h"
#include "core_mqtt.h"
#include "mqtt_demo/using_plaintext.h"
#include "backoff_algorithm.h"



#define QUEUE_LENGTH1 10
#define QUEUE_LENGTH2 10
#define QUEUE_LENGTH3 10
#define QUEUE_LENGTH4 1
#define BINARY_SEMAPHORE_LENGTH 1
#define COMBINE_LENGTH (QUEUE_LENGTH1 + QUEUE_LENGTH2 + BINARY_SEMAPHORE_LENGTH)

#define CHANNEL_ID "1955513"
#define SECRET_MQTT_USERNAME "DCAmDzgFFhoKKy8kCBw3NQA"
#define SECRET_MQTT_CLIENT_ID "DCAmDzgFFhoKKy8kCBw3NQA"
#define SECRET_MQTT_PASSWORD "qKxqzEzD+xMf2LMg0SU24WYk"



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

// TODO: insert other definitions and declarations here

//ISR for rotary encoder turn (siga only)
extern "C" {
	void vStartSimpleMQTTDemo(char* string); // ugly - should be in a header
}
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

/* Temp&humidity sensor reading by modus - high priority */
static void vTempHumidity(void* pvParameters) {
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
	relay.write(0);

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
		Data TEValue = { TE.read() / 10,temperatureSensor };
		vTaskDelay(5);
		Data RHValue = { RH.read() / 10,humiditySensor };
		vTaskDelay(5);

		mqttData Co2 = { CO2.read(),co2 };
		vTaskDelay(5);
		mqttData te = { TE.read() / 10,temperature };
		vTaskDelay(5);
		mqttData rh = { RH.read() / 10,relativehumidity };
		vTaskDelay(5);

		//xSemaphoreGive(xMutex);
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
	int Co2cnter = 0;
	char buffer[50];
	int count = 0;
	while (1) {
		count++;
		if (count > 30) {
			lcd->clear();
			count = 0;
		}
		xQueueThatContainsData = (QueueHandle_t)xQueueSelectFromSet(xQueueSet, portMAX_DELAY);
		lcd->setCursor(14, 0);
		DigitalIoPin relay(0, 27, DigitalIoPin::output); // CO2 relay
		sprintf(buffer, "R%d", relay.read());
		lcd->print(buffer);
		if (xQueueThatContainsData == sensorQueue) {
			xQueueReceive(xQueueThatContainsData, &sensorData, 0);
			// set the cursor to column 0, line 1
			// (note: line 1 is the second row, since counting begins with 0):

			if (sensorData.sensor == temperatureSensor) {
				lcd->setCursor(0, 0);
				lcd->print("T: ");
				lcd->setCursor(4, 0);
				sprintf(buffer, "%d", sensorData.sensorValue);
				lcd->print(buffer);


			}
			else if (sensorData.sensor == humiditySensor) {
				lcd->setCursor(8, 0);
				lcd->print("H: ");
				sprintf(buffer, "%d", sensorData.sensorValue);
				lcd->print(buffer);

			}
			else {
				lcd->setCursor(0, 1);
				lcd->print("CO2: ");
				lcd->setCursor(4, 1);
				sprintf(buffer, "%d", sensorData.sensorValue);
				lcd->print(buffer);
			}
		}
		if (xQueueThatContainsData == ISRQueue) {
			xQueueReceive(xQueueThatContainsData, &Co2cnter, 0);
			lcd->setCursor(10, 1);
			sprintf(buffer, "%d", Co2cnter);
			lcd->print(buffer);
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
	mqttQueue = xQueueCreate(QUEUE_LENGTH3, sizeof(mqttData));
	strQueue = xQueueCreate(QUEUE_LENGTH4, sizeof(char) * 60);
	xSemaphore = xSemaphoreCreateBinary();
	xMutex = xSemaphoreCreateMutex();

	xQueueAddToSet(sensorQueue, xQueueSet);
	xQueueAddToSet(ISRQueue, xQueueSet);
	xQueueAddToSet(xSemaphore, xQueueSet);

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
	//Chip_PININT_DisableIntHigh(LPC_GPIO_PIN_INT, PININTCH(1) | PININTCH(0));

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
	xTaskCreate(vTempHumidity, "tempHumidityTask",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
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