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
	uin16_t sensorValue;
	Sensors sensor;
}Data;

static QueueHandle_t sensorQueue = NULL, ISRQueue = NULL;
static QueueSetHandle_t xQueueSet = NULL;
QueueHandle_t xSemaphore;

// TODO: insert other definitions and declarations here

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

	DigitalIoPin* rs = new DigitalIoPin(0, 29, DigitalIoPin::output);
	DigitalIoPin* en = new DigitalIoPin(0, 9, DigitalIoPin::output);
	DigitalIoPin* d4 = new DigitalIoPin(0, 10, DigitalIoPin::output);
	DigitalIoPin* d5 = new DigitalIoPin(0, 16, DigitalIoPin::output);
	DigitalIoPin* d6 = new DigitalIoPin(1, 3, DigitalIoPin::output);
	DigitalIoPin* d7 = new DigitalIoPin(0, 0, DigitalIoPin::output);
	LiquidCrystal* lcd = new LiquidCrystal(rs, en, d4, d5, d6, d7);
	// configure display geometry
	lcd->begin(16, 2);
	// set the cursor to column 0, line 1
	// (note: line 1 is the second row, since counting begins with 0):
	lcd->setCursor(0, 0);
	// Print a message to the LCD.
	lcd->print("MQTT_FreeRTOS");


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

/* Temp&humidity sensor reading by modus - high priority */
static void vTempHumidity(void* pvParameters) {

	BaseType_t xStatus;
	const TickType_t xTicksToWait = pdMS_TO_TICKS(100);

	//Temperature sensor
	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister TE(&node3, 257, true);
	TE.read() / 10;

	//Humidity sensor
	ModbusMaster node3(241); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister RH(&node3, 256, true);
	RH.read() / 10;
	for (;; )
	{
		xStatus = xQueueSendToBack(sensorQueue, { TE.read() / 10,temperatureSensor }, xTicksToWait);
		if (xStatus != pdPASS)
		{
			vPrintString("Could not send to the queue.\r\n");
		}
	}

}
}

/* C02 sensor reading by modus controlled by relay - high priority*/
static void vC02Detected(void* pvParameters) {
	ModbusMaster node3(240); // Create modbus object that connects to slave id 241 (HMP60)
	node3.begin(9600); // all nodes must operate at the same speed!
	node3.idle(idle_delay); // idle function is called while waiting for reply from slave
	ModbusRegister CO2(&node3, 256, true);
	CO2.read();
}

/* LCD display thread - medium priority */
static void vLcdDisplay(void* pvParameters) {
}

/* Uart output thread - low priority */
static void vUARTTask(void* pvParameters) {
}

/* Rotary encoder thread - GPIO interrupt*/
static void vRotaryEncoder(void* pvParameters) {
}

/* WiFi and Mqtt thread */
static void vMqtt(void* pvParamenters) {
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
	ISRQueue = xQueueCreate(QUEUE_LENGTH12, sizeof(int));
	xSemaphore = xSemaphoreCreateBinary();

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

	xTaskCreate(task1, "test",
		configMINIMAL_STACK_SIZE * 4, NULL, (tskIDLE_PRIORITY + 1UL),
		(TaskHandle_t*)NULL);

	vStartSimpleMQTTDemo();
	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}