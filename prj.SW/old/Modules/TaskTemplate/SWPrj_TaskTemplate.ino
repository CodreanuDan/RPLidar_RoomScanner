
/* ============ GPIO ============ */
#define START_MEASUREMENT_BUTTON 18
#define STOP_MEASUREMENT_BUTTON  17 
#define MEASURE_OFF 27
#define MEASURE_ON 26

/* ======== Buttons config ======= */
struct Button {
    const uint8_t PIN;
    volatile bool pressed;
};

Button startButton{START_MEASUREMENT_BUTTON,false};
Button stopButton{STOP_MEASUREMENT_BUTTON,false};

#define BTN_DEBOUNCE_TIME 10000 
volatile unsigned long last_start_button_time = 0; 
volatile unsigned long last_stop_button_time = 0; 

/* =========== OS config ========== */

/* Task Size  */
#define TASK_SIZE_BYTES 8192u  

/* Task Handlers  */
TaskHandle_t Task_C0_Init_handle = NULL;
TaskHandle_t Task_C0_handle = NULL;
TaskHandle_t Task_C1_Init_handle = NULL;
TaskHandle_t Task_C1_handle = NULL;

/* Semaphores  */
SemaphoreHandle_t xInitSemaphore_C0;
SemaphoreHandle_t xInitSemaphore_C1;

/* Cores  */
typedef enum {
    ESP_CORE_0 = 0u,
    ESP_CORE_1
} Cores;

/*  Task info struct */
typedef struct
{
    void (*taskFunction)(void*);
    const char* taskName;
    uint32_t stackSize;
    void* params;
    UBaseType_t priority;
    TaskHandle_t* handle;
    Cores core;
} TaskDefinition;

/* ========Global init flags ====== */
volatile bool init_flag_C0 = false;
volatile bool init_flag_C1 = false;

/* ==== MEASURETMENT CONSTANTS ==== */
#define MEASURE_COUNT     180    
#define PROCESS_COUNT     180    

/* ========= ISR Handlers -======== */
void IRAM_ATTR StartBtn_ISRHdl()
{
    volatile unsigned long now = micros();
    if (now - last_start_button_time > BTN_DEBOUNCE_TIME)
    {
        startButton.pressed = true;
        last_start_button_time = now;
        detachInterrupt(startButton.PIN);
        Serial.println("Start ISR triggered");
    }
}

void IRAM_ATTR StopBtn_ISRHdl()
{
    volatile unsigned long now = micros();
    if (now - last_stop_button_time > BTN_DEBOUNCE_TIME)
    {
        stopButton.pressed = true;
        last_stop_button_time = now;
        detachInterrupt(stopButton.PIN);
        Serial.println("Stop ISR triggered");
    }
}

/* = Task & OS Function Prototypes =*/
void Task_C0_Init(void *pvParameters);
void Task_C0(void *pvParameters);
void Task_C1_Init(void *pvParameters);
void Task_C1(void *pvParameters);

/* ============= Tasks list ======== */
TaskDefinition taskList[] = {
    /* Core 0 timed tasks */
    { Task_C0_Init,        "Task_C0_Init",        TASK_SIZE_BYTES, NULL, 20, &Task_C0_Init_handle,        ESP_CORE_0 },
    { Task_C0,             "Task_C0",             TASK_SIZE_BYTES, NULL, 10, &Task_C0_handle,             ESP_CORE_0 },
    /* Core 1 timed tasks */
    { Task_C1_Init,        "Task_C1_Init",        TASK_SIZE_BYTES, NULL, 20, &Task_C1_Init_handle,        ESP_CORE_1 },
    /* Event driven tasks */
    { Task_C1,             "Task_C1",             TASK_SIZE_BYTES, NULL, 10, &Task_C1_handle,             ESP_CORE_1}
};

void OS_CreateTask(void (*taskInterface)(void *), const char* taskName, uint32_t stackSize,
                   void* taskParams, UBaseType_t priority, TaskHandle_t* taskHandler, Cores core);
void OS_ValidateTaskCreation(BaseType_t xReturned, const char* taskName);
void OS_InitOS();

/* ===== Function prototypes ======= */
void SetMeasurementState(bool on);


void setup() 
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(startButton.PIN, INPUT_PULLUP);
    attachInterrupt(startButton.PIN, StartBtn_ISRHdl, FALLING);

    pinMode(stopButton.PIN, INPUT_PULLUP);
    attachInterrupt(stopButton.PIN, StopBtn_ISRHdl, FALLING);

    pinMode(MEASURE_OFF, OUTPUT);
    pinMode(MEASURE_ON, OUTPUT);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    SetMeasurementState(false);

    OS_InitOS();
}

void loop() {}


/* ----- Task interfaces ----- */
void Task_C0_Init(void *pvParameters)
{
    /* Placeholder */
    Serial.println("Task_C0_Init");

    /* Init flag */
    init_flag_C0 = true;

    /* After Init suspend task and release semaphore for Core 0 tasks */
    if (init_flag_C0)
    {
        if (xInitSemaphore_C0)
        {    
            xSemaphoreGive(xInitSemaphore_C0);
        }

        vTaskSuspend(NULL);
    }
}

void Task_C0(void *pvParameters)
{
	for(;;)
	{
		if (startButton.pressed == true)
		{
			/* Signal that measurement has begun */
			startButton.pressed = false;
			SetMeasurementState(true);
			Serial.print("Task_C0: Measurement STARTED!");

			/* Take samples */
			uint8_t measurementCount = 0;

			while (measurementCount < MEASURE_COUNT)
			{
				/* Stop measurement and start data analysis task if stop button pressed*/
				if (stopButton.pressed == true)
				{
					stopButton.pressed = false;
					SetMeasurementState(false);
					Serial.print("Task_C0: Measurement STOPPED by button!");
					break;  
				}

				/* Aici simulam masuratorile: pornim LIDAR, masuram dupa care oprim stocam datele undeva, miscam motorul masuram 
				pe IMU rotatia dupa care reluam procesul */
				if (measurementCount == 0)
				  Serial.printf("Task_C0: Measuring sample %u/180\n", measurementCount + 1);
				if (measurementCount == MEASURE_COUNT - 1)
				  Serial.printf("Task_C0: Measuring sample %u/180\n", measurementCount + 1);
				measurementCount++;
				vTaskDelay(50 / portTICK_PERIOD_MS); 
			}

			/* Stop measurement and start data analysis task */
			SetMeasurementState(false);
			Serial.println("Task_C0: Measurement complete — triggering processing, suspend task...");

			xTaskNotifyGive(Task_C1_handle);

		}
		
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void Task_C1_Init(void *pvParameters)
{
    /* Placeholder */
    Serial.println("Task_C1_Init");

    /* Init flag */
    init_flag_C1 = true;

    /* After Init suspend task and release semaphore for Core 1 tasks */
    if (init_flag_C1)
    {
        vTaskSuspend(NULL);
    }

}

void Task_C1(void *pvParameters)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
       
		Serial.println("Task_C1: Data processing started");
		uint8_t processCount = 0;

		/* Aici o sa prelucram noi datele sau ce naiba om face cu ele */
		while (processCount < PROCESS_COUNT)
		{
			if (processCount == 0)
			  Serial.printf("Task_C1: Processing sample %u/180\n", processCount + 1);
			if (processCount == PROCESS_COUNT - 1)
			  Serial.printf("Task_C1: Processing sample %u/180\n", processCount + 1);
			processCount++;
			vTaskDelay(30 / portTICK_PERIOD_MS);
		}
		Serial.println("Task_C1: Data processing complete");
    }
}

/* ----- OS functions ----- */


/*
 *  OS_CreateTask: Wrapper to xTaskCreatePinnedToCore
 */
void OS_CreateTask(void (*taskInterface)(void *),
                   const char* taskName,
                   uint32_t stackSize,
                   void* taskParams,
                   UBaseType_t priority,
                   TaskHandle_t* taskHandler,
                   Cores core)
{
    /* Set initial flag to false */
    BaseType_t xReturned = pdFAIL;

    /* Create task */
    xReturned = xTaskCreatePinnedToCore(
          taskInterface,
          taskName,
          stackSize,
          taskParams,
          priority,
          taskHandler,
          static_cast<BaseType_t>(core)
    );

    /* Task created successfully */
    if (xReturned != pdPASS) 
    {
        Serial.print("ERROR: Failed to create task: ");Serial.println(taskName);
        while(1); 
    }

}

/*
 *  OS_InitOS: Creates task semaphores and tasks
 */
void OS_InitOS()
{
      /* Create semaphore for core 0 */
      xInitSemaphore_C0 = xSemaphoreCreateBinary();
      if (xInitSemaphore_C0 == NULL)
      {
          Serial.println("ERROR: xInitSemaphore_C0 failed to create");
          while(1);
      }

      /* Create semaphore for core 1 */
      xInitSemaphore_C1 = xSemaphoreCreateBinary();
      if (xInitSemaphore_C1 == NULL)
      {
          Serial.println("ERROR: xInitSemaphore_C1 failed to create");
          while(1);
      }

      /* Create tasks */
      const uint8_t numTasks = sizeof(taskList) / sizeof(taskList[0]);
      for (uint8_t i = 0; i < numTasks; ++i)
      {
          OS_CreateTask(taskList[i].taskFunction,
                        taskList[i].taskName,
                        taskList[i].stackSize,
                        taskList[i].params,
                        taskList[i].priority,
                        taskList[i].handle,
                        taskList[i].core);
      }

}


/* ----- Helpers ----- */

/*
 *  SetMeasurementState: Signals strat/stop of measurement
 */
void SetMeasurementState(bool on)
{
    digitalWrite(MEASURE_ON, on ? HIGH : LOW);
    digitalWrite(MEASURE_OFF, on ? LOW : HIGH);

    /* Detach both interrupts first */
    detachInterrupt(startButton.PIN);
    detachInterrupt(stopButton.PIN);
	  delayMicroseconds(2000);

    /* Re-attach interrupts */
    if (on)
    {
        attachInterrupt(stopButton.PIN, StopBtn_ISRHdl, FALLING);
        Serial.println("SetMeasurementState: Attached STOP ISR");
    }
    else
    {
        attachInterrupt(startButton.PIN, StartBtn_ISRHdl, FALLING);
        Serial.println("SetMeasurementState: Attached START ISR");
    }
}