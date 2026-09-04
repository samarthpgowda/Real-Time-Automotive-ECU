/*
 * ================================================================
 * Real-Time Automotive ECU Simulator
 *
 * Author  : Samarth P
 * MCU     : STM32F411RE
 * RTOS    : FreeRTOS / CMSIS-RTOS2
 * UART    : USART2 - 115200 Baud
 *
 * RTOS Concepts Implemented:
 *   - Tasks
 *   - Message Queues
 *   - Event Flags
 *   - Semaphore
 *   - Mutex
 *   - Software Timer
 *   - GPIO Interrupt
 *   - UART Communication
 *
 * Tasks:
 *   EngineTask
 *   SpeedTask
 *   DashboardTask
 *   DiagnosticTask
 *   EmergencyTask
 *   FuelTask
 *
 * ================================================================
 */

#include "main.h"
#include "cmsis_os.h"

#include <string.h>

/* ========================= Data Types ========================= */

typedef struct
{
    uint16_t rpm;
    uint8_t temperature;
    uint8_t engineState;
} EngineData;

typedef struct
{
    uint16_t speed;
} SpeedData;

typedef enum
{
    ECU_NORMAL = 0,
    ECU_EMERGENCY
} ECUState;

typedef struct
{
    uint8_t fuel;
} FuelData;

/* ========================= Event Flags ======================== */

#define EVENT_ENGINE_READY     (1U << 0)
#define EVENT_VEHICLE_MOVING   (1U << 1)
#define EVENT_LOW_FUEL         (1U << 2)
#define EVENT_OVER_TEMP        (1U << 3)
#define EVENT_EMERGENCY        (1U << 4)

/* ========================= RTOS Objects ======================= */

UART_HandleTypeDef huart2;

osEventFlagsId_t ecuEventHandle;

osMessageQueueId_t engineQueueHandle;
osMessageQueueId_t speedQueueHandle;
osMessageQueueId_t fuelQueueHandle;

osSemaphoreId_t emergencySemaphoreHandle;

osMutexId_t uartMutexHandle;

osTimerId_t diagnosticTimerHandle;

/* ========================= Task Handles ======================== */

osThreadId_t engineTaskHandle;
osThreadId_t speedTaskHandle;
osThreadId_t dashboardTaskHandle;
osThreadId_t diagnosticTaskHandle;
osThreadId_t emergencyTaskHandle;
osThreadId_t fuelTaskHandle;

/* ========================= Task Attributes ===================== */

const osThreadAttr_t engineTask_attributes =
{
    .name = "EngineTask",
    .priority = (osPriority_t)osPriorityAboveNormal,
    .stack_size = 512
};

const osThreadAttr_t speedTask_attributes =
{
    .name = "SpeedTask",
    .priority = (osPriority_t)osPriorityAboveNormal,
    .stack_size = 512
};

const osThreadAttr_t dashboardTask_attributes =
{
    .name = "DashboardTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 1024
};

const osThreadAttr_t diagnosticTask_attributes =
{
    .name = "DiagnosticTask",
    .priority = (osPriority_t)osPriorityBelowNormal,
    .stack_size = 512
};

const osThreadAttr_t emergencyTask_attributes =
{
    .name = "EmergencyTask",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 512
};

const osThreadAttr_t fuelTask_attributes =
{
    .name = "FuelTask",
    .priority = (osPriority_t)osPriorityBelowNormal,
    .stack_size = 512
};

/* ========================= Mutex Attributes =================== */

const osMutexAttr_t uartMutex_attributes =
{
    .name = "UartMutex",
    .attr_bits = osMutexPrioInherit
};

/* ========================= ECU State =========================== */

volatile ECUState ecuState = ECU_NORMAL;

/* ========================= Function Prototypes ================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void StartDefaultTask(void *argument);

void EngineTask(void *argument);
void SpeedTask(void *argument);
void DashboardTask(void *argument);
void DiagnosticTask(void *argument);
void EmergencyTask(void *argument);
void FuelTask(void *argument);

void UART_Print(char *message);

void DiagnosticTimerCallback(void *argument);

/* ========================= Main ================================ */

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();

    osKernelInitialize();

    uartMutexHandle = osMutexNew(&uartMutex_attributes);

    emergencySemaphoreHandle = osSemaphoreNew(1, 0, NULL);

    ecuEventHandle = osEventFlagsNew(NULL);

    engineQueueHandle = osMessageQueueNew(
        10,
        sizeof(EngineData),
        NULL
    );

    speedQueueHandle = osMessageQueueNew(
        10,
        sizeof(SpeedData),
        NULL
    );

    fuelQueueHandle = osMessageQueueNew(
        5,
        sizeof(FuelData),
        NULL
    );

    engineTaskHandle =
        osThreadNew(
            EngineTask,
            NULL,
            &engineTask_attributes
        );

    speedTaskHandle =
        osThreadNew(
            SpeedTask,
            NULL,
            &speedTask_attributes
        );

    dashboardTaskHandle =
        osThreadNew(
            DashboardTask,
            NULL,
            &dashboardTask_attributes
        );

    diagnosticTaskHandle =
        osThreadNew(
            DiagnosticTask,
            NULL,
            &diagnosticTask_attributes
        );

    emergencyTaskHandle =
        osThreadNew(
            EmergencyTask,
            NULL,
            &emergencyTask_attributes
        );

    diagnosticTimerHandle =
        osTimerNew(
            DiagnosticTimerCallback,
            osTimerPeriodic,
            NULL,
            NULL
        );

    osTimerStart(
        diagnosticTimerHandle,
        500
    );

    fuelTaskHandle =
        osThreadNew(
            FuelTask,
            NULL,
            &fuelTask_attributes
        );

    osKernelStart();

    while (1)
    {
    }
}

/* ========================= System Clock ======================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================= USART2 ============================== */

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================= GPIO ================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(
        ld2_GPIO_Port,
        ld2_Pin,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOC,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin = ld2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        ld2_GPIO_Port,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );

    HAL_NVIC_SetPriority(
        EXTI1_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI1_IRQn
    );

    HAL_NVIC_SetPriority(
        EXTI15_10_IRQn,
        5,
        0
    );

    HAL_NVIC_EnableIRQ(
        EXTI15_10_IRQn
    );
}

/* ========================= Default Task ========================= */

void StartDefaultTask(void *argument)
{
    for (;;)
    {
        osDelay(1);
    }
}

/* ========================= Diagnostic Timer ==================== */

void DiagnosticTimerCallback(void *argument)
{
    if (ecuState == ECU_EMERGENCY)
    {
        osEventFlagsSet(
            ecuEventHandle,
            EVENT_EMERGENCY
        );
    }
}

/* ========================= Engine Task ========================= */

void EngineTask(void *argument)
{
    EngineData engine;
    osStatus_t putStatus;

    engine.rpm = 1000;
    engine.temperature = 75;
    engine.engineState = 1;

    while (1)
    {
        if (ecuState == ECU_EMERGENCY)
        {
            engine.engineState = 0;
            engine.rpm = 0;
        }
        else
        {
            engine.rpm += 100;

            if (engine.rpm > 4000)
            {
                engine.rpm = 1000;
            }

            engine.temperature++;

            if (engine.temperature > 100)
            {
                osEventFlagsSet(
                    ecuEventHandle,
                    EVENT_OVER_TEMP
                );

                engine.temperature = 75;
            }

            engine.engineState = 1;

            if (engine.engineState == 1)
            {
                osEventFlagsSet(
                    ecuEventHandle,
                    EVENT_ENGINE_READY
                );
            }
        }

        putStatus = osMessageQueuePut(
            engineQueueHandle,
            &engine,
            0,
            10
        );

        if (putStatus != osOK)
        {
            UART_Print(
                "EngineTask: queue put failed\r\n"
            );
        }

        osDelay(100);
    }
}

/* ========================= Speed Task ========================== */

void SpeedTask(void *argument)
{
    SpeedData speed;
    osStatus_t putStatus;

    speed.speed = 0;

    while (1)
    {
        speed.speed += 5;

        if (speed.speed > 120)
        {
            speed.speed = 0;
        }

        putStatus = osMessageQueuePut(
            speedQueueHandle,
            &speed,
            0,
            10
        );

        if (putStatus != osOK)
        {
            UART_Print(
                "SpeedTask: queue put failed\r\n"
            );
        }

        if (speed.speed > 0)
        {
            osEventFlagsSet(
                ecuEventHandle,
                EVENT_VEHICLE_MOVING
            );
        }

        osDelay(200);
    }
}

/* ========================= Dashboard Task ====================== */

void DashboardTask(void *argument)
{
    EngineData engine;
    SpeedData speed;

    char buffer[150];

    while (1)
    {
        UART_Print(
            "Dashboard waiting for ENGINE\r\n"
        );

        if (osMessageQueueGet(
                engineQueueHandle,
                &engine,
                NULL,
                osWaitForever
            ) == osOK)
        {
            UART_Print(
                "Dashboard got ENGINE\r\n"
            );

            UART_Print(
                "Dashboard waiting for SPEED\r\n"
            );

            if (osMessageQueueGet(
                    speedQueueHandle,
                    &speed,
                    NULL,
                    osWaitForever
                ) == osOK)
            {
                UART_Print(
                    "Dashboard got SPEED\r\n"
                );

                sprintf(
                    buffer,
                    "[DASHBOARD] RPM: %u | SPEED: %u km/h | TEMP: %u C | ENGINE: %s\r\n",
                    engine.rpm,
                    speed.speed,
                    engine.temperature,
                    engine.engineState
                        ? "ON"
                        : "OFF"
                );

                UART_Print(buffer);
            }
        }
    }
}

/* ========================= Diagnostic Task ===================== */

void DiagnosticTask(void *argument)
{
    uint32_t events;

    while (1)
    {
        events = osEventFlagsWait(
            ecuEventHandle,
            EVENT_LOW_FUEL |
            EVENT_OVER_TEMP |
            EVENT_EMERGENCY,
            osFlagsWaitAny,
            osWaitForever
        );

        if (events & EVENT_LOW_FUEL)
        {
            UART_Print(
                "[DIAG] WARNING: LOW FUEL\r\n"
            );
        }

        if (events & EVENT_OVER_TEMP)
        {
            UART_Print(
                "[DIAG] WARNING: ENGINE OVER TEMPERATURE\r\n"
            );
        }

        if (events & EVENT_EMERGENCY)
        {
            UART_Print(
                "[DIAG] CRITICAL: EMERGENCY STATE\r\n"
            );
        }
    }
}

/* ========================= Emergency Task ====================== */

void EmergencyTask(void *argument)
{
    while (1)
    {
        if (osSemaphoreAcquire(
                emergencySemaphoreHandle,
                osWaitForever
            ) == osOK)
        {
            UART_Print(
                "\r\n!!! EMERGENCY BUTTON PRESSED !!!\r\n"
            );

            UART_Print(
                "ECU entering emergency state\r\n"
            );

            ecuState = ECU_EMERGENCY;

            HAL_GPIO_WritePin(
                ld2_GPIO_Port,
                ld2_Pin,
                GPIO_PIN_SET
            );
        }
    }
}

/* ========================= Fuel Task =========================== */

void FuelTask(void *argument)
{
    FuelData fuel;
    osStatus_t putStatus;

    fuel.fuel = 100;

    while (1)
    {
        if (fuel.fuel > 0)
        {
            fuel.fuel--;
        }

        putStatus = osMessageQueuePut(
            fuelQueueHandle,
            &fuel,
            0,
            10
        );

        if (putStatus != osOK)
        {
            UART_Print(
                "FuelTask: queue put failed\r\n"
            );
        }

        if (fuel.fuel < 20)
        {
            osEventFlagsSet(
                ecuEventHandle,
                EVENT_LOW_FUEL
            );
        }

        osDelay(1000);
    }
}

/* ========================= GPIO Interrupt ====================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        osSemaphoreRelease(
            emergencySemaphoreHandle
        );
    }
}

/* ========================= UART Function ======================= */

void UART_Print(char *message)
{
    osMutexAcquire(
        uartMutexHandle,
        osWaitForever
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)message,
        strlen(message),
        HAL_MAX_DELAY
    );

    osMutexRelease(
        uartMutexHandle
    );
}

/* ========================= Error Handler ======================= */

void Error_Handler(void)
{
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
}

#endif