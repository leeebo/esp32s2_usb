/**
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * taskmonitor.c - System load monitor
 */



#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "taskmonitor.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick
#define TIMER_PERIOD M2T(1000)

const static char *TAG = "TASK_MONITOR";
static void timerHandler(xTimerHandle timer);
static bool initialized = false;
static uint8_t triggerDump = 1;
static xTimerHandle timer;

typedef struct {
    uint32_t ulRunTimeCounter;
    uint32_t xTaskNumber;
} taskData_t;

#define TASK_MAX_COUNT 32
static taskData_t previousSnapshot[TASK_MAX_COUNT];
static int taskTopIndex = 0;
static uint32_t previousTotalRunTime = 0;

static StaticTimer_t timerBuffer;

void taskMonitorInit()
{
	if (initialized) {
        return;
    }
    timer = xTimerCreateStatic("sysLoadMonitorTimer", TIMER_PERIOD, pdTRUE, NULL, timerHandler, &timerBuffer);
    initialized = true;
}

void taskMonitorStart(){
	if (!initialized) {
        return;
    }
	xTimerStart(timer, 100);
}

void taskMonitorStop(){
	if (!initialized) {
        return;
    }
	xTimerStop(timer, 100);
}


static taskData_t *getPreviousTaskData(uint32_t xTaskNumber)
{
    // Try to find the task in the list of tasks
    for (int i = 0; i < taskTopIndex; i++) {
        if (previousSnapshot[i].xTaskNumber == xTaskNumber) {
            return &previousSnapshot[i];
        }
    }
    // Allocate a new entry
    ESP_ERROR_CHECK(!(taskTopIndex < TASK_MAX_COUNT));
    taskData_t *result = &previousSnapshot[taskTopIndex];
    result->xTaskNumber = xTaskNumber;
    taskTopIndex++;
    return result;   
}

static void timerHandler(xTimerHandle timer)
{
    if (triggerDump != 0) {
        uint32_t totalRunTime;

        TaskStatus_t taskStats[TASK_MAX_COUNT];
        uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
        ESP_ERROR_CHECK(!(taskTopIndex < TASK_MAX_COUNT));
        uint32_t totalDelta = totalRunTime - previousTotalRunTime;
        float f = 100.0 / totalDelta;
        // Dumps the the CPU load and stack usage for all tasks
        // CPU usage is since last dump in % compared to total time spent in tasks. Note that time spent in interrupts will be included in measured time.
        // Stack usage is displayed as nr of unused bytes at peak stack usage.

        ESP_LOGI(TAG,"Task dump\n");
        ESP_LOGI(TAG,"Load\tStack left\tName\tPRI\n");

        for (uint32_t i = 0; i < taskCount; i++) {
            TaskStatus_t *stats = &taskStats[i];
            taskData_t *previousTaskData = getPreviousTaskData(stats->xTaskNumber);

            uint32_t taskRunTime = stats->ulRunTimeCounter;
            float load = f * (taskRunTime - previousTaskData->ulRunTimeCounter);
            ESP_LOGI(TAG,"%.2f \t%u \t%s \t%u\n", load, stats->usStackHighWaterMark, stats->pcTaskName, stats->uxBasePriority);

            previousTaskData->ulRunTimeCounter = taskRunTime;
        }
		ESP_LOGI(TAG,"Free heap: %d\n", esp_get_free_heap_size());

        previousTotalRunTime = totalRunTime;

        //triggerDump = 0;
    }
}
