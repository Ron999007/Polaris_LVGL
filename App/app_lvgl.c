/**
 * @file app_lvgl.c
 * @brief FreeRTOS LVGL task with 1-second non-intrusive diagnostic UART telemetry.
 */

#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "drv_gt9147.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "lv_demo_stress.h"
#include "lv_demo_widgets.h"
#include "lv_demo_benchmark.h"
#include "app_lvgl.h"

/* External SDRAM ready semaphore */
extern osSemaphoreId_t SdramReadySemHandle;

/* Shared diagnostic profiling structure defined in lv_port_disp_template.c */
typedef struct {
    volatile uint32_t flush_calls;
    volatile uint32_t frame_complete_cnt;
    volatile uint32_t rot_us_sum;
    volatile uint32_t dcache_us_sum;
    volatile uint32_t dma2d_wait_us_sum;
    volatile uint32_t dma2d_irq_cnt;
    volatile uint32_t dma2d_timeout_cnt;
    volatile uint32_t dma2d_err_cnt;
    volatile uint32_t indev_calls;
    volatile uint32_t indev_i2c_polls;
    volatile uint32_t indev_us_sum;
    volatile uint32_t pixels_flushed;
} Polaris_LvglProf_t;

extern Polaris_LvglProf_t g_lvgl_prof;

/**
 * @brief Elevate LVGL task priority to osPriorityNormal and print hardware, cache,
 *        and function relocation addresses (verifying ITCM 0x0000xxxx and RAM_D2 0x3000xxxx).
 * @param None
 * @retval None
 */
static void App_LVGL_PrintBootDiagnostics(void)
{
    /* Elevate Task_LVGL from osPriorityLow (8) to osPriorityNormal (24) */
    (void)osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    uint32_t ccr = SCB->CCR;
    uint32_t icache_en = (ccr & SCB_CCR_IC_Msk) ? 1UL : 0UL;
    uint32_t dcache_en = (ccr & SCB_CCR_DC_Msk) ? 1UL : 0UL;
    uint32_t mpu_en    = (MPU->CTRL & MPU_CTRL_ENABLE_Msk) ? 1UL : 0UL;
    uint32_t qspi_cr   = QUADSPI->CR;
    uint32_t qspi_ccr  = QUADSPI->CCR;
    uint32_t qspi_prescaler = (qspi_cr >> 24U) & 0xFFU;

    printf("\r\n================ [BOOT_DIAG] ================\r\n");
    printf("SYSCLK:%lu MHz | HCLK:%lu MHz | ICache:%lu | DCache:%lu | MPU:%lu\r\n",
           SystemCoreClock / 1000000UL,
           HAL_RCC_GetHCLKFreq() / 1000000UL,
           icache_en, dcache_en, mpu_en);
    printf("QSPI_CR:0x%08lX (Prescaler:%lu) | QSPI_CCR:0x%08lX | VTOR:0x%08lX\r\n",
           qspi_cr, qspi_prescaler, qspi_ccr, SCB->VTOR);
    printf("Code Addr -> lv_refr_now:0x%08lX (ITCM) | lv_obj_create:0x%08lX (RAM_D2)\r\n",
           (uint32_t)(uintptr_t)&lv_refr_now,
           (uint32_t)(uintptr_t)&lv_obj_create);
    printf("LVGL Task Priority:%d | TickCust:%d | RefrPeriod:%d ms\r\n",
           (int)osThreadGetPriority(osThreadGetId()),
           (int)LV_TICK_CUSTOM,
           (int)LV_DISP_DEF_REFR_PERIOD);
    printf("=============================================\r\n\r\n");
}

/**
 * @brief FreeRTOS task entry function for LVGL GUI rendering and diagnostic logging.
 * @param pvParameters Pointer to task parameters (unused).
 * @retval None
 */
void vTask_LVGL(void *pvParameters)
{
    (void)pvParameters;

    osDelay(200);
    if (SdramReadySemHandle != NULL) {
        (void)osSemaphoreAcquire(SdramReadySemHandle, osWaitForever);
    }

    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    /* Print hardware configuration snapshot before starting the demo */
    App_LVGL_PrintBootDiagnostics();

    lv_demo_widgets();

    uint32_t report_tick_start = HAL_GetTick();
    uint32_t report_rtos_start = xTaskGetTickCount();
    uint32_t report_dwt_start  = DWT->CYCCNT;

    uint32_t loop_cnt          = 0U;
    uint32_t handler_us_sum    = 0U;
    uint32_t handler_tick_sum  = 0U;
    uint32_t sleep_req_sum     = 0U;

    for (;;)
    {
        uint32_t h_t0_tick = HAL_GetTick();
        uint32_t h_t0_dwt  = DWT->CYCCNT;

        /* Run LVGL core timer handler */
        uint32_t time_till_next = lv_timer_handler();

        uint32_t h_t1_dwt  = DWT->CYCCNT;
        uint32_t h_t1_tick = HAL_GetTick();

        loop_cnt++;
        handler_us_sum   += (h_t1_dwt - h_t0_dwt) / (SystemCoreClock / 1000000UL);
        handler_tick_sum += (h_t1_tick - h_t0_tick);

        /* Clamp sleep window between 5ms and 30ms */
        if (time_till_next < 5U) {
            time_till_next = 5U;
        } else if (time_till_next > 30U) {
            time_till_next = 30U;
        }
        sleep_req_sum += time_till_next;

        /* Print diagnostic breakdown once every 1000ms OUTSIDE lv_timer_handler() */
        uint32_t now_tick = HAL_GetTick();
        if ((now_tick - report_tick_start) >= 1000U) {
            uint32_t hal_elaps_ms  = now_tick - report_tick_start;
            uint32_t rtos_elaps_ms = xTaskGetTickCount() - report_rtos_start;
            uint32_t dwt_elaps_ms  = (DWT->CYCCNT - report_dwt_start) / (SystemCoreClock / 1000UL);

            uint32_t total_h_ms    = handler_us_sum / 1000U;
            uint32_t rot_ms        = g_lvgl_prof.rot_us_sum / 1000U;
            uint32_t dcache_ms     = g_lvgl_prof.dcache_us_sum / 1000U;
            uint32_t dma_wait_ms   = g_lvgl_prof.dma2d_wait_us_sum / 1000U;
            uint32_t indev_ms      = g_lvgl_prof.indev_us_sum / 1000U;
            uint32_t flush_total_ms = rot_ms + dcache_ms + dma_wait_ms;
            int32_t  sw_render_ms  = (int32_t)total_h_ms - (int32_t)flush_total_ms - (int32_t)indev_ms;
            if (sw_render_ms < 0) {
                sw_render_ms = 0;
            }

            printf("[LVGL_PROF] Tick(HAL:%lu,RTOS:%lu,DWT:%lums) | "
                   "Loops:%lu | FPS:%lu (Flushes:%lu,Px:%luK) | "
                   "Handler:%lums(Tick:%lums) [SW_Draw:%ldms, Rot:%lums, DCache:%lums, DMA_Wait:%lums, Touch:%lums(%lu/%lu)] | "
                   "DMA2D(IRQ:%lu,TO:%lu,Err:%lu) | LV_Idle:%u%%\r\n",
                   hal_elaps_ms, rtos_elaps_ms, dwt_elaps_ms,
                   loop_cnt,
                   g_lvgl_prof.frame_complete_cnt,
                   g_lvgl_prof.flush_calls,
                   g_lvgl_prof.pixels_flushed / 1000UL,
                   total_h_ms, handler_tick_sum,
                   (long)sw_render_ms, rot_ms, dcache_ms, dma_wait_ms,
                   indev_ms, g_lvgl_prof.indev_i2c_polls, g_lvgl_prof.indev_calls,
                   g_lvgl_prof.dma2d_irq_cnt, g_lvgl_prof.dma2d_timeout_cnt, g_lvgl_prof.dma2d_err_cnt,
                   (unsigned int)lv_timer_get_idle());

            /* Reset counters for the next 1-second window AFTER printing */
            memset((void *)&g_lvgl_prof, 0, sizeof(g_lvgl_prof));
            loop_cnt          = 0U;
            handler_us_sum    = 0U;
            handler_tick_sum  = 0U;
            sleep_req_sum     = 0U;
            report_tick_start = HAL_GetTick();
            report_rtos_start = xTaskGetTickCount();
            report_dwt_start  = DWT->CYCCNT;
        }

        osDelay(time_till_next);
    }
}
