/**
 * @file lv_port_disp_template.c
 * @brief LVGL v8.2 display port with DTCM draw buffer, dirty-area coalescing rounder,
 *        Pre-Wait Asynchronous DMA2D pipeline, and DWT hardware cycle profiling.
 */

#if 1

#if defined(__GNUC__)
#pragma GCC optimize ("O3")
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp_template.h"
#include "../../lvgl.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/*********************
 *      DEFINES
 *********************/
#define PHYSICAL_W          800U
#define PHYSICAL_H          480U
#define LOGICAL_W           480U
#define LOGICAL_H           800U
#define FB_BASE_ADDR        0xC0000000UL
#define DRAW_BUF_LINES      60U
#define DRAW_BUF_PIXELS     (LOGICAL_W * DRAW_BUF_LINES)
#define DMA2D_XFER_TIMEOUT  50U

/* Place LVGL draw buffer at the base of 128KB zero-wait-state DTCMRAM (57.6 KB used) */
#define DTCM_DRAW_BUF_ADDR  0x20000000UL

/**********************
 *      TYPEDEFS
 **********************/
/**
 * @brief Global diagnostic profiling structure for LVGL performance analysis.
 */
typedef struct {
    volatile uint32_t flush_calls;       /* Number of disp_flush() invocations per second */
    volatile uint32_t frame_complete_cnt;/* Number of full frames flushed (last chunk) per second */
    volatile uint32_t rot_us_sum;        /* Cumulative microseconds spent in 90-deg CPU rotation */
    volatile uint32_t dcache_us_sum;     /* Cumulative microseconds spent in D-Cache clean */
    volatile uint32_t dma2d_wait_us_sum; /* Cumulative microseconds spent waiting for DMA2D */
    volatile uint32_t dma2d_irq_cnt;     /* Number of times DMA2D ISR callback fired */
    volatile uint32_t dma2d_timeout_cnt; /* Number of times osSemaphoreAcquire timed out */
    volatile uint32_t dma2d_err_cnt;     /* Number of DMA2D transfer errors */
    volatile uint32_t indev_calls;       /* Number of touchpad_read() calls per second */
    volatile uint32_t indev_i2c_polls;   /* Number of actual GT9147 I2C transactions per second */
    volatile uint32_t indev_us_sum;      /* Cumulative microseconds spent in touchpad_read() */
    volatile uint32_t pixels_flushed;    /* Total pixels flushed per second */
} Polaris_LvglProf_t;

/**********************
 *  EXTERNAL & GLOBAL VARIABLES
 **********************/
extern osSemaphoreId_t DMA2DSemHandle;
extern DMA2D_HandleTypeDef hdma2d;

/* Global profiler instance shared with lv_port_indev_template.c and app_lvgl.c */
Polaris_LvglProf_t g_lvgl_prof = {0};

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
static void disp_rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area);
static void dma2d_transfer_complete_cb(DMA2D_HandleTypeDef * hdma2d_ptr);
static void dma2d_transfer_error_cb(DMA2D_HandleTypeDef * hdma2d_ptr);

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_disp_draw_buf_t draw_buf_dsc_1;

/* LVGL rendering buffer in zero-wait-state DTCMRAM (0x20000000) */
static lv_color_t * const buf_1 = (lv_color_t *)DTCM_DRAW_BUF_ADDR;

/* Intermediate rotation buffer in internal D1 AXI SRAM (0x24000000) for DMA2D source reads */
static uint16_t s_rot_buf[DRAW_BUF_PIXELS] __attribute__((aligned(32)));

/* Flag tracking whether a background DMA2D interrupt transfer is currently active */
static volatile uint8_t s_dma2d_in_flight = 0U;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initialize the LVGL display driver, DWT cycle counter, and DMA2D engine.
 * @param None
 * @retval None
 */
void lv_port_disp_init(void)
{
    disp_init();

    lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, DRAW_BUF_PIXELS);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res    = LOGICAL_W;
    disp_drv.ver_res    = LOGICAL_H;
    disp_drv.flush_cb   = disp_flush;
    disp_drv.rounder_cb = disp_rounder_cb;
    disp_drv.draw_buf   = &draw_buf_dsc_1;
    disp_drv.sw_rotate  = 0;

    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Enable Cortex-M7 DWT cycle counter and configure DMA2D for RGB565 M2M mode.
 * @param None
 * @retval None
 */
static void disp_init(void)
{
    /* Enable Cortex-M7 DWT->CYCCNT hardware cycle counter for microsecond profiling */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR          = 0xC5ACCE55UL;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_DMA2D_CLK_ENABLE();

    hdma2d.Instance          = DMA2D;
    hdma2d.Init.Mode         = DMA2D_M2M;
    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
    hdma2d.Init.OutputOffset = 0U;
    (void)HAL_DMA2D_Init(&hdma2d);

    hdma2d.LayerCfg[1].InputOffset       = 0U;
    hdma2d.LayerCfg[1].InputColorMode    = DMA2D_INPUT_RGB565;
    hdma2d.LayerCfg[1].AlphaMode         = DMA2D_NO_MODIF_ALPHA;
    hdma2d.LayerCfg[1].InputAlpha        = 0xFFU;
    hdma2d.LayerCfg[1].AlphaInverted     = DMA2D_REGULAR_ALPHA;
    hdma2d.LayerCfg[1].RedBlueSwap       = DMA2D_RB_REGULAR;
    hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
    (void)HAL_DMA2D_ConfigLayer(&hdma2d, 1U);

    hdma2d.XferCpltCallback  = dma2d_transfer_complete_cb;
    hdma2d.XferErrorCallback = dma2d_transfer_error_cb;

    HAL_NVIC_SetPriority(DMA2D_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);

    s_dma2d_in_flight = 0U;
}

/**
 * @brief Coalesce nearby fragmented dirty areas (such as concentric arcs and labels on the
 *        Analytics tab) by rounding coordinates to 32x32 pixel grid blocks.
 *        Reduces per-frame flush calls from 16 down to 2-3, eliminating redundant widget
 *        tree traversals and card shadow calculations.
 * @param disp_drv Pointer to the LVGL display driver descriptor.
 * @param area     Pointer to the dirty area coordinates to be rounded in-place.
 * @retval None
 */
static void disp_rounder_cb(lv_disp_drv_t * disp_drv, lv_area_t * area)
{
    (void)disp_drv;

    area->x1 = (lv_coord_t)((uint16_t)area->x1 & ~0x1FU);
    area->x2 = (lv_coord_t)(((uint16_t)area->x2 | 0x1FU));
    if (area->x2 >= (lv_coord_t)LOGICAL_W) {
        area->x2 = (lv_coord_t)(LOGICAL_W - 1U);
    }

    area->y1 = (lv_coord_t)((uint16_t)area->y1 & ~0x1FU);
    area->y2 = (lv_coord_t)(((uint16_t)area->y2 | 0x1FU));
    if (area->y2 >= (lv_coord_t)LOGICAL_H) {
        area->y2 = (lv_coord_t)(LOGICAL_H - 1U);
    }
}

/**
 * @brief DMA2D transfer complete ISR callback. Increments IRQ counter and releases semaphore.
 * @param hdma2d_ptr Pointer to the DMA2D handle structure.
 * @retval None
 */
static void dma2d_transfer_complete_cb(DMA2D_HandleTypeDef * hdma2d_ptr)
{
    (void)hdma2d_ptr;
    g_lvgl_prof.dma2d_irq_cnt++;
    s_dma2d_in_flight = 0U;

    if (DMA2DSemHandle != NULL) {
        (void)osSemaphoreRelease(DMA2DSemHandle);
    }
}

/**
 * @brief DMA2D transfer error ISR callback. Increments error counter and releases semaphore.
 * @param hdma2d_ptr Pointer to the DMA2D handle structure.
 * @retval None
 */
static void dma2d_transfer_error_cb(DMA2D_HandleTypeDef * hdma2d_ptr)
{
    (void)hdma2d_ptr;
    g_lvgl_prof.dma2d_err_cnt++;
    s_dma2d_in_flight = 0U;

    if (DMA2DSemHandle != NULL) {
        (void)osSemaphoreRelease(DMA2DSemHandle);
    }
}

/**
 * @brief Flush rendered portrait area to SDRAM using Pre-Wait Asynchronous DMA2D pipeline:
 *        - Step 0: Wait for the PREVIOUS DMA2D transfer to finish reading s_rot_buf.
 *        - Step 1: Rotate pixels 90-deg CCW from DTCMRAM (color_p) into AXI SRAM (s_rot_buf).
 *        - Step 2: Clean Cortex-M7 D-Cache for s_rot_buf.
 *        - Step 3: Start non-blocking DMA2D transfer from s_rot_buf to SDRAM (0xC0000000).
 *        - Step 4: Immediately call lv_disp_flush_ready() so CPU and DMA2D run in parallel.
 * @param disp_drv Pointer to the LVGL display driver descriptor.
 * @param area     Pointer to the logical screen area to be flushed.
 * @param color_p  Pointer to the rendered RGB565 pixel array in DTCMRAM.
 * @retval None
 */
__attribute__((optimize("O3")))
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    const uint32_t cycles_per_us = (SystemCoreClock / 1000000UL);
    const int32_t x1 = area->x1;
    const int32_t y1 = area->y1;
    const int32_t x2 = area->x2;
    const int32_t y2 = area->y2;

    const int32_t log_w = (x2 - x1 + 1);
    const int32_t log_h = (y2 - y1 + 1);

    const uint32_t phys_w = (uint32_t)log_h;
    const uint32_t phys_h = (uint32_t)log_w;

    const uint16_t * __restrict src_base = (const uint16_t *)color_p;
    uint16_t * __restrict rot_base = s_rot_buf;

    g_lvgl_prof.flush_calls++;
    g_lvgl_prof.pixels_flushed += (phys_w * phys_h);
    if (lv_disp_flush_is_last(disp_drv)) {
        g_lvgl_prof.frame_complete_cnt++;
    }

    /* --- STEP 0: Wait for PREVIOUS background DMA2D transfer BEFORE modifying s_rot_buf --- */
    uint32_t t_wait0 = DWT->CYCCNT;
    if ((s_dma2d_in_flight != 0U) || ((DMA2D->CR & DMA2D_CR_START) != 0U)) {
        if ((DMA2DSemHandle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
            if (osSemaphoreAcquire(DMA2DSemHandle, DMA2D_XFER_TIMEOUT) != osOK) {
                g_lvgl_prof.dma2d_timeout_cnt++;
                (void)HAL_DMA2D_Abort(&hdma2d);
            }
        }
        while ((DMA2D->CR & DMA2D_CR_START) != 0U) {
            __NOP();
        }
        s_dma2d_in_flight = 0U;
    }
    uint32_t t_wait1 = DWT->CYCCNT;
    g_lvgl_prof.dma2d_wait_us_sum += (t_wait1 - t_wait0) / cycles_per_us;

    /* --- STEP 1: Fast 4x-Unrolled 16x16 Tiled Rotation (DTCMRAM -> s_rot_buf) --- */
    uint32_t t0 = DWT->CYCCNT;

    const int32_t TILE_SIZE = 16;
    for (int32_t by = 0; by < log_h; by += TILE_SIZE) {
        int32_t y_end = (by + TILE_SIZE < log_h) ? (by + TILE_SIZE) : log_h;

        for (int32_t bx = 0; bx < log_w; bx += TILE_SIZE) {
            int32_t x_end = (bx + TILE_SIZE < log_w) ? (bx + TILE_SIZE) : log_w;

            for (int32_t lx = bx; lx < x_end; lx++) {
                uint32_t rot_row = (uint32_t)(log_w - 1 - lx);
                uint16_t * __restrict dst_ptr = &rot_base[rot_row * phys_w + (uint32_t)by];
                const uint16_t * __restrict src_ptr = &src_base[by * log_w + lx];

                int32_t ly = by;
                for (; ly <= y_end - 4; ly += 4) {
                    dst_ptr[0] = src_ptr[0];
                    src_ptr += log_w;
                    dst_ptr[1] = src_ptr[0];
                    src_ptr += log_w;
                    dst_ptr[2] = src_ptr[0];
                    src_ptr += log_w;
                    dst_ptr[3] = src_ptr[0];
                    src_ptr += log_w;
                    dst_ptr += 4;
                }
                for (; ly < y_end; ly++) {
                    *dst_ptr++ = *src_ptr;
                    src_ptr += log_w;
                }
            }
        }
    }

    uint32_t t1 = DWT->CYCCNT;
    g_lvgl_prof.rot_us_sum += (t1 - t0) / cycles_per_us;

    /* --- STEP 2: Clean Cortex-M7 D-Cache for s_rot_buf --- */
    uint32_t rot_addr = (uint32_t)s_rot_buf & ~0x1FUL;
    uint32_t rot_size = (phys_w * phys_h * 2U) + ((uint32_t)s_rot_buf - rot_addr);
    SCB_CleanDCache_by_Addr((uint32_t *)rot_addr, (int32_t)((rot_size + 31U) & ~31U));
    __DSB();

    uint32_t t2 = DWT->CYCCNT;
    g_lvgl_prof.dcache_us_sum += (t2 - t1) / cycles_per_us;

    /* --- STEP 3: Configure and Launch Non-Blocking DMA2D Transfer to SDRAM (0xC0000000) --- */
    uint32_t phys_x_start   = (uint32_t)y1;
    uint32_t phys_y_start   = (PHYSICAL_H - 1U) - (uint32_t)x2;
    uint32_t dst_sdram_addr = FB_BASE_ADDR + ((phys_y_start * PHYSICAL_W + phys_x_start) * 2U);

    hdma2d.Instance          = DMA2D;
    hdma2d.Init.Mode         = DMA2D_M2M;
    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
    hdma2d.Init.OutputOffset = PHYSICAL_W - phys_w;
    hdma2d.State             = HAL_DMA2D_STATE_READY;
    __HAL_UNLOCK(&hdma2d);

    DMA2D->CR      = DMA2D_M2M;
    DMA2D->FGPFCCR = DMA2D_INPUT_RGB565;
    DMA2D->OPFCCR  = DMA2D_OUTPUT_RGB565;
    DMA2D->FGOR    = 0U;
    DMA2D->OOR     = (PHYSICAL_W - phys_w);
    DMA2D->IFCR    = (DMA2D_FLAG_TC | DMA2D_FLAG_TE | DMA2D_FLAG_CE);

    hdma2d.XferCpltCallback  = dma2d_transfer_complete_cb;
    hdma2d.XferErrorCallback = dma2d_transfer_error_cb;

    if ((DMA2DSemHandle != NULL) && (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        (void)osSemaphoreAcquire(DMA2DSemHandle, 0U);
        s_dma2d_in_flight = 1U;

        if (HAL_DMA2D_Start_IT(&hdma2d, (uint32_t)s_rot_buf, dst_sdram_addr, phys_w, phys_h) != HAL_OK) {
            s_dma2d_in_flight = 0U;
        }
    } else {
        if (HAL_DMA2D_Start(&hdma2d, (uint32_t)s_rot_buf, dst_sdram_addr, phys_w, phys_h) == HAL_OK) {
            (void)HAL_DMA2D_PollForTransfer(&hdma2d, DMA2D_XFER_TIMEOUT);
        }
    }

    /* --- STEP 4: Notify LVGL immediately so CPU renders next chunk while DMA2D transfers --- */
    lv_disp_flush_ready(disp_drv);
}

#else

typedef int keep_pedantic_happy;
#endif
