/**
 * @file lv_port_indev_template.c
 * @brief LVGL v8.2 input device port with DWT cycle profiling for GT9147 touch diagnostics.
 */

#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev_template.h"
#include "../../lvgl.h"
#include "drv_gt9147.h"
#include "stm32h7xx_hal.h"

/*********************
 *      DEFINES
 *********************/
#define TOUCH_LOGICAL_W             480
#define TOUCH_LOGICAL_H             800
#define TOUCH_PHYSICAL_W            800
#define TOUCH_PHYSICAL_H            480
#define TOUCH_MAP_MODE              1

/**********************
 *      TYPEDEFS & EXTERNS
 **********************/
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

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y);

/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad = NULL;
static drv_gt9147_point_t s_touch_point = {0};

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initialize and register the GT9147 touchpad input device with LVGL.
 * @param None
 * @retval None
 */
void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;

    touchpad_init();

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
}

/**
 * @brief Stub function kept for compatibility with app_lvgl.c.
 * @param None
 * @retval None
 */
void lv_port_indev_create_debug_overlay(void)
{
    /* Intentionally empty so debug overlay does not trigger extra screen redraws */
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Initialize the GT9147 hardware driver.
 * @param None
 * @retval None
 */
static void touchpad_init(void)
{
    drv_gt9147_init();
}

/**
 * @brief Read touchpad state and record execution time in g_lvgl_prof.indev_us_sum.
 * @param indev_drv Pointer to the LVGL input device driver descriptor.
 * @param data      Pointer to the LVGL input device data structure.
 * @retval None
 */
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    (void)indev_drv;
    uint32_t t0 = DWT->CYCCNT;

    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    g_lvgl_prof.indev_calls++;

    if (touchpad_is_pressed()) {
        touchpad_get_xy(&last_x, &last_y);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }

    data->point.x = last_x;
    data->point.y = last_y;

    uint32_t t1 = DWT->CYCCNT;
    g_lvgl_prof.indev_us_sum += (t1 - t0) / (SystemCoreClock / 1000000UL);
}

/**
 * @brief Poll the GT9147 controller with 20ms throttling and filter out 0xFFFF noise.
 * @param None
 * @retval true if a valid touch is detected, false otherwise.
 */
static bool touchpad_is_pressed(void)
{
    static uint32_t last_poll_tick = 0;
    static bool last_press_state = false;

    uint32_t now = HAL_GetTick();

    if ((now - last_poll_tick) >= 20U) {
        last_poll_tick = now;
        g_lvgl_prof.indev_i2c_polls++;

        if (drv_gt9147_read_touch(&s_touch_point) == DRV_GT9147_OK) {
            if ((s_touch_point.is_pressed != 0U) &&
                (s_touch_point.x < (uint16_t)TOUCH_PHYSICAL_W) &&
                (s_touch_point.y < (uint16_t)TOUCH_PHYSICAL_H)) {
                last_press_state = true;
            } else {
                last_press_state = false;
            }
        }
    }

    return last_press_state;
}

/**
 * @brief Convert physical landscape GT9147 coordinates (800x480) to portrait (480x800).
 * @param x Pointer to store mapped X coordinate.
 * @param y Pointer to store mapped Y coordinate.
 * @retval None
 */
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    int32_t raw_x = (int32_t)s_touch_point.x;
    int32_t raw_y = (int32_t)s_touch_point.y;

    int32_t conv_x = (TOUCH_PHYSICAL_H - 1) - raw_y;
    int32_t conv_y = raw_x;

    if (conv_x < 0) conv_x = 0;
    if (conv_x >= TOUCH_LOGICAL_W) conv_x = TOUCH_LOGICAL_W - 1;
    if (conv_y < 0) conv_y = 0;
    if (conv_y >= TOUCH_LOGICAL_H) conv_y = TOUCH_LOGICAL_H - 1;

    *x = (lv_coord_t)conv_x;
    *y = (lv_coord_t)conv_y;
}

#else

typedef int keep_pedantic_happy;
#endif
