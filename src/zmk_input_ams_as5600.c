#define DT_DRV_COMPAT zmk_input_ams_as5600

#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "zmk_input_ams_as5600/zmk_input_ams_as5600_config.h"

LOG_MODULE_REGISTER(zmk_input_ams_as5600, CONFIG_ZMK_INPUT_AMS_AS5600_LOG_LEVEL);

#define ZMK_INPUT_AMS_AS5600_CONF_REGISTER 0x07
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER 0x0B
#define ZMK_INPUT_AMS_AS5600_AGC_REGISTER 0x1A
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_UNDERFLOW_BIT 3
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_OVERFLOW_BIT 4
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MAGNET_DETECTED_BIT 5
#define ZMK_INPUT_AMS_AS5600_PULSES_PER_REV 4096
#define ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AS5600: "
#define GET_BIT(reg, pos) ((reg) & (1 << (pos)))

struct zmk_input_ams_as5600_config {
    struct i2c_dt_spec i2c_port;
};

struct zmk_input_ams_as5600_data {
    const struct device *dev;
    uint16_t last_angle;
    bool last_angle_initialized;
    struct k_timer timer;
    struct k_work work;
};

#if IS_ENABLED(ZMK_INPUT_AMS_AS5600_LOG_AGC)
static void zmk_input_ams_as5600_log_agc(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    uint8_t agc;
    int err = i2c_burst_read_dt(&config->i2c_port, ZMK_INPUT_AMS_AS5600_AGC_REGISTER, &agc, sizeof(agc));
    if (err) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "I2C read agc failed: %d", err);
        return;
    }
    LOG_INF(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC: %d", agc);
}
#endif

static int zmk_input_ams_as5600_process(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    int err;
    uint8_t read_buffer[3];
    uint8_t status;
    uint16_t angle;
    int32_t pulses;

#if IS_ENABLED(ZMK_INPUT_AMS_AS5600_LOG_AGC)
    zmk_input_ams_as5600_log_agc(dev);
#endif

    err = i2c_burst_read_dt(&(config->i2c_port), ZMK_INPUT_AMS_AS5600_STATUS_REGISTER, read_buffer, sizeof(read_buffer));
    if (err) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "I2C read data failed: %d", err);
        return err;
    }

    status = read_buffer[0];

    if (GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_OVERFLOW_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC overflow – magnet too weak");
        return -1;
    }
    if (GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_UNDERFLOW_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC underflow – magnet too strong");
        return -1;
    }
    if (!GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MAGNET_DETECTED_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Magnet not detected");
        return -1;
    }

    angle = ((uint16_t)read_buffer[1] << 8) | read_buffer[2];
    pulses = angle - data->last_angle;

    if (pulses > (ZMK_INPUT_AMS_AS5600_PULSES_PER_REV / 2)) {
        pulses -= ZMK_INPUT_AMS_AS5600_PULSES_PER_REV;
    } else if (pulses < (-ZMK_INPUT_AMS_AS5600_PULSES_PER_REV / 2)) {
        pulses += ZMK_INPUT_AMS_AS5600_PULSES_PER_REV;
    }

    LOG_DBG(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Angle: %d, Last: %d, Pulses: %d", angle, data->last_angle, pulses);

    data->last_angle = angle;

    if (!data->last_angle_initialized) {
        data->last_angle_initialized = true;
        return 0;
    }

    if (pulses) {
        err = input_report_rel(dev, INPUT_REL_WHEEL, pulses, true, K_FOREVER);
        if (err) {
            LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Input report failed: %d", err);
            return err;
        }
    }

    return 0;
}

static void zmk_input_ams_as5600_work_handler(struct k_work *work) {
    struct zmk_input_ams_as5600_data *data = CONTAINER_OF(work, struct zmk_input_ams_as5600_data, work);
    zmk_input_ams_as5600_process(data->dev);
}

void zmk_input_ams_as5600_timer_handler(struct k_timer *timer) {
    struct zmk_input_ams_as5600_data *data = CONTAINER_OF(timer, struct zmk_input_ams_as5600_data, timer);
    k_work_submit(&data->work);
}

/* Retry Logic */
static void as5600_retry_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(as5600_retry_work, as5600_retry_work_handler);

static void as5600_retry_work_handler(struct k_work *work) {
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    uint8_t agc = 0;
    int err = i2c_burst_read_dt(&config->i2c_port, ZMK_INPUT_AMS_AS5600_AGC_REGISTER, &agc, sizeof(agc));
    if (err) {
        LOG_WRN(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Sensor not ready, retrying in 5s (err=%d)", err);
        k_work_schedule(&as5600_retry_work, K_SECONDS(5));
        return;
    }

    LOG_INF(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Sensor ready! AGC=%d", agc);

    /* Starte jetzt den Timer */
    k_timer_start(&data->timer, K_MSEC(CONFIG_ZMK_INPUT_AMS_AS5600_PERIOD), K_MSEC(CONFIG_ZMK_INPUT_AMS_AS5600_PERIOD));
}

static int zmk_input_ams_as5600_initialize(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    if (!device_is_ready(config->i2c_port.bus)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "I2C bus not ready");
        return -ENODEV;
    }

    data->dev = dev;
    data->last_angle = 0;
    data->last_angle_initialized = false;

    k_work_init(&data->work, &zmk_input_ams_as5600_work_handler);
    k_timer_init(&data->timer, &zmk_input_ams_as5600_timer_handler, NULL);

    LOG_INF(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Retrying detection in 2s");
    k_work_schedule(&as5600_retry_work, K_SECONDS(2));

    return 0;
}

#define ZMK_INPUT_AMS_AS5600_INIT(n)                                 \
    static struct zmk_input_ams_as5600_data zmk_input_ams_as5600_data##n;       \
    static const struct zmk_input_ams_as5600_config zmk_input_ams_as5600_cfg##n = { \
        .i2c_port = I2C_DT_SPEC_INST_GET(n)};                        \
    DEVICE_DT_INST_DEFINE(n, zmk_input_ams_as5600_initialize, NULL,  \
                          &zmk_input_ams_as5600_data##n,             \
                          &zmk_input_ams_as5600_cfg##n,              \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_INPUT_AMS_AS5600_INIT)
