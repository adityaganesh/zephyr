#include "icm20948_rtio.h"
#include "icm20948_decoder.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ICM20948_RTIO, CONFIG_SENSOR_LOG_LEVEL);

static int icm20948_prep_reg_read_rtio_async(struct icm20948_bus *bus,
					     uint8_t reg,
					     uint8_t *buf,
					     size_t len,
					     struct rtio_sqe **out)
{
	struct rtio_sqe *sqes[2];
	struct rtio_sqe *wr;
	struct rtio_sqe *rd;
	int rc;

	if (bus == NULL || bus->ctx == NULL || bus->iodev == NULL) {
		LOG_ERR("Invalid RTIO bus");
		return -EINVAL;
	}

	rc = rtio_sqe_acquire_array(bus->ctx, ARRAY_SIZE(sqes), sqes);
	if (rc < 0) {
		LOG_ERR("Unable to acquire RTIO SQEs");
		return rc;
	}

	wr = sqes[0];
	rd = sqes[1];

	/*
	 * I2C register read:
	 *
	 * START + slave(W) + register address
	 * RESTART + slave(R) + received bytes + STOP
	 */
	rtio_sqe_prep_tiny_write(wr,
				 bus->iodev,
				 RTIO_PRIO_NORM,
				 &reg,
				 sizeof(reg),
				 NULL);

	wr->flags |= RTIO_SQE_TRANSACTION;

	rtio_sqe_prep_read(rd,
			   bus->iodev,
			   RTIO_PRIO_NORM,
			   buf,
			   len,
			   NULL);

	rd->iodev_flags |= RTIO_IODEV_I2C_RESTART |
			   RTIO_IODEV_I2C_STOP;

	if (out != NULL) {
		*out = rd;
	}

	return 0;
}

static void icm20948_fifo_complete_cb(struct rtio *r,
				      const struct rtio_sqe *sqe,
				      int result,
				      void *arg)
{
	const struct device *dev = arg;
	struct icm20948_data *data = dev->data;
	struct rtio_iodev_sqe *user_sqe = data->streaming_sqe;

	size_t required_len = sizeof(struct icm20948_fifo_data);
	uint8_t *buf;
	uint32_t buf_len;


	int rc = rtio_sqe_rx_buf(user_sqe, required_len, required_len, &buf, &buf_len);




	if (!user_sqe) {
		return;
	}

	if (result < 0) {
		rtio_iodev_sqe_err(user_sqe, result);
		data->streaming_sqe = NULL;
		return;
	}


	rtio_iodev_sqe_ok(user_sqe,data->fifo_data.fifo_count);
}

static void icm20948_fifo_count_cb(struct rtio *r,
				   const struct rtio_sqe *sqe,
				   int result,
				   void *arg)
{
	const struct device *dev = arg;
	struct icm20948_data *data = dev->data;
	uint8_t *buf;
	uint32_t buf_len;
	struct rtio_sqe *read_sqe;
	struct rtio_sqe *cb_sqe;
	int rc;

	if (result < 0) {
		rtio_iodev_sqe_err(data->streaming_sqe, result);
		return;
	}


	size_t required_len = (uint16_t)(((uint8_t)((data->fifo_data.fifo_count_buf[0]) & (0x1f)) << 8) | data->fifo_data.fifo_count_buf[1]);

	data->fifo_data.fifo_count = required_len;

	rc = rtio_sqe_rx_buf(data->streaming_sqe,
			     required_len,
			     required_len,
			     &buf,
			     &buf_len);
	if (rc < 0) {
		rtio_iodev_sqe_err(data->streaming_sqe, rc);
		return;
	}

	uint8_t *fifo_payload = buf;

	rc = icm20948_prep_reg_read_rtio_async(&data->bus,
					       ICM20948_REG_FIFO_R_W,
					       fifo_payload,
					       required_len,
					       &read_sqe);
	if (rc < 0) {
		rtio_iodev_sqe_err(data->streaming_sqe, rc);
		return;
	}

	read_sqe->flags |= RTIO_SQE_CHAINED;

	cb_sqe = rtio_sqe_acquire(data->bus.ctx);
	rtio_sqe_prep_callback(cb_sqe, icm20948_fifo_complete_cb, (void *)dev, NULL);

	rtio_submit(data->bus.ctx, 0);


}



void icm20948_submit(const struct device *sensor, struct rtio_iodev_sqe *iodev_sqe)
{
    const struct icm20948_config *cfg = sensor->config;

    struct icm20948_data *drv_data = sensor->data;

    if(IS_ENABLED(CONFIG_ICM20948_TRIGGER)) {

        if(iodev_sqe == NULL) {
            LOG_ERR("No SQE available for streaming");
            return;
        }

        drv_data->streaming_sqe = iodev_sqe;

        gpio_pin_interrupt_configure_dt(&cfg->int_pin,
                        GPIO_INT_EDGE_TO_ACTIVE);

    }

}

void icm20948_fifo_event(const struct device *dev)
{
    const struct icm20948_config *cfg = dev->config;
    struct icm20948_data *drv_data = dev->data;
    struct rtio_sqe *sqe;
    int rc = 0;
	/* 1. Read INT_STATUS_3 */
	rc = icm20948_prep_reg_read_rtio_async(&drv_data->bus,
					       ICM20948_REG_INT_STATUS_3,
					       &drv_data->fifo_data.int_status3,
					       1,
					       &sqe);
	if (rc < 0) {
		rtio_iodev_sqe_err(drv_data->streaming_sqe, rc);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	/*2. Read fifo en1 and fifo en2*/
	rc = icm20948_prep_reg_read_rtio_async(&drv_data->bus,
					       ICM20948_REG_FIFO_EN_2,
					       &drv_data->fifo_data.fifo_en2,
					       1,
					       &sqe);
	if (rc < 0) {
		rtio_iodev_sqe_err(drv_data->streaming_sqe, rc);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	/*3. Read fifo mode*/
	rc = icm20948_prep_reg_read_rtio_async(&drv_data->bus,
					       ICM20948_FIFO_MODE,
					       &drv_data->fifo_data.fifo_mode,
					       1,
					       &sqe);
	if (rc < 0) {
		rtio_iodev_sqe_err(drv_data->streaming_sqe, rc);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	/*4. Read FIFO count high + low */
	rc = icm20948_prep_reg_read_rtio_async(&drv_data->bus,
					       ICM20948_FIFO_CNTH,
					       (uint8_t *)&drv_data->fifo_data.fifo_count_buf,
					       2,
					       &sqe);
	if (rc < 0) {
		rtio_iodev_sqe_err(drv_data->streaming_sqe, rc);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	/*5. Callback decides actual FIFO read length */
	struct rtio_sqe *cb = rtio_sqe_acquire(drv_data->bus.ctx);

	rtio_sqe_prep_callback(cb, icm20948_fifo_count_cb, (void *)dev, NULL);

	rtio_submit(drv_data->bus.ctx, 0);

}