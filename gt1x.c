/* drivers/input/touchscreen/gt1x.c
 *
 * 2010 - 2014 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * Version: 1.6   
 */
 
#include "gt1x_generic.h"
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
#include <linux/input/mt.h>
#endif
#include <linux/msm_drm_notify.h>

static struct input_dev *input_dev;
static spinlock_t irq_lock;
static int irq_disabled;
#ifdef CONFIG_OF
int gt1x_rst_gpio;
int gt1x_int_gpio;
#endif

static struct goodix_ts_data *goodix_ts;
static int gt1x_register_powermanger(struct goodix_ts_data *ts);
static int gt1x_unregister_powermanger(struct goodix_ts_data *ts);

/**
 * gt1x_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct i2c_msg msg = {
		.flags = 0,
		.addr = gt1x_i2c_client->addr,
	};
	return _do_i2c_write(&msg, addr, buffer, len);
}

/**
 * gt1x_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 *Return: 0: success, otherwise: failed
 */
s32 gt1x_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	u8 addr_buf[GTP_ADDR_LENGTH] = { (addr >> 8) & 0xFF, addr & 0xFF };
	struct i2c_msg msgs[2] = {
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = 0,
		 .buf = addr_buf,
		 .len = GTP_ADDR_LENGTH},
		{
		 .addr = gt1x_i2c_client->addr,
		 .flags = I2C_M_RD}
	};
	return _do_i2c_read(msgs, addr, buffer, len);
}

/**
 * gt1x_irq_enable - enable irq function.
 *
 */
void gt1x_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_disabled) {
		irq_disabled = 0;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		enable_irq(gt1x_i2c_client->irq);
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
	}
}

/**
 * gt1x_irq_enable - disable irq function.
 *  disable irq and wait bottom half
 *  thread(gt1x_ts_work_thread)
 */
void gt1x_irq_disable(void)
{
	unsigned long irqflags;

	/* because there is an irq enable action in
	 * the bottom half thread, we need to wait until
	 * bottom half thread finished.
	 */
	synchronize_irq(gt1x_i2c_client->irq);
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_disabled) {
		irq_disabled = 1;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		disable_irq(gt1x_i2c_client->irq);
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
	}
}

#ifndef CONFIG_OF
int gt1x_power_switch(s32 state)
{
    return 0;
}
#endif

int gt1x_debug_proc(u8 * buf, int count)
{
	return -1;
}

#ifdef CONFIG_GTP_CHARGER_SWITCH
u32 gt1x_get_charger_status(void)
{
	/* 
	 * Need to get charger status of
	 * your platform.
	 */
	 return 0;
}
#endif

/**
 * gt1x_ts_irq_handler - External interrupt service routine 
 *		for interrupt mode.
 * @irq:  interrupt number.
 * @dev_id: private data pointer.
 * Return: Handle Result.
 *  		IRQ_WAKE_THREAD: top half work finished,
 *  		wake up bottom half thread to continue the rest work.
 */
static irqreturn_t gt1x_ts_irq_handler(int irq, void *dev_id)
{
	unsigned long irqflags;

	/* irq top half, use nosync irq api to 
	 * disable irq line, if irq is enabled,
	 * then wake up bottom half thread */
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_disabled) {
		irq_disabled = 1;
		spin_unlock_irqrestore(&irq_lock, irqflags);
		disable_irq_nosync(gt1x_i2c_client->irq);
		return IRQ_WAKE_THREAD;
	} else {
		spin_unlock_irqrestore(&irq_lock, irqflags);
		return IRQ_HANDLED;
	}
}

/**
 * gt1x_touch_down - Report touch point event .
 * @id: trackId
 * @x:  input x coordinate
 * @y:  input y coordinate
 * @w:  input pressure
 * Return: none.
 */
void gt1x_touch_down(s32 x, s32 y, s32 size, s32 id)
{
#ifdef CONFIG_GTP_CHANGE_X2Y
	GTP_SWAP(x, y);
#endif

	input_report_key(input_dev, BTN_TOUCH, 1);
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
#else
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
#endif

	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_PRESSURE, size);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, size);

#ifndef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_touch_up -  Report touch release event.
 * @id: trackId
 * Return: none.
 */
void gt1x_touch_up(s32 id)
{
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#else
	input_mt_sync(input_dev);
#endif
}

/**
 * gt1x_ts_work_thread - Goodix touchscreen work function.
 * @iwork: work struct of gt1x_workqueue.
 * Return: none.
 */
static irqreturn_t gt1x_ts_work_thread(int irq, void *data)
{
	u8 point_data[11] = { 0 };
	u8 end_cmd = 0;
	u8 finger = 0;
	s32 ret = 0;

    if (update_info.status) {
        GTP_DEBUG("Ignore interrupts during fw update.");
        return IRQ_HANDLED;
    }
    
#ifdef CONFIG_GTP_GESTURE_WAKEUP
	ret = gesture_event_handler(input_dev);
	if (ret >= 0)
		goto exit_work_func;
#endif

	if (gt1x_halt) {
		GTP_DEBUG("Ignore interrupts after suspend");
        return IRQ_HANDLED;
	}

	ret = gt1x_i2c_read(GTP_READ_COOR_ADDR, point_data, sizeof(point_data));
	if (ret < 0) {
		GTP_ERROR("I2C transfer error,read coor points??? fb blank state");
		goto exit_work_func;
	}

	finger = point_data[0];
	if (finger == 0x00)
		gt1x_request_event_handler();

	if ((finger & 0x80) == 0) {
#ifdef CONFIG_HOTKNOT_BLOCK_RW
		if (!hotknot_paired_flag)
#endif
		{
			goto exit_eint;
		}
	}

#ifdef CONFIG_HOTKNOT_BLOCK_RW
	ret = hotknot_event_handler(point_data);
	if (!ret)
		goto exit_work_func;
#endif

#ifdef CONFIG_GTP_PROXIMITY
	ret = gt1x_prox_event_handler(point_data);
	if (ret > 0)
		goto exit_work_func;
#endif

#ifdef CONFIG_GTP_WITH_STYLUS
	ret = gt1x_touch_event_handler(point_data, input_dev, pen_dev);
#else
	ret = gt1x_touch_event_handler(point_data, input_dev, NULL);
#endif

exit_work_func:
	if (!gt1x_rawdiff_mode && (ret >= 0 || ret == ERROR_VALUE)) {
		ret = gt1x_i2c_write(GTP_READ_COOR_ADDR, &end_cmd, 1);
		if (ret < 0)
			GTP_ERROR("I2C write end_cmd  error!");
	}
exit_eint:
    gt1x_irq_enable();
	return IRQ_HANDLED;
}

/* 
 * Devices Tree support, 
*/
#ifdef CONFIG_OF
/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev,struct goodix_ts_data *ts)
{
	struct device_node *np;
    int ret = 0;
    u32 temp_val;

    if (!dev)
        return -ENODEV;
    
    np = dev->of_node;

    if (of_find_property(np, "vcc_ana-supply", NULL))
		ts->manage_afe_power_ana = true;
	if (of_find_property(np, "vcc_dig-supply", NULL))
		ts->manage_power_dig = true;

	if (ts->manage_afe_power_ana) {
		ret = of_property_read_u32(np, "qcom,afe-load", &temp_val);
		if (!ret) {
			ts->afe_load_ua = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read AFE load\n");
			return ret;
		}

		ret = of_property_read_u32(np, "qcom,afe-vtg-min", &temp_val);
		if (!ret) {
			ts->afe_vtg_min_uv = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read AFE min voltage\n");
			return ret;
		}

		ret = of_property_read_u32(np, "qcom,afe-vtg-max", &temp_val);
		if (!ret) {
			ts->afe_vtg_max_uv = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read AFE max voltage\n");
			return ret;
		}
	}
	if (ts->manage_power_dig) {
		ret = of_property_read_u32(np, "qcom,dig-load", &temp_val);
		if (!ret) {
			ts->dig_load_ua = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read digital load\n");
			return ret;
		}

		ret = of_property_read_u32(np, "qcom,dig-vtg-min", &temp_val);
		if (!ret) {
			ts->dig_vtg_min_uv = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read digital min voltage\n");
			return ret;
		}

		ret = of_property_read_u32(np, "qcom,dig-vtg-max", &temp_val);
		if (!ret) {
			ts->dig_vtg_max_uv = (int) temp_val;
		} else {
			dev_err(dev, "Unable to read digital max voltage\n");
			return ret;
		}
	}

	if (ts->manage_power_dig && ts->manage_afe_power_ana) {
		ret = of_property_read_u32(np,
				"qcom,afe-power-on-delay-us", &temp_val);
		if (!ret)
			ts->power_on_delay = (u32)temp_val;
		else
			dev_info(dev, "Power-On Delay is not specified\n");

		ret = of_property_read_u32(np,
				"qcom,afe-power-off-delay-us", &temp_val);
		if (!ret)
			ts->power_off_delay = (u32)temp_val;
		else
			dev_info(dev, "Power-Off Delay is not specified\n");

		dev_dbg(dev, "power-on-delay = %u, power-off-delay = %u\n",
			ts->power_on_delay, ts->power_off_delay);
	}

	ts->irq_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	if (!gpio_is_valid(ts->irq_gpio))
	{
		dev_err(dev, "No valid irq gpio");
		return -EINVAL;
	}

	ts->rst_gpio = of_get_named_gpio(np, "goodix,reset-gpio", 0);
	if (!gpio_is_valid(ts->rst_gpio)){
		dev_err(dev, "No valid rst gpio");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "irq-flags",
				   &ts->irq_flags);
	if (ret) {
		dev_info(dev,
			 "Failed get int-trigger-type from dts,set default\n");
		ts->irq_flags = GTP_INT_TRIGGER;
	}
	
	gt1x_rst_gpio = ts->rst_gpio;
	gt1x_int_gpio = ts->irq_gpio;

    return 0;
}

#endif

static int gt1x_power_init(struct goodix_ts_data *ts)
{
	int error;
	struct regulator *vcc_ana, *vcc_dig;

	if (ts->manage_afe_power_ana) {
		vcc_ana = regulator_get(&ts->client->dev, "vcc_ana");
		if (IS_ERR(vcc_ana)) {
			error = PTR_ERR(vcc_ana);
			dev_err(&ts->client->dev,"%s: regulator get failed vcc_ana rc=%d\n",
				__func__, error);
			return error;
		}

		if (regulator_count_voltages(vcc_ana) > 0) {
			error = regulator_set_voltage(vcc_ana,
				ts->afe_vtg_min_uv, ts->afe_vtg_max_uv);
			if (error) {
				dev_err(&ts->client->dev,"%s: regulator set vtg failed vcc_ana rc=%d\n",
					__func__, error);
				regulator_put(vcc_ana);
				return error;
			}
		}
		ts->vcc_ana = vcc_ana;
	}

	if (ts->manage_power_dig) {
		vcc_dig = regulator_get(&ts->client->dev, "vcc_dig");
		if (IS_ERR(vcc_dig)) {
			error = PTR_ERR(vcc_dig);
			dev_err(&ts->client->dev,"%s: regulator get failed vcc_dig rc=%d\n",
				__func__, error);
			return error;
		}

		if (regulator_count_voltages(vcc_dig) > 0) {
			error = regulator_set_voltage(vcc_dig,
				ts->dig_vtg_min_uv, ts->dig_vtg_max_uv);
			if (error) {
				dev_err(&ts->client->dev,"%s: regulator set vtg failed vcc_dig rc=%d\n",
					__func__, error);
				regulator_put(vcc_dig);
				return error;
			}
		}
		ts->vcc_dig = vcc_dig;
	}
	return 0;
}

static int gt1x_power_deinit(struct goodix_ts_data *ts)
{
	if (ts->vcc_ana)
		regulator_put(ts->vcc_ana);
	if (ts->vcc_dig)
		regulator_put(ts->vcc_dig);

	return 0;
}

static int reg_set_load_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_load(reg, load_uA) : 0;
}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int gt1x_power_switch(int on)
{
	int ret;
	struct goodix_ts_data *ts;

	ts = goodix_ts;

	if (!ts->vcc_ana)
		dev_err(&ts->client->dev,"%s: analog regulator is not available\n", __func__);

	if (!ts->vcc_dig)
		dev_err(&ts->client->dev,"%s: digital regulator is not available\n", __func__);

	if (!ts->vcc_ana && !ts->vcc_dig) {
		dev_err(&ts->client->dev,"%s: no regulators available\n", __func__);
		return -EINVAL;
	}

	if (!ts)
		goto reg_off;

	if (ts->regulator_enabled) {
		dev_info(&ts->client->dev,"%s: regulator already enabled\n", __func__);
		return 0;
	}

	if (ts->vcc_ana) {
		ret = reg_set_load_check(ts->vcc_ana,
			ts->afe_load_ua);
		if (ret < 0) {
			dev_info(&ts->client->dev,"%s: Regulator vcc_ana set_opt failed rc=%d\n",
				__func__, ret);
			return ret;
		}

		ret = regulator_enable(ts->vcc_ana);
		if (ret) {
			dev_err(&ts->client->dev,"%s: Regulator vcc_ana enable failed rc=%d\n",
				__func__, ret);
			reg_set_load_check(ts->vcc_ana, 0);
			return ret;
		}
	}

	if (ts->vcc_dig) {
		ret = reg_set_load_check(ts->vcc_dig,
			ts->dig_load_ua);
		if (ret < 0) {
			dev_err(&ts->client->dev,"%s: Regulator vcc_dig set_opt failed rc=%d\n",
				__func__, ret);
			return ret;
		}
		ret = regulator_enable(ts->vcc_dig);
		if (ret) {
			dev_err(&ts->client->dev,"%s: Regulator vcc_dig enable failed rc=%d\n",
				__func__, ret);
			reg_set_load_check(ts->vcc_dig, 0);
			return ret;
		}
	}

	ts->regulator_enabled = true;

	return 0;

reg_off:
	if (!ts->regulator_enabled) {
		dev_info(&ts->client->dev,"%s: regulator not enabled\n", __func__);
		return 0;
	}

	if (ts->vcc_dig) {
		reg_set_load_check(ts->vcc_dig, 0);
		regulator_disable(ts->vcc_dig);
	}

	if (ts->vcc_ana) {
		reg_set_load_check(ts->vcc_ana, 0);
		regulator_disable(ts->vcc_ana);
	}
	ts->regulator_enabled = false;
	return 0;
}

static void gt1x_release_resource(struct goodix_ts_data *ts)
{
    if (gpio_is_valid(GTP_INT_PORT)) {
		gpio_direction_input(GTP_INT_PORT);
		gpio_free(GTP_INT_PORT);
	}

    if (gpio_is_valid(GTP_RST_PORT)) {
		gpio_direction_output(GTP_RST_PORT, 0);
		gpio_free(GTP_RST_PORT);
	}

	gt1x_power_deinit(ts);

	if (input_dev) {
		input_unregister_device(input_dev);
		input_dev = NULL;
	}
}

/**
 * gt1x_request_gpio - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_gpio(void)
{
	s32 ret = 0;

	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		gt1x_i2c_client->irq = gpio_to_irq(GTP_INT_PORT);
	}

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);
	return ret;
}

/**
 * gt1x_request_irq - Request interrupt.
 * Return
 *      0: succeed, -1: failed.
 */
static s32 gt1x_request_irq(void)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG("INT trigger type:%x", gt1x_int_type);
	ret = devm_request_threaded_irq(&gt1x_i2c_client->dev,
			gt1x_i2c_client->irq,
			gt1x_ts_irq_handler,
			gt1x_ts_work_thread,
			irq_table[gt1x_int_type],
			gt1x_i2c_client->name,
			gt1x_i2c_client);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		return -1;
	} else {
		gt1x_irq_disable();
		return 0;
	}
}

/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#ifdef CONFIG_GTP_HAVE_TOUCH_KEY
	u8 index = 0;
#endif

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifdef CONFIG_GTP_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
    input_mt_init_slots(input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(input_dev, GTP_MAX_TOUCH); 
#endif
#endif
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef CONFIG_GTP_HAVE_TOUCH_KEY
	for (index = 0; index < GTP_MAX_KEY_NUM; index++)
		input_set_capability(input_dev, EV_KEY, gt1x_touch_key_array[index]);
#endif

#ifdef CONFIG_GTP_GESTURE_WAKEUP
	input_set_capability(input_dev, EV_KEY, KEY_GES_REGULAR);
    input_set_capability(input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

#ifdef CONFIG_GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_x_max, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, gt1x_abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, gt1x_abs_y_max, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	input_dev->name = "goodix-ts";
	input_dev->phys = "input/ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xDEAD;
	input_dev->id.product = 0xBEEF;
	input_dev->id.version = 10427;

	ret = input_register_device(input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}

static int gt1x_pinctrl_init(struct goodix_ts_data *ts)
{
	int retval = 0;

	ts->ts_pinctrl = devm_pinctrl_get(&ts->client->dev);
	if (IS_ERR_OR_NULL(ts->ts_pinctrl)) {
		retval = PTR_ERR(ts->ts_pinctrl);
		dev_info(&ts->client->dev, "No pinctrl found\n");
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		retval = PTR_ERR(ts->pinctrl_state_active);
		dev_err(&ts->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts->pinctrl_state_suspend);
		dev_err(&ts->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_release
		= pinctrl_lookup_state(ts->ts_pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_release)) {
		retval = PTR_ERR(ts->pinctrl_state_release);
		dev_err(&ts->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	dev_info(&ts->client->dev, "Success init pinctrl\n");
	return 0;
err_pinctrl_lookup:
	devm_pinctrl_put(ts->ts_pinctrl);
err_pinctrl_get:
	ts->ts_pinctrl = NULL;
	return retval;
}

/**
 * gt1x_ts_probe -   I2c probe.
 * @client: i2c device struct.
 * @id: device id.
 * Return  0: succeed, <0: failed.
 */
static int gt1x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = -1;
	struct goodix_ts_data *ts;

	/* do NOT remove these logs */
	GTP_INFO("GTP Driver Version: %s,slave addr:%02xh",
			GTP_DRIVER_VERSION, client->addr);

	gt1x_i2c_client = client;
	spin_lock_init(&irq_lock);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		return -ENODEV;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Failed alloc ts memory\n");
		devm_kfree(&client->dev, ts);
		return -EINVAL;
	}

	ts->client = client;

	i2c_set_clientdata(client, ts);

	goodix_ts = ts;

#ifdef CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		ret = gt1x_parse_dt(&client->dev,ts);
		if (ret < 0)
			return -EINVAL;
	}
#else
#error [GOODIX]only support devicetree platform
#endif

	ret = gt1x_power_init(ts);
	if (ret) {
		GTP_ERROR("Failed get regulator");
		ret = -EINVAL;
		goto exit_clean;
	}

    /* power on */
	ret = gt1x_power_switch(SWITCH_ON);
	if (ret < 0) {
		GTP_ERROR("Power on failed");
		goto exit_clean;
	}

	ret = gt1x_pinctrl_init(ts);
	if (ret < 0) {
		/* if define pinctrl must define the following state
		 * to let int-pin work normally: default, int_output_high,
		 * int_output_low, int_input
		 */
		GTP_ERROR("Failed get wanted pinctrl state");
		goto exit_clean;
	}

	/*
	* Pinctrl handle is optional. If pinctrl handle is found
	* let pins to be configured in active state. If not
	* found continue further without error.
     */
	ret = pinctrl_select_state(ts->ts_pinctrl, ts->pinctrl_state_active);
	if (ret < 0) {
		GTP_ERROR("Failed to select %s pinstate %d",PINCTRL_STATE_ACTIVE, ret);
     }

	/* gpio resource */
	ret = gt1x_request_gpio();
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		goto exit_clean;
	}

    /* reset ic & do i2c test */
	ret = gt1x_reset_guitar();
	if (ret != 0) {
		ret = gt1x_power_switch(SWITCH_OFF);
		if (ret < 0)
			goto exit_clean;
		ret = gt1x_power_switch(SWITCH_ON);
		if (ret < 0)
			goto exit_clean;
		ret = gt1x_reset_guitar(); /* retry */
		if (ret != 0) {
			GTP_ERROR("Reset guitar failed!");
			goto exit_clean;
		}
	}

	/* check firmware, initialize and send
	 * chip configuration data, initialize nodes */
	gt1x_init();

	ret = gt1x_request_input_dev();
	if (ret < 0)
		goto err_input;

	ret = gt1x_request_irq();
	if (ret < 0)
		goto err_irq;

#ifdef CONFIG_GTP_ESD_PROTECT
	/* must before auto update */
	gt1x_init_esd_protect();
	gt1x_esd_switch(SWITCH_ON);
#endif

#ifdef CONFIG_GTP_AUTO_UPDATE
	do {
		struct task_struct *thread = NULL;
		thread = kthread_run(gt1x_auto_update_proc,
				(void *)NULL,
				"gt1x_auto_update");
		if (IS_ERR(thread))
			GTP_ERROR("Failed to create auto-update thread: %d.", ret);
	} while (0);
#endif

	gt1x_register_powermanger(ts);
	gt1x_irq_enable();
	return 0;

err_irq:
err_input:
	gt1x_deinit();
exit_clean:
	gt1x_release_resource(ts);
	devm_kfree(&client->dev, ts);
	i2c_set_clientdata(client, NULL);
	GTP_ERROR("GTP probe failed:%d", ret);
	return -ENODEV;
}

/**
 * gt1x_ts_remove -  Goodix touchscreen driver release function.
 * @client: i2c device struct.
 * Return  0: succeed, -1: failed.
 */
static int gt1x_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data * ts ;

	GTP_INFO("GTP driver removing...");

	ts = i2c_get_clientdata(client);

	gt1x_unregister_powermanger(ts);

    gt1x_deinit();
    gt1x_release_resource(ts);

    return 0;
}


/* frame buffer notifier block control the suspend/resume procedure */
static int gt1x_dis_panel_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int blank;
	struct goodix_ts_data *ts =
	container_of(self, struct goodix_ts_data, dis_panel_notifier);

	if (!evdata || (evdata->id != 0))
		return 0;
    
	if (event == MSM_DRM_EARLY_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
        if ( blank == MSM_DRM_BLANK_UNBLANK) {
			GTP_INFO("%s receives EARLY_BLANK:UNBLANK",__func__);
        }else if (blank == MSM_DRM_BLANK_POWERDOWN) {
			GTP_INFO("%s: receives EARLY_BLANK:POWERDOWN\n",
				__func__);
			gt1x_suspend(ts);
		} else {
			GTP_INFO("%s: receives wrong data EARLY_BLANK:%d\n",
				__func__, blank);
		}
	}

	if ( event == MSM_DRM_EVENT_BLANK) {
		blank = *(int *)(evdata->data);
		if (blank == MSM_DRM_BLANK_POWERDOWN) {
			GTP_INFO("%s: receives BLANK:POWERDOWN\n", __func__);
		} else if (blank == MSM_DRM_BLANK_UNBLANK) {
			GTP_INFO("%s: receives BLANK:UNBLANK\n", __func__);
			 gt1x_resume(ts);
		} else {
			GTP_INFO("%s: receives wrong data BLANK:%d\n",
				__func__, blank);
		}
	}

	return 0;
}

static int gt1x_register_powermanger(struct goodix_ts_data *ts)
{
	ts->dis_panel_notifier.notifier_call = gt1x_dis_panel_notifier_callback;
	msm_drm_register_client(&ts->dis_panel_notifier);
	return 0;
}

static int gt1x_unregister_powermanger(struct goodix_ts_data *ts)
{
	msm_drm_unregister_client(&ts->dis_panel_notifier);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gt1x_match_table[] = {
		{.compatible = "goodix,gt1x",},
		{ },
};
#endif

static const struct i2c_device_id gt1x_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver gt1x_ts_driver = {
	.probe = gt1x_ts_probe,
	.remove = gt1x_ts_remove,
	.id_table = gt1x_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = gt1x_match_table,
#endif
		   },
};

/**   
 * gt1x_ts_init - Driver Install function.
 * Return   0---succeed.
 */
static int __init gt1x_ts_init(void)
{
	GTP_INFO("GTP driver installing...");
	return i2c_add_driver(&gt1x_ts_driver);
}

/**   
 * gt1x_ts_exit - Driver uninstall function.
 * Return   0---succeed.
 */
static void __exit gt1x_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&gt1x_ts_driver);
}

module_init(gt1x_ts_init);
module_exit(gt1x_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL v2");
