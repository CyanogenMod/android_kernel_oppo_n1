/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
//#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/melfas_ts.h>

#include <linux/firmware.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>

#include <linux/input/mt.h> // slot


#include "bin_tsp.c"

#define SLOT_TYPE	0 // touch event type choise

#define I2C_RETRY_CNT 5 //Fixed value
#define DOWNLOAD_RETRY_CNT 5 //Fixed value
#define MELFAS_DOWNLOAD 1 //Fixed value


#define TS_READ_LEN_ADDR 0x0F //Fixed value
#define TS_READ_START_ADDR 0x10 //Fixed value
#define TS_READ_REGS_LEN 66 //Fixed value
#define TS_WRITE_REGS_LEN 16 //Fixed value


#define TS_MAX_TOUCH 	10 //Model Dependent
#define TS_READ_HW_VER_ADDR 0xF1 //Model Dependent
#define TS_READ_SW_VER_ADDR 0xF5 //Model Dependent



#define MELFAS_HW_REVISON 0x01 //Model Dependent
#define MELFAS_FW_VERSION 0x02 //Model Dependent

/* OPPO 2012-08-22 Van add begin for reason */
static int logo_level  = 2;

#define DEBUG_INFO_ERROR     1
#define DEBUG_INFO           2
#define DEBUG_TRACE          3

#define print_melfas_ts(level, ...) \
			do { \
				if (logo_level	>= (level)) \
					printk(__VA_ARGS__); \
			} while (0) 
/* OPPO 2012-08-22 Van add end for reason */

#if SLOT_TYPE
#define REPORT_MT(touch_number, x, y, area, pressure) \
do {     \
	input_mt_slot(ts->input_dev, touch_number);	\
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);	\
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, area);         \
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, pressure); \
} while (0)
#endif

static struct workqueue_struct *melfas_work;


static struct muti_touch_info g_Mtouch_info[TS_MAX_TOUCH];


#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif
extern int isc_fw_download(struct melfas_ts_data *info, const u8 *data, size_t len);

static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
    struct i2c_adapter *adapter = client->adapter;
    struct i2c_msg msg;
    int ret = -1;

    msg.addr = client->addr;
    msg.flags = 0x00;
    msg.len = 1;
    msg.buf = (u8 *) & addr;

    ret = i2c_transfer(adapter, &msg, 1);

    if (ret >= 0)
    {
        msg.addr = client->addr;
        msg.flags = I2C_M_RD;
        msg.len = length;
        msg.buf = (u8 *) value;

        ret = i2c_transfer(adapter, &msg, 1);
    }

    if (ret < 0)
    {
        pr_err("[TSP] : read error : [%d]", ret);
    }

    return ret;
}

/* OPPO 2012-07-12 wangjc Delete begin for reason */
#if 0
static int melfas_i2c_write(struct i2c_client *client, char *buf, int length)
{
    int i;
    char data[TS_WRITE_REGS_LEN];

    if (length > TS_WRITE_REGS_LEN)
    {
        pr_err("[TSP] %s :size error \n", __FUNCTION__);
        return -EINVAL;
    }

    for (i = 0; i < length; i++)
        data[i] = *buf++;

    i = i2c_master_send(client, (char *) data, length);

    if (i == length)
        return length;
    else
    {
        pr_err("[TSP] :write error : [%d]", i);
        return -EIO;
    }
}
#endif
/* OPPO 2012-07-12 wangjc Delete end */

#if SLOT_TYPE
static void melfas_ts_release_all_finger(struct melfas_ts_data *ts)
{
	int i;
	for (i = 0; i < TS_MAX_TOUCH; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(ts->input_dev);
}
#else	
static void melfas_ts_release_all_finger(struct melfas_ts_data *ts)
{
	int i;
	for(i=0; i<TS_MAX_TOUCH; i++)
	{
		if(-1 == g_Mtouch_info[i].pressure)
			continue;

		if(g_Mtouch_info[i].pressure == 0)
			input_mt_sync(ts->input_dev);

		if(0 == g_Mtouch_info[i].pressure)
			g_Mtouch_info[i].pressure = -1;
	}
	input_sync(ts->input_dev);
}
#endif

static int check_firmware(struct melfas_ts_data *ts, u8 *val)
{
    int ret = 0;
    uint8_t i = 0;

    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = melfas_i2c_read(ts->client, TS_READ_HW_VER_ADDR, 1, &val[0]);
        ret = melfas_i2c_read(ts->client, TS_READ_SW_VER_ADDR, 1, &val[1]);

        if (ret >= 0)
        {
            pr_info("[TSP] : HW Revision[0x%02x] SW Version[0x%02x] \n", val[0], val[1]);
            break; // i2c success
        }
    }

    if (ret < 0)
    {
        pr_info("[TSP] %s,%d: i2c read fail[%d] \n", __FUNCTION__, __LINE__, ret);
        return ret;
    }
}

static int firmware_update(struct melfas_ts_data *ts)
{
    int ret = 0;
    uint8_t fw_ver[2] = {0, };

    struct melfas_ts_data *info = ts;
    //struct i2c_client *client = info->client;
    //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	return 0;

    ret = check_firmware(ts, fw_ver);
    if (ret < 0)
        pr_err("[TSP] check_firmware fail! [%d]", ret);
    else
    {
#if MELFAS_DOWNLOAD
        if (fw_ver[1] < MELFAS_FW_VERSION)
        {
            //int ver;

            pr_info("[TSP] %s: \n", __func__);
            ret = isc_fw_download(info, MELFAS_binary, MELFAS_binary_nLength);
        }
#endif
    }

    return ret;
}

static void melfas_ts_get_data(struct melfas_ts_data *ts)
{
    int ret = 0, i;
    uint8_t buf[TS_READ_REGS_LEN] = { 0, };
    int read_num, fingerID, Touch_Type = 0, touchState = 0;

	//print_melfas_ts(DEBUG_TRACE, "%s\n", __func__);
	
    if (ts == NULL)
		print_melfas_ts(DEBUG_INFO_ERROR, "%s erro ts data is null\n", __func__);

    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = melfas_i2c_read(ts->client, TS_READ_LEN_ADDR, 1, buf);

        if (ret >= 0)
        {
			//print_melfas_ts(DEBUG_TRACE, "%s, TS_READ_LEN_ADDR [%d]\n", __func__, ret);
            break; // i2c success
        }
    }

    if (ret < 0)
    {
        pr_info("[TSP] %s,%d: i2c read fail[%u] \n", __FUNCTION__, __LINE__, ret);
        return;
    }
    else
    {
        read_num = buf[0];
		//print_melfas_ts(DEBUG_TRACE, "%s, read_num[%d]\n", __func__, read_num);
    }

    if (read_num > 0)
    {
    	if(read_num > 60) {
			pr_err("[TSP] %s read_num is too large. \n", __FUNCTION__);
		}
		
        for (i = 0; i < I2C_RETRY_CNT; i++)
        {
            ret = melfas_i2c_read(ts->client, TS_READ_START_ADDR, read_num, buf);
            if (ret >= 0)
            {
				//print_melfas_ts(DEBUG_TRACE, "%s, TS_READ_START_ADDR [%d]\n", __func__, ret);
                break; // i2c success
            }
        }

        if (ret < 0)
        {
            pr_info("[TSP] %s,%d: i2c read fail[%u] \n", __FUNCTION__, __LINE__, ret);
            return;
        }
        else
        {
            for (i = 0; i < read_num; i = i + 6)
            {
                Touch_Type = (buf[i] >> 5) & 0x03;

                /* touch type is panel */
                if (Touch_Type == TOUCH_SCREEN)
                {
                    fingerID = (buf[i] & 0x0F) - 1;
                    touchState = (buf[i] & 0x80);

					if(fingerID >= 0 && fingerID <= 9) {
	                    g_Mtouch_info[fingerID].posX = (uint16_t)(buf[i + 1] & 0x0F) << 8 | buf[i + 2];
	                    g_Mtouch_info[fingerID].posY = (uint16_t)(buf[i + 1] & 0xF0) << 4 | buf[i + 3];
	                    g_Mtouch_info[fingerID].area = buf[i + 4];

	                    if (touchState)
	                        g_Mtouch_info[fingerID].pressure = buf[i + 5];
	                    else
	                        g_Mtouch_info[fingerID].pressure = 0;
					}
                }
            }


            for (i = 0; i < TS_MAX_TOUCH; i++)
            {
                if (g_Mtouch_info[i].pressure == -1)
                    continue;

			  	#if SLOT_TYPE
				  	if(g_Mtouch_info[i].pressure == 0)
				  	{
					  // release event
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false); 
				  	}
				  	else
				  	{
						REPORT_MT(i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].area, g_Mtouch_info[i].pressure);
				  	}
	          	#else
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
				
				  	if(g_Mtouch_info[i].pressure == 0)
				  	{
					  	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					  	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
						print_melfas_ts(DEBUG_TRACE, "ts %d up\n", i);
				  	}
				  	else
				  	{
					  	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].area);
					  	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_Mtouch_info[i].pressure);
						print_melfas_ts(DEBUG_TRACE, "ts %d dn\n", i);
				  	}
	          	#endif
				
					input_mt_sync(ts->input_dev);  

                	if (g_Mtouch_info[i].pressure == 0)
                    	g_Mtouch_info[i].pressure = -1;
            }
            input_sync(ts->input_dev);
        }
    }
}

static void melfas_ts_work_func(struct work_struct *work)
{
	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);

	melfas_ts_get_data(ts);
	enable_irq(ts->client->irq);
}


static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	int ret;
    struct melfas_ts_data *ts;// = (struct melfas_ts_data *) handle;
	print_melfas_ts(DEBUG_TRACE, "ts irq\n");
	
	ts = (struct melfas_ts_data *) handle;
	if(!ts) {
		pr_info("[TSP] %s NULL ts\n", __FUNCTION__);
		return IRQ_HANDLED;
	}
	if(!melfas_work) {
		pr_info("[TSP] %s NULL melfas_work\n", __FUNCTION__);
	}
	if(!&ts->work) {
		pr_info("[TSP] %s NULL ts->work\n", __FUNCTION__);
	}
	disable_irq_nosync(ts->client->irq);
	ret = queue_work(melfas_work, &ts->work);
    //melfas_ts_get_data(ts);
    return IRQ_HANDLED;
}

static struct regulator *melfas_tp_2p8;


static int melfas_ts_ldo_init(struct i2c_client *client, int init)
{
	int rc = 0;

	if (init) {
		melfas_tp_2p8 = devm_regulator_get(&client->dev, "8921_l16");
		if (IS_ERR(melfas_tp_2p8)) {
			dev_err(NULL, "[TSP] unable to get 8921_l16 2p8\n");
			return PTR_ERR(melfas_tp_2p8);
		}

		rc = regulator_set_voltage(melfas_tp_2p8, 2800000,
				2800000);
		if (rc) {
			dev_err(NULL, "[TSP] unable to set voltage level for"
					"8921_l16 2p8\n");
			return rc;
		}

		return 0;
	}

	regulator_set_voltage(melfas_tp_2p8, 0, 2800000);
	return rc;
}

static int melfas_ts_ldo_enable(int on)
{
	int ret = 0;

	if (IS_ERR(melfas_tp_2p8)) {
		pr_err("[TSP] %s: melfas_tp_2p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(melfas_tp_2p8,
				5000);
		if (ret < 0) {
			pr_err("[TSP] %s: Unable to set HPM of the regulator:"
				"melfas_tp_2p8\n", __func__);
			return ret;
		}

		ret = regulator_enable(melfas_tp_2p8);
		if (ret) {
			dev_err(NULL, "[TSP] %s: unable to enable the melfas_tp_2p8\n",
				__func__);
			regulator_set_optimum_mode(melfas_tp_2p8, 0);
			return ret;
		}

	} else {
		ret = regulator_disable(melfas_tp_2p8);
		if (ret) {
			dev_err(NULL, "[TSP] %s: unable to disable the hsusb 3p3\n",
				 __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(melfas_tp_2p8, 0);
		if (ret < 0)
			pr_err("[TSP] %s: Unable to set LPM of the regulator:"
				"melfas_tp_2p8\n", __func__);
	}

	pr_debug("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
}



static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct melfas_ts_data *ts;
	struct melfas_tsi_platform_data *pdata;
    int ret = 0, i;
    //uint8_t buf[4] = {0, };

    pr_info("[TSP] melfas_ts_probe\n.");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        pr_info("[TSP] melfas_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        pr_info("[TSP] %s: failed to create a state of melfas-ts\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

	ret = melfas_ts_ldo_init(client, 1);
	if (ret) {
		pr_err("melfas ts ldo init failed\n");
		goto err_init_ldo_failed;
	}

	ret = melfas_ts_ldo_enable(1);
	if (ret) {
		dev_err(NULL, "melfas vreg enable failed\n");
		goto err_enable_ldo_failed;
	}

//	ts->pdata = client->dev.platform_data;
//	if (ts->pdata->power_enable)
//		ts->power = ts->pdata->power_enable;
	

//    ts->power(1);
	
//    msleep(200);
//    msleep(300);

    ts->client = client;
    i2c_set_clientdata(client, ts);

	pdata = client->dev.platform_data;
	ts->pdata = pdata;

    ret = firmware_update(ts);

    if (ret < 0)
    {
        pr_info("[TSP] %s: firmware update fail\n", __FUNCTION__);
        goto err_detect_failed;
    }

    ts->input_dev = input_allocate_device();
    if (!ts->input_dev)
    {
        pr_info("[TSP] %s: Not enough memory\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }

    ts->input_dev->name = "melfas-ts";
	
/* OPPO 2012-08-22 Van Modify begin for reason */
	/* For single touch */
	input_set_abs_params(ts->input_dev, ABS_X, 0, pdata->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, pdata->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);

	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, pdata->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, pdata->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, pdata->max_area, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, pdata->max_pressure, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 9, 0, 0);

    __set_bit(EV_ABS,  ts->input_dev->evbit);
	__set_bit(EV_SYN, ts->input_dev->evbit);
/* OPPO 2012-08-22 Van Modify end */
    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        pr_info("[TSP] %s: Failed to register device\n", __FUNCTION__);
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }

    if (ts->client->irq)
    {
        ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING/*(IRQF_TRIGGER_LOW | IRQF_ONESHOT)*/, ts->client->name, ts);

        if (ret > 0)
        {
            pr_info("[TSP] %s: Can't allocate irq %d, ret %d\n", __FUNCTION__, ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

    for (i = 0; i < TS_MAX_TOUCH; i++) /* _SUPPORT_MULTITOUCH_ */
        g_Mtouch_info[i].pressure = -1;

	INIT_WORK(&ts->work, melfas_ts_work_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = melfas_ts_early_suspend;
    ts->early_suspend.resume = melfas_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    pr_err("[TSP] %s: Start touchscreen. name: %s, irq: %d\n", __FUNCTION__, ts->client->name, ts->client->irq);

    return 0;

err_request_irq:
    pr_info("[TSP] %s: err_request_irq failed\n", __func__);
    free_irq(client->irq, ts);
err_input_register_device_failed:
    pr_info("[TSP] %s: err_input_register_device failed\n", __func__);
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    pr_info("[TSP] %s: err_input_dev_alloc failed\n", __func__);
err_detect_failed:
    pr_info("[TSP] %s: err_after_get_regulator failed_\n", __func__);
	melfas_ts_ldo_enable(0);
err_enable_ldo_failed:
	pr_info("[TSP] %s: err_after_get_regulator failed_\n", __func__);
	melfas_ts_ldo_init(client, 0);
err_init_ldo_failed:
	pr_info("[TSP] %s: err_after_get_regulator failed_\n", __func__);
err_alloc_data_failed:
	pr_info("[TSP] %s: err_after_get_regulator failed_\n", __func__);
				
    kfree(ts);

err_check_functionality_failed:
    pr_info("[TSP] %s: err_check_functionality failed_\n", __func__);

    return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    unregister_early_suspend(&ts->early_suspend);
	cancel_work_sync(&ts->work);
    free_irq(client->irq, ts);
    //ts->power(0);
	melfas_ts_ldo_enable(0);
	melfas_ts_ldo_init(client, 0);
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    //int ret;
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    melfas_ts_release_all_finger(ts);
    disable_irq(client->irq);

    //ts->power(0);
	melfas_ts_ldo_enable(0);
    return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
    //int ret;
    //struct melfas_ts_data *ts = i2c_get_clientdata(client);
    
    //ts->power(1);
	melfas_ts_ldo_enable(1);
    msleep(200);
    enable_irq(client->irq); // scl wave
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
    { MELFAS_TS_NAME, 0 },
    { }
};

static struct i2c_driver melfas_ts_driver =
{
    .driver =
    {
        .name = MELFAS_TS_NAME,
        .owner	= THIS_MODULE,
    }, 
    .id_table = melfas_ts_id, 
    .probe = melfas_ts_probe, 
    .remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = melfas_ts_suspend, 
    .resume = melfas_ts_resume,
#endif
};

extern int get_pcb_version(void);

static int __devinit melfas_ts_init(void)
{
    if (get_pcb_version() >= 20)
        return -EINVAL;
    pr_err("\n%s\n\n", __FUNCTION__);
    melfas_work = create_singlethread_workqueue("ts_wq");
    if (!melfas_work) {
        pr_err("%s create workqueue failed.\n", __func__);
        return -ENOMEM;
    }
    return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
    i2c_del_driver(&melfas_ts_driver);
	if (melfas_work)
		destroy_workqueue(melfas_work);
}

MODULE_DESCRIPTION("Driver for Melfas MIP Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.2");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

