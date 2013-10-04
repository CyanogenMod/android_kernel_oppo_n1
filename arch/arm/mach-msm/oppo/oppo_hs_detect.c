/*
 * Copyright (C) 2013 The CyanogenMod Project
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/switch.h>

#include <mach/oppo_hs_detect.h>

static struct switch_dev h2w_switch;

int oppo_hs_detect_power(int enable)
{
	int rc;
	struct regulator *vreg_ts3a;

	vreg_ts3a = regulator_get(NULL, "8921_l15");
	if (IS_ERR(vreg_ts3a))
		return PTR_ERR(vreg_ts3a);

	if (enable) {
		rc = regulator_enable(vreg_ts3a);
		if (rc) {
			pr_err("%s: unable to enable vreg_ts3a", __func__);
			goto power_err;
		}
	} else {
		rc = regulator_disable(vreg_ts3a);
		if (rc) {
			pr_err("%s: unable to disable vreg_ts3a", __func__);
			goto power_err;
		}
	}

power_err:
	regulator_put(vreg_ts3a);
	return rc;
}

void oppo_hs_detect_update(int hs_on, int mic_on)
{
	int state = 0;

	if (hs_on && mic_on)
		state = 1 << 0;
	else if (hs_on)
		state = 1 << 1;
	else if (mic_on)
		state = 1 << 2;
	else
		state = 0;

	switch_set_state(&h2w_switch, state);
}

static ssize_t oppo_hs_detect_print_name(struct switch_dev *s, char *buf)
{
	switch (switch_get_state(s)) {
		case NO_DEVICE:
			return sprintf(buf, "No Device\n");
		case HS_WITH_MIC:
			return sprintf(buf, "Headset\n");
		case HS_WITHOUT_MIC:
			return sprintf(buf, "Headset\n");
	}

	return -EINVAL;
}

static int __init oppo_hs_detect_init(void)
{
	int rc = 0;
	struct regulator *vreg_ts3a;

	vreg_ts3a = regulator_get(NULL, "8921_l15");
	if (IS_ERR(vreg_ts3a))
		return PTR_ERR(vreg_ts3a);

	rc = regulator_set_voltage(vreg_ts3a, 3300000, 3300000);
	if (rc) {
		pr_err("%s: unable to set voltage for vreg_ts3a", __func__);
		regulator_put(vreg_ts3a);
		return rc;
	}

	h2w_switch.name = "h2w";
	h2w_switch.print_name = oppo_hs_detect_print_name;
	rc = switch_dev_register(&h2w_switch);
	if (rc) {
		pr_err("%s: failed to register h2w switch device", __func__);
		goto init_regulator_put;
	}

	rc = gpio_request(58, "TSA3_SELECT_GPIO");
	if (rc < 0) {
		pr_err("%s: failed to allocate TSA3_SELECT_GPIO", __func__);
		gpio_free(58);
		goto init_switch_reg;
	}
	gpio_direction_output(58, 0);

	return 0;

init_switch_reg:
	switch_dev_unregister(&h2w_switch);
init_regulator_put:
	regulator_put(vreg_ts3a);
	return rc;
}

static void __exit oppo_hs_detect_exit(void)
{
	oppo_hs_detect_power(0);

	gpio_free(58);
}

module_init(oppo_hs_detect_init);
module_exit(oppo_hs_detect_exit);
MODULE_DESCRIPTION("Headset detection driver for Oppo N1");
MODULE_AUTHOR("The CyanogenMod Project");
MODULE_LICENSE("GPL");
