/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/setup.h>

#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include <linux/pcb_version.h>

static int ftm_mode = 0;
static int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "normal"))
		ftm_mode = MSM_BOOT_MODE_NORMAL;
	else if (!strcmp(s, "factory2"))
		ftm_mode = MSM_BOOT_MODE_FACTORY;
	else if (!strcmp(s, "ftmrecovery"))
		ftm_mode = MSM_BOOT_MODE_RECOVERY;
	else if (!strcmp(s, "charge"))
		ftm_mode = MSM_BOOT_MODE_CHARGE;
	else if (!strcmp(s, "ftmwifi"))
		ftm_mode = MSM_BOOT_MODE_WLAN;
	else if (!strcmp(s, "ftmrf"))
		ftm_mode = MSM_BOOT_MODE_RF;
	else
		ftm_mode = MSM_BOOT_MODE_NORMAL;

	pr_err("board_mfg_mode_init" "ftm_mode=%d\n", ftm_mode);

	return 0;
}
__setup("oppo_ftm_mode=", board_mfg_mode_init);

int get_boot_mode(void)
{
	return ftm_mode;
}

static char pwron_event[16];
static int __init start_reason_setup(char *str)
{
	strcpy(pwron_event, str);
	printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);

	return 1;
}
__setup("androidboot.startupmode=", start_reason_setup);

char * get_start_reason(void)
{
	return pwron_event;
}

static char boot_mode[16];
static int __init boot_mode_setup(char *str)
{
	strcpy(boot_mode, str);

	printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
	return 1;
}
__setup("androidboot.mode=", boot_mode_setup);

char * get_boot_mode_str(void)
{
	return boot_mode;
}

static int current_pcb_version_num = PCB_VERSION_UNKNOWN;
int get_pcb_version(void)
{
	return current_pcb_version_num;
}
EXPORT_SYMBOL(get_pcb_version);

static char *saved_command_line_pcb_version;
int __init board_pcb_verson_init(char *s)
{
	saved_command_line_pcb_version = s;

	if (!strcmp(s, "evb"))
		current_pcb_version_num = PCB_VERSION_EVB;
	else if (!strcmp(s, "evt"))
		current_pcb_version_num = PCB_VERSION_EVT;
	else if (!strcmp(s, "dvt"))
		current_pcb_version_num = PCB_VERSION_DVT;
	else if (!strcmp(s, "pvt"))
		current_pcb_version_num = PCB_VERSION_PVT;
	else if (!strcmp(s, "td_evb"))
		current_pcb_version_num = PCB_VERSION_EVB_TD;
	else if (!strcmp(s, "td_pvt2"))
		current_pcb_version_num = PCB_VERSION_PVT2_TD;
	else if (!strcmp(s, "td_pvt3"))
		current_pcb_version_num = PCB_VERSION_PVT3_TD;
	else if (!strcmp(s, "n1t_evt"))
		current_pcb_version_num = PCB_VERSION_EVT_N1;
	else if (!strcmp(s, "n1f_evt"))
		current_pcb_version_num = PCB_VERSION_EVT_N1F;
	else if (!strcmp(s, "n1f_evt3"))
		current_pcb_version_num = PCB_VERSION_EVT3_N1F;
	else if (!strcmp(s, "n1f_dvt"))
		current_pcb_version_num = PCB_VERSION_DVT_N1F;
	else if (!strcmp(s, "n1f_pvt"))
		current_pcb_version_num = PCB_VERSION_PVT_N1F;
	else if (!strcmp(s, "n1t_evt3"))
		current_pcb_version_num = PCB_VERSION_EVT3_N1T;
	else if (!strcmp(s, "n1t_dvt"))
		current_pcb_version_num = PCB_VERSION_DVT_N1T;
	else if (!strcmp(s, "n1t_pvt"))
		current_pcb_version_num = PCB_VERSION_PVT_N1T;

	return 0;
}
__setup("oppo.pcb_version=", board_pcb_verson_init);

/********** Extra Oppo attributes begin here **********/

static ssize_t closemodem_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	//writing '1' to close and '0' to open
	//pr_err("closemodem buf[0] = 0x%x",buf[0]);
	switch (buf[0]) {
	case 0x30:
		break;
	case 0x31:
	//  pr_err("closemodem now");
		gpio_direction_output(27, 0);
		mdelay(4000);
		break;
	default:
		break;
	}

	return count;
}

struct kobj_attribute closemodem_attr = {
	.attr = { "closemodem", 0644 },
//	.show = &closemodem_show,
	.store = &closemodem_store
};

static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", ftm_mode);
}

static struct kobj_attribute ftmmode_attr = {
	.attr = { "ftmmode", 0644 },
	.show = &ftmmode_show,
};

static struct attribute *systeminfo_attr_list[] = {
	&ftmmode_attr.attr,
	&closemodem_attr.attr,
	NULL,
};

static struct attribute_group systeminfo_attr_group = {
	.attrs = systeminfo_attr_list,
};

static struct kobject *systeminfo_kobj;

static char pin_info[64] = {0};
static ssize_t pin_info_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return snprintf(buf, 4096, "%s\n", pin_info);
}

static ssize_t pin_info_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	printk("pininfo_store----count:%d   wjp debug     \n", count);
	if(count >= sizeof(pin_info))
		count = sizeof(pin_info) - 1;
	strncpy(pin_info, buf, count);
	pin_info[count] = '\0';
	return count;
}

struct kobj_attribute pin_info_attr = {
	.attr = { "pin_info", 0660 },
	.show = &pin_info_show,
	.store = &pin_info_store
};

extern int get_modem_reset_num(void);
static int modem_reset_count = 0;
static ssize_t modem_reset_count_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	modem_reset_count = get_modem_reset_num();
	return sprintf(buf, "%d\n", modem_reset_count);
}

static ssize_t modem_reset_count_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char *after;
	unsigned long reset_count = simple_strtoul(buf, &after, 10);
	modem_reset_count = (int)reset_count;
	return count;
}

struct kobj_attribute modem_reset_count_attr = {
	.attr = { "modem_reset_count", 0660 },
	.show = &modem_reset_count_show,
	.store = &modem_reset_count_store,
};

static int need_pin_process_flag = -1;
static ssize_t need_pin_process_flag_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", need_pin_process_flag);
}

static ssize_t need_pin_process_flag_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char *after;
	unsigned long flag = simple_strtoul(buf, &after, 10);
	need_pin_process_flag = (int)flag;
	return count;
}

void set_need_pin_process_flag(int flag)
{
	need_pin_process_flag = flag;
}

struct kobj_attribute need_pin_process_flag_attr = {
	.attr = { "need_pin_process_flag", 0660 },
	.show = &need_pin_process_flag_show,
	.store = &need_pin_process_flag_store,
};

static int sim_status = -1;
static ssize_t sim_status_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	sim_status = gpio_get_value(72);
	return sprintf(buf, "%d\n", sim_status);
}

static ssize_t sim_status_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	char *after;
	unsigned long status = simple_strtoul(buf, &after, 10);
	sim_status = (int)status;
	return count;
}

int get_sim_status(void)
{
	return gpio_get_value(72);
}

struct kobj_attribute sim_status_attr = {
	.attr = { "sim_status", 0660 },
	.show = &sim_status_show,
	.store = &sim_status_store,
};

static struct attribute *modeminfo_attr_list[] = {
	&pin_info_attr.attr,
	&modem_reset_count_attr.attr,
	&need_pin_process_flag_attr.attr,
	&sim_status_attr.attr,
	NULL,
};

static struct attribute_group modeminfo_attr_group = {
	.attrs = modeminfo_attr_list,
};

static struct kobject *modeminfo_kobj;

static int __init oppo_params_init(void)
{
	int rc;

	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	if (systeminfo_kobj == NULL) {
		pr_err("%s: Failed to create system info kobject", __func__);
		return -ENOMEM;
	}
	rc = sysfs_create_group(systeminfo_kobj, &systeminfo_attr_group);
	if (rc != 0) {
		pr_err("%s: Failed to create system info sysfs node: %d",
				__func__, rc);
		return rc;
	}

	modeminfo_kobj = kobject_create_and_add("modeminfo", NULL);
	if (modeminfo_kobj == NULL) {
		pr_err("%s: Failed to create modem info kobject", __func__);
		return -ENOMEM;
	}
	rc = sysfs_create_group(modeminfo_kobj, &modeminfo_attr_group);
	if (rc != 0) {
		pr_err("%s: Failed to create modem info sysfs node: %d",
				__func__, rc);
		return rc;
	}

	return 0;
}

static void __exit oppo_params_exit(void)
{
	sysfs_remove_group(systeminfo_kobj, &systeminfo_attr_group);
	kobject_del(systeminfo_kobj);
	sysfs_remove_group(modeminfo_kobj, &modeminfo_attr_group);
	kobject_del(modeminfo_kobj);
}

module_init(oppo_params_init);
module_exit(oppo_params_exit);
MODULE_DESCRIPTION("Oppo boot params initialization module");
MODULE_AUTHOR("The CyanogenMod Project");
MODULE_LICENSE("GPL");
