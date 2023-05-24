// SPDX-License-Identifier: GPL-2.0
/*
 * Author: Alexey Smirnov <s.alexey@gmail.com>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/time64.h>
#include <linux/time_namespace.h>

static bool set_uptime_offset = true;
module_param(set_uptime_offset, bool, S_IRUGO);

struct vrtc_data {
	struct rtc_device *rtc;
	struct timespec64 ts;
	struct timespec64 start_time;
};

static struct platform_device *plat_dev;

static int vrtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct vrtc_data *rtc = dev_get_drvdata(dev);

	struct timespec64 uptime, diff, runtime;

	/* fs/proc/uptime.c */

	/* get current uptime */
	ktime_get_boottime_ts64(&uptime);
	timens_add_boottime(&uptime);

	if (set_uptime_offset) {
		/* append uptime to start_time */
		runtime = timespec64_add(rtc->start_time, uptime);
	} else {
		/* find runtime since start_time */
		diff = timespec64_sub(uptime, rtc->ts);
		runtime = timespec64_add(rtc->start_time, diff);
	}

	rtc_time64_to_tm(runtime.tv_sec, tm);

	return 0;
}

static int vrtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct vrtc_data *rtc = dev_get_drvdata(dev);

	/* save tstamp */
	ktime_get_boottime_ts64(&rtc->ts);
	timens_add_boottime(&rtc->ts);

	rtc->start_time.tv_sec = rtc_tm_to_time64(tm);
	rtc->start_time.tv_nsec = 0;

	return 0;
}

static const struct rtc_class_ops vrtc_ops = {
	.read_time = vrtc_read_time,
	.set_time = vrtc_set_time,
};

static ssize_t vrtc_sysfs_set_runtime(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct rtc_time tm;
	unsigned long long runtime = 0;

	if (sscanf(buf, " %llu ", &runtime) != 1) {
		dev_err(dev, "Failed to set runtime\n");
		return -EINVAL;
	}

	rtc_time64_to_tm(runtime, &tm);
	vrtc_set_time(dev->parent, &tm);

	return count;
}

static ssize_t vrtc_sysfs_show_runtime(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct rtc_time tm;
	time64_t runtime;

	vrtc_read_time(dev->parent, &tm);
	runtime = rtc_tm_to_time64(&tm);

	return sprintf(buf, "%llu\n", (unsigned long long)runtime);
}

static DEVICE_ATTR(runtime, S_IRUGO | S_IWUSR, vrtc_sysfs_show_runtime,
		   vrtc_sysfs_set_runtime);

static struct attribute *vrtc_attrs[] = {
	&dev_attr_runtime.attr, //
	NULL //
};

static const struct attribute_group vrtc_sysfs_files = {
	.attrs = vrtc_attrs,
};

static int vrtc_nvmem_read(void *priv, unsigned int offset, void *_val,
			   size_t bytes)
{
	struct platform_device *pdev = priv;
	time64_t *val = _val;
	struct rtc_time tm;

	vrtc_read_time(&pdev->dev, &tm);

	*val = rtc_tm_to_time64(&tm);

	return 0;
}

static int vrtc_nvmem_write(void *priv, unsigned int offset, void *_val,
			    size_t bytes)
{
	struct platform_device *pdev = priv;
	time64_t *val = _val;
	struct rtc_time tm;

	rtc_time64_to_tm(*val, &tm);
	vrtc_set_time(&pdev->dev, &tm);

	return 0;
}

static struct nvmem_config vrtc_nvmem_config = {
	.name = "vrtc_nvmem",
	.word_size = sizeof(time64_t),
	.stride = sizeof(time64_t),
	.size = sizeof(time64_t),
	.reg_read = vrtc_nvmem_read,
	.reg_write = vrtc_nvmem_write,
};

static int vrtc_probe(struct platform_device *pdev)
{
	struct vrtc_data *vrtc;
	int ret;

	vrtc = devm_kzalloc(&pdev->dev, sizeof(*vrtc), GFP_KERNEL);
	if (!vrtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, vrtc);

	vrtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(vrtc->rtc))
		return PTR_ERR(vrtc->rtc);

	ret = rtc_add_group(vrtc->rtc, &vrtc_sysfs_files);
	if (ret)
		return ret;

	vrtc_nvmem_config.priv = pdev;

	vrtc->rtc->ops = &vrtc_ops;
	ret = devm_rtc_register_device(vrtc->rtc);
	if (!ret)
		devm_rtc_nvmem_register(vrtc->rtc, &vrtc_nvmem_config);
	return ret;
}

static struct platform_driver vrtc_driver = {
    .probe = vrtc_probe,
    .driver =
	{
	    .name = "simple_vrtc",
	},
};

static int __init vrtc_init(void)
{
	int err;

	err = platform_driver_register(&vrtc_driver);
	if (err)
		return err;

	err = -ENOMEM;
	plat_dev = platform_device_alloc("simple_vrtc", -1);
	if (!plat_dev)
		goto exit_free_mem;

	err = platform_device_add(plat_dev);
	if (err)
		goto exit_device_del;

	return 0;

exit_device_del:
	platform_device_del(plat_dev);

exit_free_mem:
	platform_device_put(plat_dev);

	platform_driver_unregister(&vrtc_driver);
	return err;
}

static void __exit vrtc_exit(void)
{
	platform_device_unregister(plat_dev);
	platform_driver_unregister(&vrtc_driver);
}

MODULE_DESCRIPTION("Virtual RTC Timer driver");
MODULE_AUTHOR("Alexey Smirnov <s.alexey@gmail.com");
MODULE_LICENSE("GPL");

module_init(vrtc_init);
module_exit(vrtc_exit);
