// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (c) 2019 Luc Verhaegen <libv@skynet.be>
 */

/*
 * This is the Allwinner CMOS sensor interface, for the secondary interface,
 * which should allow for full 24bit RGB input.
 *
 * We are using this for the FOSDEM Video teams HDMI input board.
 *
 * We are building up this functionality in logical single steps, and our
 * first approximation has us receive raw pixelbus data from a tfp401 module,
 * so we need no interaction with an i2c module and are free to bring this
 * trivial hw, with non-trivial v4l2 plumbing, without outside influence.
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define MODULE_NAME	"sun4i-csi1"

static int sun4i_csi1_probe(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;

	dev_info(dev, "%s();\n", __func__);

	return 0;
}

static int sun4i_csi1_remove(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;

	dev_info(dev, "%s();\n", __func__);

	return 0;
}

/* We are currently only testing on sun7i, but should work for sun4i as well */
static const struct of_device_id sun4i_csi1_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-csi1", },
	{ .compatible = "allwinner,sun7i-a20-csi1", },
	{},
};
MODULE_DEVICE_TABLE(of, sun4i_csi1_of_match);

static struct platform_driver sun4i_csi1_platform_driver = {
	.probe = sun4i_csi1_probe,
	.remove = sun4i_csi1_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(sun4i_csi1_of_match),
	},
};
module_platform_driver(sun4i_csi1_platform_driver);

MODULE_DESCRIPTION("Allwinner A10/A20 CMOS Sensor Interface 1 V4L2 driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
