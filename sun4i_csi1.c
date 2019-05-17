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
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include <media/v4l2-device.h>

#define MODULE_NAME	"sun4i-csi1"

struct sun4i_csi1 {
	struct device *dev;

	struct clk *clk_bus;
	struct clk *clk_module;
	struct clk *clk_ram;
	struct reset_control *reset;

	void __iomem *mmio;

	bool powered;

	struct v4l2_device v4l2_dev[1];
};

#define SUN4I_CSI1_ENABLE		0X000
#define SUN4I_CSI1_CONFIG		0X004
#define SUN4I_CSI1_CAPTURE		0X008
#define SUN4I_CSI1_SCALE		0X00C
#define SUN4I_CSI1_FIFO0_BUFFER_A	0X010
#define SUN4I_CSI1_FIFO0_BUFFER_B	0X014
#define SUN4I_CSI1_FIFO1_BUFFER_A	0X018
#define SUN4I_CSI1_FIFO1_BUFFER_B	0X01C
#define SUN4I_CSI1_FIFO2_BUFFER_A	0X020
#define SUN4I_CSI1_FIFO2_BUFFER_B	0X024
#define SUN4I_CSI1_BUFFER_CONTROL	0X028
#define SUN4I_CSI1_BUFFER_STATUS	0X02C
#define SUN4I_CSI1_INT_ENABLE		0X030
#define SUN4I_CSI1_INT_STATUS		0X034
#define SUN4I_CSI1_HSIZE		0X040
#define SUN4I_CSI1_VSIZE		0X044
#define SUN4I_CSI1_STRIDE		0X048

static void __maybe_unused sun4i_csi1_write(struct sun4i_csi1 *csi,
					    int address, uint32_t value)
{
	writel(value, csi->mmio + address);
}

static uint32_t __maybe_unused sun4i_csi1_read(struct sun4i_csi1 *csi,
					       int address)
{
	return readl(csi->mmio + address);
}

static void __maybe_unused sun4i_csi1_mask(struct sun4i_csi1 *csi, int address,
					   uint32_t value, uint32_t mask)
{
	uint32_t temp = readl(csi->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, csi->mmio + address);
}

static void __maybe_unused sun4i_registers_print(struct sun4i_csi1 *csi)
{
	pr_info("SUN4I_CSI1_ENABLE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_ENABLE));
	pr_info("SUN4I_CSI1_CONFIG: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_CONFIG));
	pr_info("SUN4I_CSI1_CAPTURE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_CAPTURE));
	pr_info("SUN4I_CSI1_SCALE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_SCALE));
	pr_info("SUN4I_CSI1_FIFO0_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO0_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO0_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO0_BUFFER_B));
	pr_info("SUN4I_CSI1_FIFO1_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO1_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO1_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO1_BUFFER_B));
	pr_info("SUN4I_CSI1_FIFO2_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO2_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO2_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_FIFO2_BUFFER_B));
	pr_info("SUN4I_CSI1_BUFFER_CONTROL: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_BUFFER_CONTROL));
	pr_info("SUN4I_CSI1_BUFFER_STATUS: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_BUFFER_STATUS));
	pr_info("SUN4I_CSI1_INT_ENABLE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_INT_ENABLE));
	pr_info("SUN4I_CSI1_INT_STATUS: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_INT_STATUS));
	pr_info("SUN4I_CSI1_HSIZE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_HSIZE));
	pr_info("SUN4I_CSI1_VSIZE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_VSIZE));
	pr_info("SUN4I_CSI1_STRIDE: 0x%02X\n",
		sun4i_csi1_read(csi, SUN4I_CSI1_STRIDE));
}

static int sun4i_csi1_poweron(struct sun4i_csi1 *csi)
{
	struct device *dev = csi->dev;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	ret = reset_control_deassert(csi->reset);
	if (ret) {
		dev_err(dev, "%s(): reset_control_deassert() failed: %d.\n",
			__func__, ret);
		goto err_reset;
	}

	ret = clk_prepare_enable(csi->clk_bus);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(bus) failed: %d.\n",
			__func__, ret);
		goto err_bus;
	}

	ret = clk_prepare_enable(csi->clk_ram);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(ram) failed: %d.\n",
			__func__, ret);
		goto err_ram;
	}

	clk_set_rate(csi->clk_module, 24000000);
	ret = clk_prepare_enable(csi->clk_module);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(module) failed: %d.\n",
			__func__, ret);
		goto err_module;
	}

	/* enable module */
	sun4i_csi1_mask(csi, SUN4I_CSI1_ENABLE, 0x01, 0x01);

	return 0;

 err_module:
	clk_disable_unprepare(csi->clk_ram);
 err_ram:
	clk_disable_unprepare(csi->clk_bus);
 err_bus:
	reset_control_assert(csi->reset);
 err_reset:
	return ret;
}

/*
 * We do not bother with checking return values here, we are powering
 * down anyway.
 */
static int sun4i_csi1_poweroff(struct sun4i_csi1 *csi)
{
	struct device *dev = csi->dev;

	dev_info(dev, "%s();\n", __func__);

	/* reset and disable module */
	sun4i_csi1_mask(csi, SUN4I_CSI1_ENABLE, 0, 0x01);

	clk_disable_unprepare(csi->clk_module);

	clk_disable_unprepare(csi->clk_ram);

	clk_disable_unprepare(csi->clk_bus);

	reset_control_assert(csi->reset);

	return 0;
}

static irqreturn_t sun4i_csi1_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int sun4i_csi1_resources_get(struct sun4i_csi1 *csi,
				    struct platform_device *platform_dev)
{
	struct device *dev = csi->dev;
	struct resource *resource;
	int irq, ret;

	csi->clk_bus = devm_clk_get(dev, "bus");
	if (IS_ERR(csi->clk_bus)) {
		dev_err(dev, "%s(): devm_clk_get(bus) failed: %ld.\n",
			__func__, PTR_ERR(csi->clk_bus));
		return PTR_ERR(csi->clk_bus);
	}

	csi->clk_module = devm_clk_get(dev, "mod");
	if (IS_ERR(csi->clk_module)) {
		dev_err(dev, "%s(): devm_clk_get(module) failed: %ld.\n",
			__func__, PTR_ERR(csi->clk_module));
		return PTR_ERR(csi->clk_module);
	}

	csi->clk_ram = devm_clk_get(dev, "ram");
	if (IS_ERR(csi->clk_ram)) {
		dev_err(dev, "%s(): devm_clk_get(ram) failed: %ld.\n",
			__func__, PTR_ERR(csi->clk_ram));
		return PTR_ERR(csi->clk_ram);
	}

	csi->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(csi->reset)) {
		dev_err(dev, "%s(): devm_reset_control_get() failed: %ld.\n",
			__func__, PTR_ERR(csi->reset));
		return PTR_ERR(csi->reset);
	}

	resource = platform_get_resource(platform_dev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(dev, "%s(): platform_get_resource() failed.\n",
			__func__);
		return EINVAL;
	}

	csi->mmio = devm_ioremap_resource(dev, resource);
	if (IS_ERR(csi->mmio)) {
		dev_err(dev, "%s(): devm_ioremap_resource() failed: %ld.\n",
			__func__, PTR_ERR(csi->mmio));
		return PTR_ERR(csi->mmio);
	}

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		dev_err(dev, "%s(): platform_get_irq() failed: %d.\n",
			__func__, -irq);
		return -irq;
	}

	ret = devm_request_irq(dev, irq, sun4i_csi1_isr, 0, MODULE_NAME, csi);
	if (ret) {
		dev_err(dev, "%s(): devm_request_irq() failed: %d.\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

/*
 * We might want to power up/down depending on actual usage though.
 */
static int sun4i_csi1_resume(struct device *dev)
{
	struct sun4i_csi1 *csi = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!csi->powered)
		return 0;

	return sun4i_csi1_poweron(csi);
}

static int sun4i_csi1_suspend(struct device *dev)
{
	struct sun4i_csi1 *csi = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!csi->powered)
		return 0;

	return sun4i_csi1_poweroff(csi);
}

static const struct dev_pm_ops sun4i_csi1_pm_ops = {
	SET_RUNTIME_PM_OPS(sun4i_csi1_suspend, sun4i_csi1_resume, NULL)
};

static int sun4i_csi1_v4l2_initialize(struct sun4i_csi1 *csi)
{
	struct device *dev = csi->dev;
	int ret;

	ret = v4l2_device_register(dev, csi->v4l2_dev);
	if (ret) {
		dev_err(dev, "%s(): v4l2_device_register() failed: %d.\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int sun4i_csi1_v4l2_cleanup(struct sun4i_csi1 *csi)
{
	v4l2_device_unregister(csi->v4l2_dev);

	return 0;
}

static int sun4i_csi1_probe(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct sun4i_csi1 *csi;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	csi = devm_kzalloc(dev, sizeof(struct sun4i_csi1), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;
	csi->dev = dev;

	ret = sun4i_csi1_resources_get(csi, platform_dev);
	if (ret)
		return ret;

	platform_set_drvdata(platform_dev, csi);

	ret =  sun4i_csi1_poweron(csi);
	if (ret)
		return ret;
	csi->powered = true;

	sun4i_registers_print(csi);

	ret = sun4i_csi1_v4l2_initialize(csi);
	if (ret)
		return ret;

	return 0;
}

static int sun4i_csi1_remove(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct sun4i_csi1 *csi = platform_get_drvdata(platform_dev);
	int ret;

	dev_info(dev, "%s();\n", __func__);

	ret = sun4i_csi1_v4l2_cleanup(csi);
	if (ret)
		return ret;

	ret =  sun4i_csi1_poweroff(csi);
	if (ret)
		return ret;
	csi->powered = false;

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
		.pm = &sun4i_csi1_pm_ops,
	},
};
module_platform_driver(sun4i_csi1_platform_driver);

MODULE_DESCRIPTION("Allwinner A10/A20 CMOS Sensor Interface 1 V4L2 driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
