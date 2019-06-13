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
#include <linux/mutex.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>

#define MODULE_NAME	"sun4i-csi1"

struct sun4i_csi1_buffer {
	struct vb2_v4l2_buffer v4l2_buffer;
	struct list_head list;
	dma_addr_t dma_addr;
};

struct sun4i_csi1 {
	struct device *dev;

	struct clk *clk_bus;
	struct clk *clk_module;
	struct clk *clk_ram;
	struct reset_control *reset;

	void __iomem *mmio;

	bool powered;

	struct v4l2_device v4l2_dev[1];
	struct v4l2_format v4l2_format[1];

	/*
	 * We need these values as allwinners CSI does not take in DE, and
	 * needs to be told the distance between h/vsync and the start of
	 * pixel data.
	 * When we have the adv7611 running, we should read this information
	 * from the ADV7611 registers
	 */
	int hdisplay_start;
	int vdisplay_start;

	/*
	 * This too needs to be preset, and will be read from the adv7611
	 * in future.
	 */
	bool hsync_polarity;
	bool vsync_polarity;

	struct vb2_queue vb2_queue[1];
	struct mutex vb2_queue_lock[1];

	struct video_device slashdev[1];

	/*
	 * This is both a lock on the buffer list, and on the registers,
	 * as playing with buffers invariably means updating at least
	 * buffer addresses in the registers.
	 */
	struct spinlock buffer_lock[1];
	struct list_head buffer_list[1];

	struct sun4i_csi1_buffer *buffers[2];
	uint64_t sequence;
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

static void __maybe_unused sun4i_csi1_write_spin(struct sun4i_csi1 *csi,
						 int address, uint32_t value)
{
	unsigned long flags;

	spin_lock_irqsave(csi->buffer_lock, flags);

	writel(value, csi->mmio + address);

	spin_unlock_irqrestore(csi->buffer_lock, flags);
}

static uint32_t __maybe_unused sun4i_csi1_read(struct sun4i_csi1 *csi,
					       int address)
{
	return readl(csi->mmio + address);
}

static uint32_t __maybe_unused sun4i_csi1_read_spin(struct sun4i_csi1 *csi,
						    int address)
{
	unsigned long flags;
	uint32_t ret;

	spin_lock_irqsave(csi->buffer_lock, flags);

	ret = readl(csi->mmio + address);

	spin_unlock_irqrestore(csi->buffer_lock, flags);

	return ret;
}

static void __maybe_unused sun4i_csi1_mask(struct sun4i_csi1 *csi, int address,
					   uint32_t value, uint32_t mask)
{
	uint32_t temp = readl(csi->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, csi->mmio + address);
}

static void __maybe_unused sun4i_csi1_mask_spin(struct sun4i_csi1 *csi,
						int address,
						uint32_t value, uint32_t mask)
{
	unsigned long flags;
	uint32_t temp;

	spin_lock_irqsave(csi->buffer_lock, flags);

	temp = readl(csi->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, csi->mmio + address);

	spin_unlock_irqrestore(csi->buffer_lock, flags);
}

static void __maybe_unused sun4i_registers_print(struct sun4i_csi1 *csi)
{
	pr_info("SUN4I_CSI1_ENABLE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_ENABLE));
	pr_info("SUN4I_CSI1_CONFIG: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_CONFIG));
	pr_info("SUN4I_CSI1_CAPTURE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_CAPTURE));
	pr_info("SUN4I_CSI1_SCALE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_SCALE));
	pr_info("SUN4I_CSI1_FIFO0_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO0_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO0_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO0_BUFFER_B));
	pr_info("SUN4I_CSI1_FIFO1_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO1_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO1_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO1_BUFFER_B));
	pr_info("SUN4I_CSI1_FIFO2_BUFFER_A: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO2_BUFFER_A));
	pr_info("SUN4I_CSI1_FIFO2_BUFFER_B: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_FIFO2_BUFFER_B));
	pr_info("SUN4I_CSI1_BUFFER_CONTROL: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_BUFFER_CONTROL));
	pr_info("SUN4I_CSI1_BUFFER_STATUS: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_BUFFER_STATUS));
	pr_info("SUN4I_CSI1_INT_ENABLE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_INT_ENABLE));
	pr_info("SUN4I_CSI1_INT_STATUS: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_INT_STATUS));
	pr_info("SUN4I_CSI1_HSIZE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_HSIZE));
	pr_info("SUN4I_CSI1_VSIZE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_VSIZE));
	pr_info("SUN4I_CSI1_STRIDE: 0x%02X\n",
		sun4i_csi1_read_spin(csi, SUN4I_CSI1_STRIDE));
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
	sun4i_csi1_mask_spin(csi, SUN4I_CSI1_ENABLE, 0x01, 0x01);

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
	sun4i_csi1_mask_spin(csi, SUN4I_CSI1_ENABLE, 0, 0x01);

	clk_disable_unprepare(csi->clk_module);

	clk_disable_unprepare(csi->clk_ram);

	clk_disable_unprepare(csi->clk_bus);

	reset_control_assert(csi->reset);

	return 0;
}

/*
 * Called from ISR.
 */
static void sun4i_csi1_frame_done(struct sun4i_csi1 *csi)
{
	struct v4l2_pix_format *pixel = &csi->v4l2_format->fmt.pix;
	int offset = pixel->width * pixel->height;
	struct sun4i_csi1_buffer *old, *new;
	uint64_t sequence;
	uint32_t dma_addr;
	int index;

	spin_lock(csi->buffer_lock);

	sequence = csi->sequence;
	csi->sequence++;

	index = sequence & 0x01;

	old = csi->buffers[index];

	new = list_first_entry(csi->buffer_list,
			       struct sun4i_csi1_buffer, list);
	list_del_init(&new->list);
	dma_addr = new->dma_addr;
	csi->buffers[index] = new;

	if (!index) {
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO0_BUFFER_A, dma_addr);
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO1_BUFFER_A, dma_addr + offset);
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO2_BUFFER_A, dma_addr + 2 * offset);
	} else {
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO0_BUFFER_B, dma_addr);
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO1_BUFFER_B, dma_addr + offset);
		sun4i_csi1_write(csi, SUN4I_CSI1_FIFO2_BUFFER_B, dma_addr + 2 * offset);
	}

	spin_unlock(csi->buffer_lock);

	old->v4l2_buffer.vb2_buf.timestamp = ktime_get_ns();
	old->v4l2_buffer.sequence = sequence;
	vb2_buffer_done(&old->v4l2_buffer.vb2_buf, VB2_BUF_STATE_DONE);
}

static irqreturn_t sun4i_csi1_isr(int irq, void *dev_id)
{
	struct sun4i_csi1 *csi = (struct sun4i_csi1 *) dev_id;
	uint32_t value;

	spin_lock(csi->buffer_lock);

	value = sun4i_csi1_read(csi, SUN4I_CSI1_INT_STATUS);

	/* ack. */
	sun4i_csi1_write(csi, SUN4I_CSI1_INT_STATUS, value);

	spin_unlock(csi->buffer_lock);

	if (value & 0x02)
		sun4i_csi1_frame_done(csi);

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

/*
 * We currently only care about 24bit RGB.
 */
static void sun4i_csi1_format_initialize(struct sun4i_csi1 *csi,
					 int width, int height,
					 int hdisplay_start,
					 int vdisplay_start,
					 bool hsync_polarity,
					 bool vsync_polarity)
{
	struct v4l2_pix_format *pixel = &csi->v4l2_format->fmt.pix;

	csi->v4l2_format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pixel->pixelformat = V4L2_PIX_FMT_RGB24;

	pixel->width = width;
	pixel->height = height;

	pixel->bytesperline = pixel->width * 3;
	pixel->sizeimage = pixel->bytesperline * pixel->height;

	pixel->field = V4L2_FIELD_NONE;

	pixel->colorspace = V4L2_COLORSPACE_RAW;
	pixel->quantization = V4L2_QUANTIZATION_DEFAULT;
	pixel->xfer_func = V4L2_XFER_FUNC_NONE;

	csi->hdisplay_start = hdisplay_start;
	csi->vdisplay_start = vdisplay_start;
	csi->hsync_polarity = hsync_polarity;
	csi->vsync_polarity = vsync_polarity;
}

/*
 * This is second guessing v4l2 infrastructure, and to properly tell us when
 * there are any buffers still present.
 *
 * This whole buffer management stuff has me wondering though, why can we not
 * just ask vb2 infrastructure for the next queued buffer, and support us with
 * double or triple buffering? Perhaps it does, and it just not often used?
 */
static void sun4i_csi1_buffer_list_clear(struct sun4i_csi1 *csi)
{
	while (1) {
		struct sun4i_csi1_buffer *buffer;
		unsigned long flags;

		spin_lock_irqsave(csi->buffer_lock, flags);

		buffer = list_first_entry_or_null(csi->buffer_list,
						  struct sun4i_csi1_buffer,
						  list);
		if (buffer)
			list_del(&buffer->list);

		spin_unlock_irqrestore(csi->buffer_lock, flags);

		if (!buffer)
			break;

		vb2_buffer_done(&buffer->v4l2_buffer.vb2_buf,
				VB2_BUF_STATE_ERROR);

		dev_err(csi->dev, "%s: Cleared buffer 0x%px from the queue.\n",
			__func__, &buffer->v4l2_buffer.vb2_buf);
	}
}

static int sun4i_csi1_queue_setup(struct vb2_queue *queue,
				  unsigned int *buffer_count,
				  unsigned int *planes_count,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(queue);

	if (buffer_count)
		dev_info(csi->dev, "%s(%d);\n", __func__, *buffer_count);
	else
		dev_info(csi->dev, "%s();\n", __func__);

	if (*planes_count) {
		dev_err(csi->dev, "%s: invalid planes_count pointer.\n",
			__func__);
		return -EINVAL;
	}

	*planes_count = 1;
	sizes[0] = csi->v4l2_format->fmt.pix.sizeimage;

	sun4i_csi1_buffer_list_clear(csi);

	return 0;
}

static int sun4i_csi1_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);
	struct sun4i_csi1_buffer *buffer =
		container_of(v4l2_buffer, struct sun4i_csi1_buffer,
			     v4l2_buffer);

	vb2_set_plane_payload(vb2_buffer, 0,
			      csi->v4l2_format->fmt.pix.sizeimage);

	/* make very sure that this is properly initialized */
	INIT_LIST_HEAD(&buffer->list);
	buffer->dma_addr = vb2_dma_contig_plane_dma_addr(vb2_buffer, 0);

	return 0;
}

static void sun4i_csi1_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);
	struct sun4i_csi1_buffer *buffer =
		container_of(v4l2_buffer, struct sun4i_csi1_buffer,
			     v4l2_buffer);
	unsigned long flags;

	spin_lock_irqsave(csi->buffer_lock, flags);
	list_add_tail(&buffer->list, csi->buffer_list);
	spin_unlock_irqrestore(csi->buffer_lock, flags);
}

static void sun4i_csi1_engine_start(struct sun4i_csi1 *csi)
{
	struct v4l2_pix_format *pixel = &csi->v4l2_format->fmt.pix;
	int offset = pixel->width * pixel->height;
	unsigned long flags;

	spin_lock_irqsave(csi->buffer_lock, flags);

	csi->sequence = 0;

	csi->buffers[0] = list_first_entry(csi->buffer_list,
					   struct sun4i_csi1_buffer, list);
	list_del_init(&csi->buffers[0]->list);
	csi->buffers[1] = list_first_entry(csi->buffer_list,
					   struct sun4i_csi1_buffer, list);
	list_del_init(&csi->buffers[1]->list);

	/* set input format: yuv444 */
	sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0x00400000, 0x00700000);

	/* set output format: field planar yuv444 */
	sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0x000C0000, 0x000F0000);

	if (csi->vsync_polarity) /* positive */
		sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0x04, 0x04);
	else
		sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0, 0x04);
	if (csi->hsync_polarity) /* positive */
		sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0x02, 0x02);
	else
		sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0, 0x02);

	/* PCLK is low */
	sun4i_csi1_mask(csi, SUN4I_CSI1_CONFIG, 0, 0x01);

	/* set buffer addresses */
	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO0_BUFFER_A, csi->buffers[0]->dma_addr);
	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO1_BUFFER_A, csi->buffers[0]->dma_addr + offset);
	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO2_BUFFER_A, csi->buffers[0]->dma_addr + 2 * offset);

	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO0_BUFFER_B, csi->buffers[1]->dma_addr);
	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO1_BUFFER_B, csi->buffers[1]->dma_addr + offset);
	sun4i_csi1_write(csi, SUN4I_CSI1_FIFO2_BUFFER_B, csi->buffers[1]->dma_addr + 2 * offset);

	/* enable double buffering, and select buffer A first */
	sun4i_csi1_write(csi, SUN4I_CSI1_BUFFER_CONTROL, 0x01);

	/* enable interrupts: frame done */
	sun4i_csi1_mask(csi, SUN4I_CSI1_INT_ENABLE, 0x02, 0x02);

	sun4i_csi1_mask(csi, SUN4I_CSI1_HSIZE, pixel->width << 16, 0x1FFF0000);
	sun4i_csi1_mask(csi, SUN4I_CSI1_HSIZE, csi->hdisplay_start, 0x1FFF);

	sun4i_csi1_mask(csi, SUN4I_CSI1_VSIZE, pixel->height << 16, 0x1FFF0000);
	sun4i_csi1_mask(csi, SUN4I_CSI1_VSIZE, csi->vdisplay_start, 0x1FFF);

	//sun4i_csi1_mask(csi, SUN4I_CSI1_STRIDE, pixel->bytesperline, 0x1FFF);
	sun4i_csi1_mask(csi, SUN4I_CSI1_STRIDE, pixel->width, 0x1FFF);

	/* start. */
	sun4i_csi1_mask(csi, SUN4I_CSI1_CAPTURE, 0x02, 0x02);

	spin_unlock_irqrestore(csi->buffer_lock, flags);
}

static void sun4i_csi1_engine_stop(struct sun4i_csi1 *csi)
{
	sun4i_csi1_write_spin(csi, SUN4I_CSI1_CAPTURE, 0);
}

static int sun4i_csi1_streaming_start(struct vb2_queue *queue, unsigned int count)
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(queue);
	int ret;

	dev_info(csi->dev, "%s();\n", __func__);

	ret =  sun4i_csi1_poweron(csi);
	if (ret)
		return ret;
	csi->powered = true;

	sun4i_registers_print(csi);

	sun4i_csi1_engine_start(csi);

	dev_info(csi->dev, "After engine start:\n");

	sun4i_registers_print(csi);

	return 0;
}

static void sun4i_csi1_buffers_mark_done(struct vb2_queue *queue)
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(queue);
	int i;

	dev_info(csi->dev, "%s(%d);\n", __func__, queue->num_buffers);

	for (i = 0; i < queue->num_buffers; i++) {
		struct vb2_buffer *vb2_buffer = queue->bufs[i];
		struct vb2_v4l2_buffer *v4l2_buffer =
			to_vb2_v4l2_buffer(vb2_buffer);
		struct sun4i_csi1_buffer *buffer =
			container_of(v4l2_buffer, struct sun4i_csi1_buffer,
				     v4l2_buffer);
		unsigned long flags;

		spin_lock_irqsave(csi->buffer_lock, flags);
		list_del(&buffer->list);
		spin_unlock_irqrestore(csi->buffer_lock, flags);

		/* only disable active buffers, otherwise we get a WARN_ON() */
		if (vb2_buffer->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb2_buffer, VB2_BUF_STATE_ERROR);
	}
}

static void sun4i_csi1_streaming_stop(struct vb2_queue *queue)
{
	struct sun4i_csi1 *csi = vb2_get_drv_priv(queue);

	dev_info(csi->dev, "%s();\n", __func__);

	sun4i_csi1_engine_stop(csi);

	sun4i_registers_print(csi);

	sun4i_csi1_buffers_mark_done(queue);

	sun4i_csi1_buffer_list_clear(csi);

	sun4i_csi1_poweroff(csi);
	csi->powered = false;
}

static const struct vb2_ops sun4i_csi1_vb2_queue_ops = {
	.queue_setup = sun4i_csi1_queue_setup,
	.buf_prepare = sun4i_csi1_buffer_prepare,
	.buf_queue = sun4i_csi1_buffer_queue,
	.start_streaming = sun4i_csi1_streaming_start,
	.stop_streaming = sun4i_csi1_streaming_stop,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int sun4i_csi1_vb2_queue_initialize(struct sun4i_csi1 *csi)
{
	struct vb2_queue *queue = csi->vb2_queue;
	int ret;

	queue->drv_priv = csi;
	queue->dev = csi->dev;

	queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	queue->io_modes = VB2_MMAP | VB2_DMABUF;
	queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	queue->min_buffers_needed = 3;
	queue->buf_struct_size = sizeof(struct sun4i_csi1_buffer);

	queue->ops = &sun4i_csi1_vb2_queue_ops;
	queue->mem_ops = &vb2_dma_contig_memops;

	mutex_init(csi->vb2_queue_lock);
	queue->lock = csi->vb2_queue_lock;

	spin_lock_init(csi->buffer_lock);
	INIT_LIST_HEAD(csi->buffer_list);

	ret = vb2_queue_init(queue);
	if (ret) {
		dev_err(csi->dev, "%s(): vb2_queue_init() failed: %d\n",
			__func__, ret);
		mutex_destroy(csi->vb2_queue_lock);
		return ret;
	}

	return 0;
}

static void sun4i_csi1_vb2_queue_free(struct sun4i_csi1 *csi)
{
	struct vb2_queue *queue = csi->vb2_queue;

	vb2_queue_release(queue);
	mutex_destroy(csi->vb2_queue_lock);
}

static const struct v4l2_file_operations sun4i_csi1_slashdev_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int sun4i_csi1_ioctl_capability_query(struct file *file, void *handle,
					     struct v4l2_capability *capability)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	strscpy(capability->driver, "sun4i_csi1", sizeof(capability->driver));
	strscpy(capability->card, csi->slashdev->name,
		sizeof(capability->card));

	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", csi->dev->of_node->name);

	return 0;
}

static int sun4i_csi1_ioctl_format_enumerate(struct file *file, void *handle,
					     struct v4l2_fmtdesc *descriptor)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	if (descriptor->index > 0)
		return -EINVAL;

	descriptor->pixelformat = V4L2_PIX_FMT_RGB24;

	return 0;
}

static int sun4i_csi1_ioctl_format_get(struct file *file, void *handle,
			       struct v4l2_format *format)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	*format = csi->v4l2_format[0];

	return 0;
}

static int sun4i_csi1_format_test(struct v4l2_format *old,
				  struct v4l2_format *new)
{
	struct v4l2_pix_format *pixel_old = &old->fmt.pix;
	struct v4l2_pix_format *pixel_new = &new->fmt.pix;

	if (old->type != new->type)
		return -EINVAL;

	if ((pixel_old->pixelformat != pixel_new->pixelformat) ||
	    (pixel_old->width != pixel_new->width) ||
	    (pixel_old->height != pixel_new->height) ||
	    (pixel_old->bytesperline != pixel_new->bytesperline) ||
	    (pixel_old->sizeimage != pixel_new->sizeimage) ||
	    (pixel_old->field != pixel_new->field) ||
	    (pixel_old->colorspace != pixel_new->colorspace) ||
	    (pixel_old->quantization != pixel_new->quantization) ||
	    (pixel_old->xfer_func != pixel_new->xfer_func))
		return -EINVAL;

	return 0;
}

static int sun4i_csi1_ioctl_format_set(struct file *file, void *handle,
				       struct v4l2_format *format)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	return sun4i_csi1_format_test(csi->v4l2_format, format);
}

static int sun4i_csi1_ioctl_format_try(struct file *file, void *handle,
				       struct v4l2_format *format)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	return sun4i_csi1_format_test(csi->v4l2_format, format);
}

static int sun4i_csi1_ioctl_input_enumerate(struct file *file, void *handle,
					    struct v4l2_input *input)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	if (input->index)
		return -EINVAL;

	strscpy(input->name, "direct", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int sun4i_csi1_ioctl_input_set(struct file *file, void *handle,
				      unsigned int input)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	if (input)
		return -EINVAL;

	return 0;
}

static int sun4i_csi1_ioctl_input_get(struct file *file, void *handle,
				      unsigned int *input)
{
	struct sun4i_csi1 *csi = video_drvdata(file);

	dev_info(csi->dev, "%s();\n", __func__);

	*input = 0;

	return 0;
}

static const struct v4l2_ioctl_ops sun4i_csi1_ioctl_ops = {
	.vidioc_querycap = sun4i_csi1_ioctl_capability_query,
	.vidioc_enum_fmt_vid_cap = sun4i_csi1_ioctl_format_enumerate,
	.vidioc_g_fmt_vid_cap = sun4i_csi1_ioctl_format_get,
	.vidioc_s_fmt_vid_cap = sun4i_csi1_ioctl_format_set,
	.vidioc_try_fmt_vid_cap = sun4i_csi1_ioctl_format_try,

	.vidioc_enum_input = sun4i_csi1_ioctl_input_enumerate,
	.vidioc_s_input = sun4i_csi1_ioctl_input_set,
	.vidioc_g_input = sun4i_csi1_ioctl_input_get,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int sun4i_csi1_slashdev_initialize(struct sun4i_csi1 *csi)
{
	struct video_device *slashdev = csi->slashdev;
	int ret;

	video_set_drvdata(slashdev, csi);
	strscpy(slashdev->name, KBUILD_MODNAME, sizeof(slashdev->name));

	slashdev->vfl_type = VFL_TYPE_GRABBER;
	slashdev->vfl_dir = VFL_DIR_RX;
	slashdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE;

	slashdev->v4l2_dev = csi->v4l2_dev;
	slashdev->queue = csi->vb2_queue;
	slashdev->lock = csi->vb2_queue_lock;

	slashdev->release = video_device_release_empty;
	slashdev->fops = &sun4i_csi1_slashdev_fops;
	slashdev->ioctl_ops = &sun4i_csi1_ioctl_ops;

	ret = video_register_device(slashdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(csi->dev, "%s(): video_register_device failed: %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static void sun4i_csi1_slashdev_free(struct sun4i_csi1 *csi)
{
	struct video_device *slashdev = csi->slashdev;

	video_unregister_device(slashdev);
}

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

	/*
	 * vesa 640x480@60Hz: 640 656 752 800  480 490 492 525
	 * xtotal - xsync_start = xdisplay_start
	 * h: 800 - 656 = 144
	 * v: 525 - 492 = 33
	 */
	sun4i_csi1_format_initialize(csi, 640, 480, 144, 33, true, true);

	ret = sun4i_csi1_vb2_queue_initialize(csi);
	if (ret)
		goto error;

	ret = sun4i_csi1_slashdev_initialize(csi);
	if (ret)
		goto error;

	return 0;

 error:
	sun4i_csi1_vb2_queue_free(csi);
	v4l2_device_unregister(csi->v4l2_dev);
	return ret;
}

static int sun4i_csi1_v4l2_cleanup(struct sun4i_csi1 *csi)
{
	sun4i_csi1_slashdev_free(csi);
	sun4i_csi1_vb2_queue_free(csi);
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
