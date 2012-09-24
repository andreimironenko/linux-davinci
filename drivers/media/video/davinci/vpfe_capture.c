/*
 * Copyright (C) 2011 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Driver name : VPFE Capture driver
 *    VPFE Capture driver allows applications to capture and stream video
 *    frames on DaVinci SoCs (DM6446, DM355 etc) from a YUV source such as
 *    TVP5146 or  Raw Bayer RGB image data from an image sensor
 *    such as Microns' MT9T001, MT9T031 etc.
 *
 *    These SoCs have, in common, a Video Processing Subsystem (VPSS) that
 *    consists of a Video Processing Front End (VPFE) for capturing
 *    video/raw image data and Video Processing Back End (VPBE) for displaying
 *    YUV data through an in-built analog encoder or Digital LCD port. This
 *    driver is for capture through VPFE. A typical EVM using these SoCs have
 *    following high level configuration.
 *
 *    decoder(TVP5146/		YUV/
 *	MT9T001)   -->  Raw Bayer RGB ---> MUX -> VPFE (CCDC/ISIF)
 *			data input              |      |
 *							V      |
 *						      SDRAM    |
 *							       V
 *							   Image Processor
 *							       |
 *							       V
 *							     SDRAM
 *    The data flow happens from a decoder connected to the VPFE over a
 *    YUV embedded (BT.656/BT.1120) or separate sync or raw bayer rgb interface
 *    and to the input of VPFE through an optional MUX (if more inputs are
 *    to be interfaced on the EVM). The input data is first passed through
 *    CCDC (CCD Controller, a.k.a Image Sensor Interface, ISIF). The CCDC
 *    does very little or no processing on YUV data and does pre-process Raw
 *    Bayer RGB data through modules such as Defect Pixel Correction (DFC)
 *    Color Space Conversion (CSC), data gain/offset etc. After this, data
 *    can be written to SDRAM or can be connected to the image processing
 *    block such as IPIPE (on DM355/DM365 only).
 *
 *    Features supported
 *		- MMAP IO
 *		- USERPTR IO
 *		- Capture using TVP5146 over BT.656
 *		- support for interfacing decoders using sub device model
 *		- Work with DM365 or DM355 or DM6446 CCDC to do Raw Bayer
 *		  RGB/YUV data capture to SDRAM.
 *		- Chaining of Image Processor
 *		- SINGLE-SHOT mode
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <media/v4l2-common.h>
#include <media/v4l2-mediabus.h>

#include <media/media-entity.h>
#include <media/media-device.h>


#include <media/davinci/videohd.h>
#include <media/davinci/vpfe_capture.h>

#include <mach/cputype.h>

#include "ccdc_hw_device.h"

#define HD_IMAGE_SIZE		(1920 * 1080 * 2)
#define PAL_IMAGE_SIZE		(720 * 576 * 2)
#define SECOND_IMAGE_SIZE_MAX	(640 * 480 * 2)


static int debug;
static u32 numbuffers = 3;
static u32 bufsize = HD_IMAGE_SIZE + SECOND_IMAGE_SIZE_MAX;
static int interface;
static u32 cont_bufoffset;
static u32 cont_bufsize;

module_param(interface, bool, S_IRUGO);
module_param(numbuffers, uint, S_IRUGO);
module_param(bufsize, uint, S_IRUGO);
module_param(debug, bool, 0644);
module_param(cont_bufoffset, uint, S_IRUGO);
module_param(cont_bufsize, uint, S_IRUGO);

/**
 * VPFE capture can be used for capturing video such as from TVP5146 or TVP7002
 * and for capture raw bayer data from camera sensors such as mt9p031. At this
 * point there is problem in co-existence of mt9p031 and tvp5146 due to i2c
 * address collision. So set the variable below from bootargs to do either video
 * capture or camera capture.
 * interface = 0 - video capture (from TVP514x or such),
 * interface = 1 - Camera capture (from mt9p031 or such)
 * Re-visit this when we fix the co-existence issue
 */
MODULE_PARM_DESC(interface, "interface 0-1 (default:0)");
MODULE_PARM_DESC(numbuffers, "buffer count (default:3)");
MODULE_PARM_DESC(bufsize, "buffer size in bytes, (default:4147200 bytes)");
MODULE_PARM_DESC(debug, "Debug level 0-1");
MODULE_PARM_DESC(cont_bufoffset, "Capture buffer offset (default 0)");
MODULE_PARM_DESC(cont_bufsize, "Capture buffer size (default 0)");

MODULE_DESCRIPTION("VPFE Video for Linux Capture Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");

/* lock for accessing ccdc information */
static DEFINE_MUTEX(ccdc_lock);

void mbus_to_pix(const struct v4l2_mbus_framefmt *mbus,
			   struct v4l2_pix_format *pix)
{
	/* TODO: revisit */
	switch (mbus->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_UYVY;
		pix->bytesperline = pix->width * 2;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_UYVY;
		pix->bytesperline = pix->width * 2;
		break;
	case V4L2_MBUS_FMT_YUYV10_1X20:
		pix->pixelformat = V4L2_PIX_FMT_UYVY;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				V4L2_MBUS_FMT_YUYV10_2X10);
	} else {
		pix->field = V4L2_FIELD_NONE;
		/* assume V4L2_PIX_FMT_SBGGR8 */
		pix->pixelformat = V4L2_PIX_FMT_SBGGR8;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				V4L2_MBUS_FMT_SBGGR8_1X8);
	}

	/* if sub device supports g_mbus_fmt, override the defaults */
	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev,
			sdinfo->grp_id, video, g_mbus_fmt, &mbus_fmt);

	if (ret && ret != -ENOIOCTLCMD) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"error in getting g_mbus_fmt from sub device\n");
		return ret;
	}
	v4l2_fill_pix_format(pix, &mbus_fmt);
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;

	/* Sets the values in CCDC */
	ret = vpfe_config_ccdc_image_format(vpfe_dev);
	if (ret)
		return ret;

	/* Update the values of sizeimage and bytesperline */
	if (!ret) {
		pix->bytesperline = ccdc_dev->hw_ops.get_line_length();
		pix->sizeimage = pix->bytesperline * pix->height;
	}
	return ret;
}

static int vpfe_initialize_device(struct vpfe_device *vpfe_dev)
{
	int ret = 0;

	/* set first input of current subdevice as the current input */
	vpfe_dev->current_input = 0;

	/* set default standard */
	vpfe_dev->std_index = 0;

	/* Configure the default format information */
	ret = vpfe_config_image_format(vpfe_dev,
				&vpfe_standards[vpfe_dev->std_index].std_id);
	if (ret)
		return ret;

	/* now open the ccdc device to initialize it */
	mutex_lock(&ccdc_lock);
	if (NULL == ccdc_dev) {
		v4l2_err(&vpfe_dev->v4l2_dev, "ccdc device not registered\n");
		ret = -ENODEV;
		goto unlock;
	}

	if (!try_module_get(ccdc_dev->owner)) {
		v4l2_err(&vpfe_dev->v4l2_dev, "Couldn't lock ccdc module\n");
		ret = -ENODEV;
		goto unlock;
	}
	ret = ccdc_dev->hw_ops.open(vpfe_dev->pdev);
	if (!ret)
		vpfe_dev->initialized = 1;

	/* Clear all VPFE/CCDC interrupts */
	if (vpfe_dev->cfg->clr_intr)
		vpfe_dev->cfg->clr_intr(-1);

unlock:
	mutex_unlock(&ccdc_lock);
	return ret;
}

/*
 * vpfe_open : It creates object of file handle structure and
 * stores it in private_data  member of filepointer
 */
static int vpfe_open(struct file *file)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_open\n");

	if (!vpfe_dev->cfg->num_subdevs) {
		v4l2_err(&vpfe_dev->v4l2_dev, "No decoder registered\n");
		return -ENODEV;
	}

	/* Allocate memory for the file handle object */
	fh = kmalloc(sizeof(struct vpfe_fh), GFP_KERNEL);
	if (NULL == fh) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}
	/* store pointer to fh in private_data member of file */
	file->private_data = fh;
	fh->vpfe_dev = vpfe_dev;
	mutex_lock(&vpfe_dev->lock);
	/* If decoder is not initialized. initialize it */
	if (!vpfe_dev->initialized) {
		if (vpfe_initialize_device(vpfe_dev)) {
			mutex_unlock(&vpfe_dev->lock);
			return -ENODEV;
		}
	}
	/* Increment device usrs counter */
	vpfe_dev->usrs++;
	/* Set io_allowed member to false */
	fh->io_allowed = 0;
	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&vpfe_dev->prio, &fh->prio);
	mutex_unlock(&vpfe_dev->lock);
	return 0;
}

static void vpfe_schedule_next_buffer(struct vpfe_device *vpfe_dev)
{
	unsigned long addr;

	vpfe_dev->next_frm = list_entry(vpfe_dev->dma_queue.next,
					struct videobuf_buffer, queue);
	list_del(&vpfe_dev->next_frm->queue);
	vpfe_dev->next_frm->state = VIDEOBUF_ACTIVE;
	addr = videobuf_to_dma_contig(vpfe_dev->next_frm);

	ccdc_dev->hw_ops.setfbaddr(addr);
}

static void vpfe_schedule_bottom_field(struct vpfe_device *vpfe_dev)
{
	unsigned long addr;

	addr = videobuf_to_dma_contig(vpfe_dev->cur_frm);
	addr += vpfe_dev->field_off;
	ccdc_dev->hw_ops.setfbaddr(addr);
}

static void vpfe_process_buffer_complete(struct vpfe_device *vpfe_dev)
{
	struct timeval timevalue;

	do_gettimeofday(&timevalue);
	vpfe_dev->cur_frm->ts = timevalue;
	vpfe_dev->cur_frm->state = VIDEOBUF_DONE;
	vpfe_dev->cur_frm->size = vpfe_dev->fmt.fmt.pix.sizeimage;
	wake_up_interruptible(&vpfe_dev->cur_frm->done);
	vpfe_dev->cur_frm = vpfe_dev->next_frm;
}

/* ISR for VINT0*/
static irqreturn_t vpfe_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;
	enum v4l2_field field;
	int fid;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "\nStarting vpfe_isr...\n");
	field = vpfe_dev->fmt.fmt.pix.field;

	/* if streaming not started, don't do anything */
	if (!vpfe_dev->started)
		goto clear_intr;

	/* only for 6446 this will be applicable */
	if (NULL != ccdc_dev->hw_ops.reset)
		ccdc_dev->hw_ops.reset();

	if (field == V4L2_FIELD_NONE) {
		/* handle progressive frame capture */
		v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev,
			"frame format is progressive...\n");
		if (vpfe_dev->cur_frm != vpfe_dev->next_frm)
			vpfe_process_buffer_complete(vpfe_dev);
		goto clear_intr;
	}

	/* interlaced or TB capture check which field we are in hardware */
	fid = ccdc_dev->hw_ops.getfid();

	/* switch the software maintained field id */
	vpfe_dev->field_id ^= 1;
	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "field id = %x:%x.\n",
		fid, vpfe_dev->field_id);
	if (fid == vpfe_dev->field_id) {
		/* we are in-sync here,continue */
		if (fid == 0) {
			/*
			 * One frame is just being captured. If the next frame
			 * is available, release the current frame and move on
			 */
			if (vpfe_dev->cur_frm != vpfe_dev->next_frm)
				vpfe_process_buffer_complete(vpfe_dev);
			/*
			 * based on whether the two fields are stored
			 * interleavely or separately in memory, reconfigure
			 * the CCDC memory address
			 */
			if (field == V4L2_FIELD_SEQ_TB) {
				vpfe_schedule_bottom_field(vpfe_dev);
			}
			goto clear_intr;
		}
		/*
		 * if one field is just being captured configure
		 * the next frame get the next frame from the empty
		 * queue if no frame is available hold on to the
		 * current buffer
		 */
		spin_lock(&vpfe_dev->dma_queue_lock);
		if (!list_empty(&vpfe_dev->dma_queue) &&
		    vpfe_dev->cur_frm == vpfe_dev->next_frm)
			vpfe_schedule_next_buffer(vpfe_dev);
		spin_unlock(&vpfe_dev->dma_queue_lock);
	} else if (fid == 0) {
		/*
		 * out of sync. Recover from any hardware out-of-sync.
		 * May loose one frame
		 */
		vpfe_dev->field_id = fid;
	}
clear_intr:
	if (vpfe_dev->cfg->clr_intr)
		vpfe_dev->cfg->clr_intr(irq);

	return IRQ_HANDLED;
}

/* vdint1_isr - isr handler for VINT1 interrupt */
static irqreturn_t vdint1_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "\nInside vdint1_isr...\n");

	/* if streaming not started, don't do anything */
	if (!vpfe_dev->started) {
		if (vpfe_dev->cfg->clr_intr)
			vpfe_dev->cfg->clr_intr(irq);
		return IRQ_HANDLED;
	}

	spin_lock(&vpfe_dev->dma_queue_lock);
	if ((vpfe_dev->fmt.fmt.pix.field == V4L2_FIELD_NONE) &&
	    !list_empty(&vpfe_dev->dma_queue) &&
	    vpfe_dev->cur_frm == vpfe_dev->next_frm)
		vpfe_schedule_next_buffer(vpfe_dev);
	spin_unlock(&vpfe_dev->dma_queue_lock);

	if (vpfe_dev->cfg->clr_intr)
		vpfe_dev->cfg->clr_intr(irq);

	return IRQ_HANDLED;
}

static void vpfe_detach_irq(struct vpfe_device *vpfe_dev)
{
	enum ccdc_frmfmt frame_format;

	frame_format = ccdc_dev->hw_ops.get_frame_format();
	if (frame_format == CCDC_FRMFMT_PROGRESSIVE)
		free_irq(vpfe_dev->ccdc_irq1, vpfe_dev);
}

static int vpfe_attach_irq(struct vpfe_device *vpfe_dev)
{
	enum ccdc_frmfmt frame_format;

	frame_format = ccdc_dev->hw_ops.get_frame_format();
	if (frame_format == CCDC_FRMFMT_PROGRESSIVE) {
		return request_irq(vpfe_dev->ccdc_irq1, vdint1_isr,
				    IRQF_DISABLED, "vpfe_capture1",
				    vpfe_dev);
	}
	return 0;
}

/* vpfe_stop_ccdc_capture: stop streaming in ccdc/isif */
static void vpfe_stop_ccdc_capture(struct vpfe_device *vpfe_dev)
{
	vpfe_dev->started = 0;
	ccdc_dev->hw_ops.enable(0);
	if (ccdc_dev->hw_ops.enable_out_to_sdram)
		ccdc_dev->hw_ops.enable_out_to_sdram(0);
}

/*
 * vpfe_release : This function deletes buffer queue, frees the
 * buffers and the vpfe file  handle
 */
static int vpfe_release(struct file *file)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;
	struct vpfe_subdev_info *sdinfo;
	int ret;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_release\n");

	/* Get the device lock */
	mutex_lock(&vpfe_dev->lock);
	/* if this instance is doing IO */
	if (fh->io_allowed) {
		if (vpfe_dev->started) {
			sdinfo = vpfe_dev->current_subdev;
			ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev,
							 sdinfo->grp_id,
							 video, s_stream, 0);
			if (ret && (ret != -ENOIOCTLCMD))
				v4l2_err(&vpfe_dev->v4l2_dev,
				"stream off failed in subdev\n");
			vpfe_stop_ccdc_capture(vpfe_dev);
			vpfe_detach_irq(vpfe_dev);
			videobuf_streamoff(&vpfe_dev->buffer_queue);
		}
		vpfe_dev->io_usrs = 0;
		vpfe_dev->numbuffers = config_params.numbuffers;
	}

	/* Decrement device usrs counter */
	vpfe_dev->usrs--;
	/* Close the priority */
	v4l2_prio_close(&vpfe_dev->prio, fh->prio);
	/* If this is the last file handle */
	if (!vpfe_dev->usrs) {
		vpfe_dev->initialized = 0;
		if (ccdc_dev->hw_ops.close)
			ccdc_dev->hw_ops.close(vpfe_dev->pdev);
		module_put(ccdc_dev->owner);
	}
	mutex_unlock(&vpfe_dev->lock);
	file->private_data = NULL;
	/* Free memory allocated to file handle object */
	kfree(fh);
	return 0;
}

/*
 * vpfe_mmap : It is used to map kernel space buffers
 * into user spaces
 */
static int vpfe_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* Get the device object and file handle object */
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_mmap\n");

	return videobuf_mmap_mapper(&vpfe_dev->buffer_queue, vma);
}

/*
 * vpfe_poll: It is used for select/poll system call
 */
static unsigned int vpfe_poll(struct file *file, poll_table *wait)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_poll\n");

	if (vpfe_dev->started)
		return videobuf_poll_stream(file,
					    &vpfe_dev->buffer_queue, wait);
	return 0;
}

/* vpfe capture driver file operations */
static const struct v4l2_file_operations vpfe_fops = {
	.owner = THIS_MODULE,
	.open = vpfe_open,
	.release = vpfe_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vpfe_mmap,
	.poll = vpfe_poll
};

/*
 * vpfe_check_format()
 * This function adjust the input pixel format as per hardware
 * capabilities and update the same in pixfmt.
 * Following algorithm used :-
 *
 *	If given pixformat is not in the vpfe list of pix formats or not
 *	supported by the hardware, current value of pixformat in the device
 *	is used
 *	If given field is not supported, then current field is used. If field
 *	is different from current, then it is matched with that from sub device.
 *	Minimum height is 2 lines for interlaced or tb field and 1 line for
 *	progressive. Maximum height is clamped to active active lines of scan
 *	Minimum width is 32 bytes in memory and width is clamped to active
 *	pixels of scan.
 *	bytesperline is a multiple of 32.
 */
static const struct vpfe_pixel_format *
	vpfe_check_format(struct vpfe_device *vpfe_dev,
			  struct v4l2_pix_format *pixfmt)
{
	u32 min_height = 1, min_width = 32, max_width, max_height;
	const struct vpfe_pixel_format *vpfe_pix_fmt;
	u32 pix;
	int temp, found;

	vpfe_pix_fmt = vpfe_lookup_pix_format(pixfmt->pixelformat);
	if (NULL == vpfe_pix_fmt) {
		/*
		 * use current pixel format in the vpfe device. We
		 * will find this pix format in the table
		 */
		pixfmt->pixelformat = vpfe_dev->fmt.fmt.pix.pixelformat;
		vpfe_pix_fmt = vpfe_lookup_pix_format(pixfmt->pixelformat);
	}

	/* check if hw supports it */
	temp = 0;
	found = 0;
	while (ccdc_dev->hw_ops.enum_pix(&pix, temp) >= 0) {
		if (vpfe_pix_fmt->fmtdesc.pixelformat == pix) {
			found = 1;
			break;
		}
		temp++;
	}

	if (!found) {
		/* use current pixel format */
		pixfmt->pixelformat = vpfe_dev->fmt.fmt.pix.pixelformat;
		/*
		 * Since this is currently used in the vpfe device, we
		 * will find this pix format in the table
		 */
		vpfe_pix_fmt = vpfe_lookup_pix_format(pixfmt->pixelformat);
	}

	/* check what field format is supported */
	if (pixfmt->field == V4L2_FIELD_ANY) {
		/* if field is any, use current value as default */
		pixfmt->field = vpfe_dev->fmt.fmt.pix.field;
	}

	/*
	 * if field is not same as current field in the vpfe device
	 * try matching the field with the sub device field
	 */
	if (vpfe_dev->fmt.fmt.pix.field != pixfmt->field) {
		/*
		 * If field value is not in the supported fields, use current
		 * field used in the device as default
		 */
		switch (pixfmt->field) {
		case V4L2_FIELD_INTERLACED:
		case V4L2_FIELD_SEQ_TB:
			/* if sub device is supporting progressive, use that */
			if (!vpfe_dev->std_info.frame_format)
				pixfmt->field = V4L2_FIELD_NONE;
			break;
		case V4L2_FIELD_NONE:
			if (vpfe_dev->std_info.frame_format)
				pixfmt->field = V4L2_FIELD_INTERLACED;
			break;

		default:
			/* use current field as default */
			pixfmt->field = vpfe_dev->fmt.fmt.pix.field;
			break;
		}
	}

	/* Now adjust image resolutions supported */
	if (pixfmt->field == V4L2_FIELD_INTERLACED ||
	    pixfmt->field == V4L2_FIELD_SEQ_TB)
		min_height = 2;

	max_width = vpfe_dev->std_info.active_pixels;
	max_height = vpfe_dev->std_info.active_lines;
	min_width /= vpfe_pix_fmt->bpp;

	v4l2_info(&vpfe_dev->v4l2_dev, "width = %d, height = %d, bpp = %d\n",
		  pixfmt->width, pixfmt->height, vpfe_pix_fmt->bpp);

	pixfmt->width = clamp((pixfmt->width), min_width, max_width);
	pixfmt->height = clamp((pixfmt->height), min_height, max_height);

	/* If interlaced, adjust height to be a multiple of 2 */
	if (pixfmt->field == V4L2_FIELD_INTERLACED)
		pixfmt->height &= (~1);
	/*
	 * recalculate bytesperline and sizeimage since width
	 * and height might have changed
	 */
	pixfmt->bytesperline = (((pixfmt->width * vpfe_pix_fmt->bpp) + 31)
				& ~31);
	if (pixfmt->pixelformat == V4L2_PIX_FMT_NV12)
		pixfmt->sizeimage =
			pixfmt->bytesperline * pixfmt->height +
			((pixfmt->bytesperline * pixfmt->height) >> 1);
	else
		pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;

	v4l2_info(&vpfe_dev->v4l2_dev, "adjusted width = %d, height ="
		 " %d, bpp = %d, bytesperline = %d, sizeimage = %d\n",
		 pixfmt->width, pixfmt->height, vpfe_pix_fmt->bpp,
		 pixfmt->bytesperline, pixfmt->sizeimage);
	return vpfe_pix_fmt;
}

static int vpfe_querycap(struct file *file, void  *priv,
			       struct v4l2_capability *cap)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_querycap\n");

	cap->version = VPFE_CAPTURE_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "VPFE", sizeof(cap->bus_info));
	strlcpy(cap->card, vpfe_dev->cfg->card_name, sizeof(cap->card));
	return 0;
}

static int vpfe_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret = 0;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_g_fmt_vid_cap\n");
	/* Fill in the information about format */
	*fmt = vpfe_dev->fmt;
	return ret;
}

static int vpfe_enum_fmt_vid_cap(struct file *file, void  *priv,
				   struct v4l2_fmtdesc *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	const struct vpfe_pixel_format *pix_fmt;
	int temp_index;
	u32 pix;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_enum_fmt_vid_cap\n");

	if (ccdc_dev->hw_ops.enum_pix(&pix, fmt->index) < 0)
		return -EINVAL;

	/* Fill in the information about format */
	pix_fmt = vpfe_lookup_pix_format(pix);
	if (NULL != pix_fmt) {
		temp_index = fmt->index;
		*fmt = pix_fmt->fmtdesc;
		fmt->index = temp_index;
		return 0;
	}
	return -EINVAL;
}

static int vpfe_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	const struct vpfe_pixel_format *pix_fmts;
	int ret = 0;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_s_fmt_vid_cap\n");

	/* If streaming is started, return error */
	if (vpfe_dev->started) {
		v4l2_err(&vpfe_dev->v4l2_dev, "Streaming is started\n");
		return -EBUSY;
	}

	/* Check for valid frame format */
	pix_fmts = vpfe_check_format(vpfe_dev, &fmt->fmt.pix);

	if (NULL == pix_fmts)
		return -EINVAL;

	/* store the pixel format in the device  object */
	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	/* First detach any IRQ if currently attached */
	vpfe_detach_irq(vpfe_dev);
	vpfe_dev->fmt = *fmt;
	/* set image capture parameters in the ccdc */
	ret = vpfe_config_ccdc_image_format(vpfe_dev);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	const struct vpfe_pixel_format *pix_fmts;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_try_fmt_vid_cap\n");

	pix_fmts = vpfe_check_format(vpfe_dev, &f->fmt.pix);
	if (NULL == pix_fmts)
		return -EINVAL;
	return 0;
}

/*
 * vpfe_get_subdev_input_index - Get subdev index and subdev input index for a
 * given app input index
 */
static int vpfe_get_subdev_input_index(struct vpfe_device *vpfe_dev,
					int *subdev_index,
					int *subdev_input_index,
					int app_input_index)
{
	struct vpfe_config *cfg = vpfe_dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (app_input_index < (j + sdinfo->num_inputs)) {
			*subdev_index = i;
			*subdev_input_index = app_input_index - j;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

/*
 * vpfe_get_app_input - Get app input index for a given subdev input index
 * driver stores the input index of the current sub device and translate it
 * when application request the current input
 */
static int vpfe_get_app_input_index(struct vpfe_device *vpfe_dev,
				    int *app_input_index)
{
	struct vpfe_config *cfg = vpfe_dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (!strcmp(sdinfo->name, vpfe_dev->current_subdev->name)) {
			if (vpfe_dev->current_input >= sdinfo->num_inputs)
				return -1;
			*app_input_index = j + vpfe_dev->current_input;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

static int vpfe_enum_input(struct file *file, void *priv,
				 struct v4l2_input *inp)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int subdev, index ;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_enum_input\n");

	if (vpfe_get_subdev_input_index(vpfe_dev,
					&subdev,
					&index,
					inp->index) < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev, "input information not found"
			 " for the subdev\n");
		return -EINVAL;
	}
	sdinfo = &vpfe_dev->cfg->sub_devs[subdev];
	memcpy(inp, &sdinfo->inputs[index], sizeof(struct v4l2_input));
	return 0;
}

static int vpfe_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_g_input\n");

	return vpfe_get_app_input_index(vpfe_dev, index);
}


static int vpfe_s_input(struct file *file, void *priv, unsigned int index)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int subdev_index, inp_index;
	struct vpfe_route *route;
	u32 input = 0, output = 0;
	int ret = -EINVAL;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_s_input\n");

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	/*
	 * If streaming is started return device busy
	 * error
	 */
	if (vpfe_dev->started) {
		v4l2_err(&vpfe_dev->v4l2_dev, "Streaming is on\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	if (vpfe_get_subdev_input_index(vpfe_dev,
					&subdev_index,
					&inp_index,
					index) < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev, "invalid input index\n");
		goto unlock_out;
	}

	sdinfo = &vpfe_dev->cfg->sub_devs[subdev_index];
	route = &sdinfo->routes[inp_index];
	if (route && sdinfo->can_route) {
		input = route->input;
		output = route->output;
	}

	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					 video, s_routing, input, output, 0);

	if (ret) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"vpfe_doioctl:error in setting input in decoder\n");
		ret = -EINVAL;
		goto unlock_out;
	}
	vpfe_dev->current_subdev = sdinfo;
	vpfe_dev->current_input = index;
	vpfe_dev->std_index = 0;

	/* set the bus/interface parameter for the sub device in ccdc */
	ret = ccdc_dev->hw_ops.set_hw_if_params(&sdinfo->ccdc_if_params);
	if (ret)
		goto unlock_out;

	/* set the default image parameters in the device */
	ret = vpfe_config_image_format(vpfe_dev,
				&vpfe_standards[vpfe_dev->std_index].std_id);
unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_querystd(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_querystd\n");

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	sdinfo = vpfe_dev->current_subdev;
	if (ret)
		return ret;
	/* Call querystd function of decoder device */
	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					 video, querystd, std_id);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_s_std(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_s_std\n");

	/* Call decoder driver function to set the standard */
	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	sdinfo = vpfe_dev->current_subdev;
	/* If streaming is started, return device busy error */
	if (vpfe_dev->started) {
		v4l2_err(&vpfe_dev->v4l2_dev, "streaming is started\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					 core, s_std, *std_id);
	if (ret < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev, "Failed to set standard\n");
		goto unlock_out;
	}
	ret = vpfe_config_image_format(vpfe_dev, std_id);

unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_g_std(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_g_std\n");

	*std_id = vpfe_standards[vpfe_dev->std_index].std_id;
	return 0;
}
/*
 *  Videobuf operations
 */
static int vpfe_videobuf_setup(struct videobuf_queue *vq,
				unsigned int *count,
				unsigned int *size)
{
	struct vpfe_fh *fh = vq->priv_data;
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_buffer_setup\n");
	*size = vpfe_dev->fmt.fmt.pix.sizeimage;
	if (vpfe_dev->memory == V4L2_MEMORY_MMAP &&
		vpfe_dev->fmt.fmt.pix.sizeimage > config_params.device_bufsize)
		*size = config_params.device_bufsize;

	if (*count < config_params.min_numbuffers)
		*count = config_params.min_numbuffers;
	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev,
		"count=%d, size=%d\n", *count, *size);
	return 0;
}

static int vpfe_videobuf_prepare(struct videobuf_queue *vq,
				struct videobuf_buffer *vb,
				enum v4l2_field field)
{
	struct vpfe_fh *fh = vq->priv_data;
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	unsigned long addr;
	int ret;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_buffer_prepare\n");

	/* If buffer is not initialized, initialize it */
	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		vb->width = vpfe_dev->fmt.fmt.pix.width;
		vb->height = vpfe_dev->fmt.fmt.pix.height;
		vb->size = vpfe_dev->fmt.fmt.pix.sizeimage;
		vb->field = field;

		ret = videobuf_iolock(vq, vb, NULL);
		if (ret < 0)
			return ret;

	/* pitch should be 32 bytes aligned */
	pix->bytesperline = ALIGN(pix->bytesperline, 32);

	pix->sizeimage = pix->bytesperline * pix->height;
}
EXPORT_SYMBOL(mbus_to_pix);

/* ISR for VINT0*/
irqreturn_t vpfe_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_isr\n");

	ccdc_buffer_isr(&vpfe_dev->vpfe_ccdc);

	prv_buffer_isr(&vpfe_dev->vpfe_previewer);

	rsz_buffer_isr(&vpfe_dev->vpfe_resizer);

	return IRQ_HANDLED;
}

/* vpfe_vdint1_isr - isr handler for VINT1 interrupt */
irqreturn_t vpfe_vdint1_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_vdint1_isr\n");

	ccdc_vidint1_isr(&vpfe_dev->vpfe_ccdc);

	return IRQ_HANDLED;
}

irqreturn_t vpfe_imp_dma_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;

	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev, "vpfe_imp_dma_isr\n");

	prv_dma_isr(&vpfe_dev->vpfe_previewer);

	rsz_dma_isr(&vpfe_dev->vpfe_resizer);

	return IRQ_HANDLED;
}

static struct vpfe_device *vpfe_initialize(void)
{
	struct vpfe_device *vpfe_dev;

	/* Allocate memory for device objects */
	vpfe_dev = kzalloc(sizeof(*vpfe_dev), GFP_KERNEL);

	/* initialize config settings */
	vpfe_dev->config_params.min_numbuffers = 3;
	vpfe_dev->config_params.numbuffers = 3;
	vpfe_dev->config_params.min_bufsize = 1280 * 720 * 2;
	vpfe_dev->config_params.device_bufsize = 1920 * 1080 * 2;

	/* Default number of buffers should be 3 */
	if ((numbuffers > 0) &&
	    (numbuffers < vpfe_dev->config_params.min_numbuffers))
		numbuffers = vpfe_dev->config_params.min_numbuffers;

	/*
	 * Set buffer size to min buffers size if invalid buffer size is
	 * given
	 */
	if (bufsize < vpfe_dev->config_params.min_bufsize)
		bufsize = vpfe_dev->config_params.min_bufsize;

	vpfe_dev->config_params.numbuffers = numbuffers;

	if (numbuffers)
		vpfe_dev->config_params.device_bufsize = ALIGN(bufsize, 4096);

	return vpfe_dev;
}

static void vpfe_disable_clock(struct vpfe_device *vpfe_dev)
{
	struct vpfe_config *vpfe_cfg = vpfe_dev->cfg;
	int i;

	for (i = 0; i < vpfe_cfg->num_clocks; i++) {
		clk_disable(vpfe_dev->clks[i]);
		clk_put(vpfe_dev->clks[i]);
	}

	kzfree(vpfe_dev->clks);
	v4l2_info(vpfe_dev->pdev->driver, "vpfe capture clocks disabled\n");
}

/**
 * vpfe_enable_clock() - Enable clocks for vpfe capture driver
 * @vpfe_dev - ptr to vpfe capture device
 *
 * Enables clocks defined in vpfe configuration. The function
 * assumes that at least one clock is to be defined which is
 * true as of now. re-visit this if this assumption is not true
 */
static int vpfe_enable_clock(struct vpfe_device *vpfe_dev)
{
	struct vpfe_config *vpfe_cfg = vpfe_dev->cfg;
	int ret = -EFAULT, i;

	if (!vpfe_cfg->num_clocks)
		return 0;

	vpfe_dev->clks = kzalloc(vpfe_cfg->num_clocks *
				   sizeof(struct clock *), GFP_KERNEL);

	if (NULL == vpfe_dev->clks) {
		v4l2_err(vpfe_dev->pdev->driver, "Memory allocation failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < vpfe_cfg->num_clocks; i++) {
		if (NULL == vpfe_cfg->clocks[i]) {
			v4l2_err(vpfe_dev->pdev->driver,
				"clock %s is not defined in vpfe config\n",
				vpfe_cfg->clocks[i]);
			goto out;
		}

		vpfe_dev->clks[i] = clk_get(vpfe_dev->pdev,
					      vpfe_cfg->clocks[i]);
		if (NULL == vpfe_dev->clks[i]) {
			v4l2_err(vpfe_dev->pdev->driver,
				"Failed to get clock %s\n",
				vpfe_cfg->clocks[i]);
			goto out;
		}

		if (clk_enable(vpfe_dev->clks[i])) {
			v4l2_err(vpfe_dev->pdev->driver,
				"vpfe clock %s not enabled\n",
				vpfe_cfg->clocks[i]);
			goto out;
		}

		v4l2_info(vpfe_dev->pdev->driver, "vpss clock %s enabled",
			  vpfe_cfg->clocks[i]);
	}
	return 0;
out:
	for (i = 0; i < vpfe_cfg->num_clocks; i++) {
		if (vpfe_dev->clks[i])
			clk_put(vpfe_dev->clks[i]);
	}

	v4l2_err(vpfe_dev->pdev->driver,
				"failed to enable clocks\n");

	kzfree(vpfe_dev->clks);
	return ret;
}

static void vpfe_detach_irq(struct vpfe_device *vpfe_dev)
{
	free_irq(vpfe_dev->ccdc_irq0, vpfe_dev);
	free_irq(vpfe_dev->ccdc_irq1, vpfe_dev);
	free_irq(vpfe_dev->imp_dma_irq, vpfe_dev);
}

static int vpfe_attach_irq(struct vpfe_device *vpfe_dev)
{
	int ret = 0;

	ret = request_irq(vpfe_dev->ccdc_irq0, vpfe_isr, IRQF_DISABLED,
			"vpfe_capture0", vpfe_dev);
	if (ret < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"Error: requesting VINT0 interrupt\n");
		return ret;
	}

	ret = request_irq(vpfe_dev->ccdc_irq1,
					vpfe_vdint1_isr,
					IRQF_DISABLED,
					"vpfe_capture1", vpfe_dev);
	if (ret < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"Error: requesting VINT1 interrupt\n");
		free_irq(vpfe_dev->ccdc_irq0, vpfe_dev);
		return ret;
	}

	ret = request_irq(vpfe_dev->imp_dma_irq,
				vpfe_imp_dma_isr,
				IRQF_DISABLED,
				"Imp_Sdram_Irq",
				vpfe_dev);
	if (ret < 0) {
		v4l2_err(&vpfe_dev->v4l2_dev,
				"Error: requesting IMP"
				" IRQ interrupt\n");
		free_irq(vpfe_dev->ccdc_irq1, vpfe_dev);
		free_irq(vpfe_dev->ccdc_irq0, vpfe_dev);
		return ret;
	}

	return 0;
}

static int register_i2c_devices(struct vpfe_device *vpfe_dev)
{
	struct vpfe_config *vpfe_cfg;
	struct i2c_adapter *i2c_adap;
	int i, k, ret;
	unsigned int num_subdevs;
	struct vpfe_subdev_info *sdinfo;

	vpfe_cfg = vpfe_dev->cfg;

	i2c_adap = i2c_get_adapter(1);
	num_subdevs = vpfe_cfg->num_subdevs;

	vpfe_dev->sd = kzalloc(sizeof(struct v4l2_subdev *) *num_subdevs,
			       GFP_KERNEL);

	if (NULL == vpfe_dev->sd) {
		v4l2_err(&vpfe_dev->v4l2_dev,
			"unable to allocate memory for subdevice pointers\n");
		return -ENOMEM;
	}

	for (i = 0, k = 0; i < num_subdevs; i++) {
		sdinfo = &vpfe_cfg->sub_devs[i];
		/**
		 * register subdevices based on interface setting. Currently
		 * tvp5146 and mt9p031 cannot co-exists due to i2c address
		 * conflicts. So only one of them is registered. Re-visit this
		 * once we have support for i2c switch handling in i2c driver
		 * framework
		 */

		if (interface == sdinfo->is_camera) {
			/* setup input path */
			if (vpfe_cfg->setup_input) {
				if (vpfe_cfg->setup_input(sdinfo->grp_id) < 0) {
					ret = -EFAULT;
					v4l2_info(&vpfe_dev->v4l2_dev, "could"
							" not setup input for %s\n",
							sdinfo->module_name);
					goto probe_sd_out;
				}
			}
			/* Load up the subdevice */
			vpfe_dev->sd[k] =
				v4l2_i2c_new_subdev_board(
						  &vpfe_dev->v4l2_dev,
						  i2c_adap,
						  &sdinfo->board_info,
						  NULL,
						1);
			if (vpfe_dev->sd[k]) {
				v4l2_info(&vpfe_dev->v4l2_dev,
						"v4l2 sub device %s registered\n",
						sdinfo->module_name);

				vpfe_dev->sd[k]->grp_id = sdinfo->grp_id;
				k++;

				sdinfo->registered = 1;
			}
			} else {
				v4l2_info(&vpfe_dev->v4l2_dev,
						"v4l2 sub device %s register fails\n",
						sdinfo->module_name);
			}
	}

	vpfe_dev->num_subdevs = k;
	return 0;

probe_sd_out:
	kzfree(vpfe_dev->sd);
	return ret;
}

static int vpfe_register_entities(struct vpfe_device *vpfe_dev)
{
	int ret, i;
	unsigned int flags = 0;

	/* register i2c devices first */
	ret = register_i2c_devices(vpfe_dev);
	if (ret)
		return ret;

	/* register rest of the sub-devs */
	ret = vpfe_ccdc_register_entities(&vpfe_dev->vpfe_ccdc,
					  &vpfe_dev->v4l2_dev);
	if (ret)
		return ret;

	ret = vpfe_previewer_register_entities(&vpfe_dev->vpfe_previewer,
					       &vpfe_dev->v4l2_dev);
	if (ret)
		goto out_ccdc_register;

	ret = vpfe_resizer_register_entities(&vpfe_dev->vpfe_resizer,
					     &vpfe_dev->v4l2_dev);
	if (ret)
		goto out_previewer_register;

	ret = vpfe_aew_register_entities(&vpfe_dev->vpfe_aew,
					 &vpfe_dev->v4l2_dev);
	if (ret)
		goto out_resizer_register;

	/* adjust the width to 16 pixel boundary */
	crop->c.width = ((crop->c.width + 15) & ~0xf);

	/* make sure parameters are valid */
	if ((crop->c.left + crop->c.width >
		vpfe_dev->std_info.active_pixels) ||
	    (crop->c.top + crop->c.height >
		vpfe_dev->std_info.active_lines)) {
		v4l2_err(&vpfe_dev->v4l2_dev, "Error in S_CROP params\n");
		ret = -EINVAL;
		goto unlock_out;
	}

	ret = media_entity_create_link(&vpfe_dev->vpfe_ccdc.subdev.entity,
					1, &vpfe_dev->vpfe_aew.subdev.entity,
					0, flags);
	if (ret < 0)
		goto out_resizer_register;

static long vpfe_param_handler(struct file *file, void *priv,
		bool valid_prio, int cmd, void *param)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret = 0;

	ret = media_entity_create_link(&vpfe_dev->vpfe_ccdc.subdev.entity,
				       1,
				       &vpfe_dev->vpfe_previewer.subdev.entity,
				       0, flags);
	if (ret < 0)
		goto out_resizer_register;

	ret = media_entity_create_link(&vpfe_dev->vpfe_previewer.subdev.entity,
				       1, &vpfe_dev->vpfe_resizer.subdev.entity,
				       0, flags);
	if (ret < 0)
		goto out_resizer_register;

	return 0;

out_aew_register:
	vpfe_aew_unregister_entities(&vpfe_dev->vpfe_aew);
out_resizer_register:
	vpfe_resizer_unregister_entities(&vpfe_dev->vpfe_resizer);
out_previewer_register:
	vpfe_previewer_unregister_entities(&vpfe_dev->vpfe_previewer);
out_ccdc_register:
	vpfe_ccdc_unregister_entities(&vpfe_dev->vpfe_ccdc);
	return ret;
}

static void vpfe_unregister_entities(struct vpfe_device *vpfe_dev)
{
	vpfe_ccdc_unregister_entities(&vpfe_dev->vpfe_ccdc);
	vpfe_previewer_unregister_entities(&vpfe_dev->vpfe_previewer);
	vpfe_resizer_unregister_entities(&vpfe_dev->vpfe_resizer);
	vpfe_aew_unregister_entities(&vpfe_dev->vpfe_aew);
	vpfe_af_unregister_entities(&vpfe_dev->vpfe_af);
}

static void vpfe_cleanup_modules(struct vpfe_device *vpfe_dev,
				 struct platform_device *pdev)
{
	vpfe_ccdc_cleanup(pdev);
	vpfe_previewer_cleanup(pdev);
	vpfe_resizer_cleanup(pdev);
	vpfe_aew_cleanup(pdev);
	vpfe_af_cleanup(pdev);
}

static int vpfe_initialize_modules(struct vpfe_device *vpfe_dev,
				   struct platform_device *pdev)
{
	int ret;

	ret = vpfe_ccdc_init(&vpfe_dev->vpfe_ccdc, pdev);
	if (ret)
		return ret;

	ret = vpfe_previewer_init(&vpfe_dev->vpfe_previewer, pdev);
	if (ret)
		goto out_ccdc_init;

	ret = vpfe_resizer_init(&vpfe_dev->vpfe_resizer, pdev);
	if (ret)
		goto out_previewer_init;

	ret = vpfe_aew_init(&vpfe_dev->vpfe_aew, pdev);
	if (ret)
		goto out_resizer_init;

	ret = vpfe_af_init(&vpfe_dev->vpfe_af, pdev);
	if (ret)
		goto out_resizer_init;

	return 0;

out_resizer_init:
	vpfe_resizer_cleanup(pdev);
out_previewer_init:
	vpfe_previewer_cleanup(pdev);
out_ccdc_init:
	vpfe_ccdc_cleanup(pdev);
	return ret;
}

/**
 * vpfe_probe : vpfe probe function
 * @pdev: platform device pointer
 *
 * This function creates device entries by register itself to the V4L2 driver
 * and initializes fields of each device objects
 */
static __devinit int vpfe_probe(struct platform_device *pdev)
{
	struct vpfe_config *vpfe_cfg;
	struct resource *res1;
	struct vpfe_device *vpfe_dev;
	int ret = -ENOMEM, err;
	unsigned long phys_end_kernel;
	size_t size;

	/* Get the pointer to the device object */
	vpfe_dev = vpfe_initialize();

	if (!vpfe_dev) {
		v4l2_err(pdev->dev.driver,
			"Failed to allocate memory for vpfe_dev\n");
		return ret;
	}

	vpfe_dev->pdev = &pdev->dev;

	if (cont_bufsize) {
		/* attempt to determine the end of Linux kernel memory */
		phys_end_kernel = virt_to_phys((void *)PAGE_OFFSET) +
			(num_physpages << PAGE_SHIFT);
		size = cont_bufsize;
		phys_end_kernel += cont_bufoffset;
		err = dma_declare_coherent_memory(&pdev->dev, phys_end_kernel,
				phys_end_kernel, size,
				DMA_MEMORY_MAP | DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Unable to declare MMAP memory.\n");
			ret = -ENOENT;
			goto probe_free_dev_mem;
		}
		vpfe_dev->config_params.video_limit = size;
	}

	if (NULL == pdev->dev.platform_data) {
		v4l2_err(pdev->dev.driver, "Unable to get vpfe config\n");
		ret = -ENOENT;
		goto probe_free_dev_mem;
	}

	vpfe_cfg = pdev->dev.platform_data;
	vpfe_dev->cfg = vpfe_cfg;
	if (NULL == vpfe_cfg->card_name ||
	    NULL == vpfe_cfg->sub_devs) {
		v4l2_err(pdev->dev.driver, "null ptr in vpfe_cfg\n");
		ret = -ENOENT;
		goto probe_free_dev_mem;
	}

	/* enable vpss clocks */
	ret = vpfe_enable_clock(vpfe_dev);
	if (ret)
		goto probe_free_dev_mem;

	mutex_lock(&ccdc_lock);

	if (vpfe_initialize_modules(vpfe_dev, pdev))
		goto probe_disable_clock;

	/* Get VINT0 irq resource */
	res1 = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res1) {
		v4l2_err(pdev->dev.driver,
			 "Unable to get interrupt for VINT0\n");
		ret = -ENOENT;
		goto probe_out_ccdc_cleanup;
	}
	vpfe_dev->ccdc_irq0 = res1->start;

	/* Get VINT1 irq resource */
	res1 = platform_get_resource(pdev,
				IORESOURCE_IRQ, 1);
	if (!res1) {
		v4l2_err(pdev->dev.driver,
			 "Unable to get interrupt for VINT1\n");
		ret = -ENOENT;
		goto probe_out_ccdc_cleanup;
	}
	vpfe_dev->ccdc_irq1 = res1->start;

	/* Get PRVUINT irq resource */
	res1 = platform_get_resource(pdev,
				IORESOURCE_IRQ, 2);
	if (!res1) {
		v4l2_err(pdev->dev.driver,
			 "Unable to get interrupt for PRVUINT\n");
		ret = -ENOENT;
		goto probe_out_ccdc_cleanup;
	}
	vpfe_dev->imp_dma_irq = res1->start;

	vpfe_dev->media_dev.dev = vpfe_dev->pdev;
	strcpy((char *)&vpfe_dev->media_dev.model, "davinci-media");
	ret = media_device_register(&vpfe_dev->media_dev);
	if (ret)
		goto probe_out_ccdc_cleanup;

	vpfe_dev->v4l2_dev.mdev = &vpfe_dev->media_dev;

	ret = v4l2_device_register(&pdev->dev, &vpfe_dev->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
			"Unable to register v4l2 device.\n");
		goto probe_out_video_release;
	}
	v4l2_info(&vpfe_dev->v4l2_dev, "v4l2 device registered\n");

	/* register video device */
	v4l2_dbg(1, debug, &vpfe_dev->v4l2_dev,
		"trying to register vpfe device.\n");

	/* set the driver data in platform device */
	platform_set_drvdata(pdev, vpfe_dev);

	/* register subdevs/entities */
	if (vpfe_register_entities(vpfe_dev))
		goto probe_out_video_unregister;

	ret = vpfe_attach_irq(vpfe_dev);
	if (ret)
		goto probe_out_register_entities;

	mutex_unlock(&ccdc_lock);
	return 0;

probe_out_register_entities:
	vpfe_register_entities(vpfe_dev);
probe_out_video_unregister:
	/*TODO we need this?*/
probe_out_video_release:
	/*TODO we need this?*/
probe_out_ccdc_cleanup:
	vpfe_cleanup_modules(vpfe_dev, pdev);
probe_disable_clock:
	vpfe_disable_clock(vpfe_dev);
	mutex_unlock(&ccdc_lock);
probe_free_dev_mem:
	kzfree(vpfe_dev);
	return ret;
}

/*
 * vpfe_remove : It un-registers device from V4L2 driver
 */
static int vpfe_remove(struct platform_device *pdev)
{
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);

	v4l2_info(pdev->dev.driver, "vpfe_remove\n");

	kzfree(vpfe_dev->sd);
	vpfe_detach_irq(vpfe_dev);
	vpfe_unregister_entities(vpfe_dev);
	vpfe_cleanup_modules(vpfe_dev, pdev);
	v4l2_device_unregister(&vpfe_dev->v4l2_dev);
	vpfe_disable_clock(vpfe_dev);
	kzfree(vpfe_dev);
	return 0;
}

static struct platform_driver vpfe_driver = {
	.driver = {
		.name = CAPTURE_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = vpfe_probe,
	.remove = __devexit_p(vpfe_remove),
};

module_platform_driver(vpfe_driver);
