/*
 * Copyright 2004-2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 by Sascha Hauer <kernel@pengutronix.de>
 * Copyright (C) 2009 by Jan Weitzel Phytec Messtechnik GmbH,
 *                       <armlinux@phytec.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux-v3.h>

static void __iomem *base;

#ifdef CONFIG_DEBUG_FS
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
static struct dentry *debugfs_iomux;
static int iomux_count;
#endif

/*
 * configures a single pad in the iomuxer
 */
int mxc_iomux_v3_setup_pad(iomux_v3_cfg_t pad)
{
	u32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 mux_mode = (pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT;
	u32 sel_input_ofs = (pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
	u32 sel_input = (pad & MUX_SEL_INPUT_MASK) >> MUX_SEL_INPUT_SHIFT;
	u32 pad_ctrl_ofs = (pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
	u32 pad_ctrl = (pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;

	if (mux_ctrl_ofs)
		__raw_writel(mux_mode, base + mux_ctrl_ofs);

	if (sel_input_ofs)
		__raw_writel(sel_input, base + sel_input_ofs);

	if (!(pad_ctrl & NO_PAD_CTRL) && pad_ctrl_ofs)
		__raw_writel(pad_ctrl, base + pad_ctrl_ofs);

	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_pad);

/*
 * Read a single pad in the iomuxer
 */
int mxc_iomux_v3_get_pad(iomux_v3_cfg_t *pad)
{
	u32 mux_ctrl_ofs = (*pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 pad_ctrl_ofs = (*pad & MUX_PAD_CTRL_OFS_MASK)
						>> MUX_PAD_CTRL_OFS_SHIFT;
	u32 sel_input_ofs = (*pad & MUX_SEL_INPUT_OFS_MASK)
						>> MUX_SEL_INPUT_OFS_SHIFT;
	u32 mux_mode = 0;
	u32 sel_input = 0;
	u32 pad_ctrl = 0;
	iomux_v3_cfg_t pad_info = 0;

	mux_mode = __raw_readl(base + mux_ctrl_ofs) & 0xFF;
	pad_ctrl = __raw_readl(base + pad_ctrl_ofs) & 0x1FFFF;
	sel_input = __raw_readl(base + sel_input_ofs) & 0x7;

	pad_info = (((iomux_v3_cfg_t)mux_mode << MUX_MODE_SHIFT) | \
		((iomux_v3_cfg_t)pad_ctrl << MUX_PAD_CTRL_SHIFT) | \
		((iomux_v3_cfg_t)sel_input << MUX_SEL_INPUT_SHIFT));

	*pad &= ~(MUX_MODE_MASK | MUX_PAD_CTRL_MASK | MUX_SEL_INPUT_MASK);
	*pad |= pad_info;

	return 0;
}


int mxc_iomux_v3_setup_multiple_pads(iomux_v3_cfg_t *pad_list, unsigned count)
{
	iomux_v3_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = mxc_iomux_v3_setup_pad(*p);
		if (ret)
			return ret;
		p++;
	}
	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_setup_multiple_pads);

/*
 * Read multiple pads in the iomuxer
 */
int mxc_iomux_v3_get_multiple_pads(iomux_v3_cfg_t *pad_list, unsigned count)
{
	iomux_v3_cfg_t *p = pad_list;
	int i;

	for (i = 0; i < count; i++) {
		mxc_iomux_v3_get_pad(p);
		p++;
	}
	return 0;
}
EXPORT_SYMBOL(mxc_iomux_v3_get_multiple_pads);

void mxc_iomux_set_gpr_register(int group, int start_bit, int num_bits, int value)
{
	int i = 0;
	u32 reg;
	reg = __raw_readl(base + group * 4);
	while (num_bits) {
		reg &= ~(1<<(start_bit + i));
		i++;
		num_bits--;
	}
	reg |= (value << start_bit);
	__raw_writel(reg, base + group * 4);
}
EXPORT_SYMBOL(mxc_iomux_set_gpr_register);
void mxc_iomux_set_specialbits_register(u32 pad_addr, u32 value, u32 mask)
{
	u32 reg;
	reg = __raw_readl(base + pad_addr);
	reg &= ~mask;
	reg |= value;
	__raw_writel(reg, base + pad_addr);
}
EXPORT_SYMBOL(mxc_iomux_set_specialbits_register);

#ifdef CONFIG_DEBUG_FS

static int iomux_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t  iomux_read_file(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	void __iomem *iomux_base = file->private_data;
	ssize_t ret=0;
	int i,step=4;
	char *values = kmalloc(((iomux_count+PAGE_SIZE-1) & (~ (PAGE_SIZE-1))), GFP_KERNEL);
	char *buf = kmalloc(10*PAGE_SIZE, GFP_KERNEL);	
	for(i=0;i<iomux_count;i++){
		values[i] = __raw_readl(iomux_base+i*step);
	}
	//now print to page buffer	
	ret += sprintf(buf+ret,"iomux register dumper\n");			
	for(i=0;i<iomux_count;i++){
		if(0==i%4)
			ret += sprintf(buf+ret,"\n0x%04x: ",i*step);			
		ret += sprintf(buf+ret,"%04x ",values[i]);
	}
	
	ret += sprintf(buf+ret,"\n");
	if (ret >= 0)
		ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);
	kfree(buf);
	kfree(values);
	return ret;
}

static ssize_t  iomux_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	void __iomem *iomux_base = file->private_data;
	char buf[32];
	int buf_size;	
	int step=4;
	char *start = buf;
	unsigned long reg, value;
	

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	while (*start == ' ')
		start++;
	reg = simple_strtoul(start, &start, 16);
	if (reg >= (iomux_count*step) )
	{
		printk("address out of dev io space\n");
		return -EINVAL;
	}
	while (*start == ' ')
		start++;
	if (strict_strtoul(start, 16, &value))
		return -EINVAL;
	//permission granted?
	printk("write %lx to %lx\n",value,reg);
	__raw_writel(value,(void __iomem *)(iomux_base+reg));

	return buf_size;
}


static const struct file_operations iomux_fops = {
	.open =  iomux_open_file,
	.read =  iomux_read_file,
	.write = iomux_write_file,
};

#endif
void mxc_iomux_v3_init(void __iomem *iomux_v3_base)
{
	base = iomux_v3_base;
}

static int __init iomux_late_init(void){
#ifdef CONFIG_DEBUG_FS
#ifdef CONFIG_SOC_IMX6Q
	if(cpu_is_mx6q()){
		iomux_count = 0x094C/4+1;//from RM
		printk("iomux reg count = %d\n",iomux_count);
	}
#endif
	debugfs_iomux = debugfs_create_file("iomux", 0644,
					 NULL, base,
					 &iomux_fops);	

#endif
	return 0;
}
late_initcall(iomux_late_init);

