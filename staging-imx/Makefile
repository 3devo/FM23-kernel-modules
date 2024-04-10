# SPDX-License-Identifier: GPL-2.0

imx8-capture-modified-objs := imx8-isi-core.o
imx8-isi-capture-modified-objs := imx8-isi-fmt.o imx8-isi-cap.o
imx8-isi-mem2mem-modified-objs := imx8-isi-fmt.o imx8-isi-m2m.o
imx8-isi-hw-modified-objs := imx8-isi-hw.o
imx8-media-dev-modified-objs := imx8-media-dev.o

# This adds -modified to all the module files to make it very clear that
# the modified kernel modules are correctly loaded
obj-m   += imx8-capture-modified.o
obj-m   += imx8-isi-capture-modified.o
obj-m   += imx8-isi-mem2mem-modified.o
obj-m   += imx8-isi-hw-modified.o
obj-m   += imx8-media-dev-modified.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean

# Rename all exported symbols, to allow loading these modules alongside
# the same code that is already built into the kernel.
# grep -r EXPORT_SYMBOL imx8-*.c | sed 's/.*(\(.*\));/EXTRA_CFLAGS+=-D\1=\1_modified/'
EXTRA_CFLAGS+=-Dmxc_isi_cap_frame_write_done=mxc_isi_cap_frame_write_done_modified
EXTRA_CFLAGS+=-Dis_buf_active=is_buf_active_modified
EXTRA_CFLAGS+=-Dmxc_isi_dev_get_parent=mxc_isi_dev_get_parent_modified
EXTRA_CFLAGS+=-Dmxc_isi_get_hostdata=mxc_isi_get_hostdata_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_set_outbuf=mxc_isi_channel_set_outbuf_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_set_m2m_src_addr=mxc_isi_channel_set_m2m_src_addr_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_sw_reset=mxc_isi_channel_sw_reset_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_source_config=mxc_isi_channel_source_config_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_set_chain_buf=mxc_isi_channel_set_chain_buf_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_init=mxc_isi_channel_init_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_deinit=mxc_isi_channel_deinit_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_config=mxc_isi_channel_config_modified
EXTRA_CFLAGS+=-Dmxc_isi_clean_registers=mxc_isi_clean_registers_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_enable=mxc_isi_channel_enable_modified
EXTRA_CFLAGS+=-Dmxc_isi_channel_disable=mxc_isi_channel_disable_modified
EXTRA_CFLAGS+=-Dmxc_isi_enable_irq=mxc_isi_enable_irq_modified
EXTRA_CFLAGS+=-Dmxc_isi_disable_irq=mxc_isi_disable_irq_modified
EXTRA_CFLAGS+=-Dmxc_isi_get_irq_status=mxc_isi_get_irq_status_modified
EXTRA_CFLAGS+=-Dmxc_isi_clean_irq_status=mxc_isi_clean_irq_status_modified
EXTRA_CFLAGS+=-Dmxc_isi_m2m_config_src=mxc_isi_m2m_config_src_modified
EXTRA_CFLAGS+=-Dmxc_isi_m2m_config_dst=mxc_isi_m2m_config_dst_modified
EXTRA_CFLAGS+=-Dmxc_isi_m2m_start_read=mxc_isi_m2m_start_read_modified
EXTRA_CFLAGS+=-Dmxc_isi_m2m_frame_write_done=mxc_isi_m2m_frame_write_done_modified
