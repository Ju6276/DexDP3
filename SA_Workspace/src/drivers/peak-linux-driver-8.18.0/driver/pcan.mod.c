#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xc1514a3b, "free_irq" },
	{ 0xea6a9460, "usb_alloc_urb" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xa9669bda, "usb_free_urb" },
	{ 0xc7e3f354, "param_ops_uint" },
	{ 0x5b78ae2a, "param_ops_ulong" },
	{ 0x13c49cc2, "_copy_from_user" },
	{ 0x7210d84f, "pci_enable_device" },
	{ 0xafd744c6, "__x86_indirect_thunk_rbp" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0x5e515be6, "ktime_get_ts64" },
	{ 0xa3ae1169, "usb_get_current_frame_number" },
	{ 0x4dd92e2a, "proc_create" },
	{ 0xeaa8ccd, "param_ops_ushort" },
	{ 0x44cf5f9d, "pci_iomap" },
	{ 0xb0e602eb, "memmove" },
	{ 0x656e4a6e, "snprintf" },
	{ 0xefae6aa8, "i2c_bit_add_bus" },
	{ 0x61298e51, "sysfs_add_file_to_group" },
	{ 0x8e602eaf, "rt_mutex_base_init" },
	{ 0x9c1745cd, "finish_wait" },
	{ 0x39806eab, "usb_register_driver" },
	{ 0x4c236f6f, "__x86_indirect_thunk_r15" },
	{ 0xc20d48a1, "__pci_register_driver" },
	{ 0xedc03953, "iounmap" },
	{ 0xf6e4338c, "param_array_ops" },
	{ 0x92d4ed68, "pci_request_regions" },
	{ 0x69acdf38, "memcpy" },
	{ 0x37a0cba, "kfree" },
	{ 0x9a090f3f, "pcpu_hot" },
	{ 0xdf6af08e, "seq_lseek" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0xda85f4a0, "prepare_to_wait_event" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x59aeb6f, "__wake_up" },
	{ 0x3609acb0, "pci_irq_vector" },
	{ 0x752eaef, "param_ops_byte" },
	{ 0xcc5005fe, "msleep_interruptible" },
	{ 0x269d2b80, "pci_unregister_driver" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x1035c7c2, "__release_region" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x122c3a7e, "_printk" },
	{ 0x7070425f, "usb_clear_halt" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x1000e51, "schedule" },
	{ 0xd79ea67c, "usb_bulk_msg" },
	{ 0x4a3aaa1e, "rt_spin_trylock" },
	{ 0x4c191b82, "usb_reset_device" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xb2fcb56d, "queue_delayed_work_on" },
	{ 0x6383b27c, "__x86_indirect_thunk_rdx" },
	{ 0x38f8e35, "usb_submit_urb" },
	{ 0x748c6de2, "_dev_info" },
	{ 0xf9b6b9b3, "i2c_del_adapter" },
	{ 0x167c5967, "print_hex_dump" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xbcba933e, "_dev_err" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0x4c03a563, "random_kmalloc_seed" },
	{ 0x7c029ad7, "mutex_lock" },
	{ 0x858d85e0, "dma_alloc_attrs" },
	{ 0xc987d4b6, "pci_read_config_word" },
	{ 0xf2764a7d, "usb_control_msg" },
	{ 0x9166fada, "strncpy" },
	{ 0x88f44c14, "usb_set_interface" },
	{ 0x1edb69d6, "ktime_get_raw_ts64" },
	{ 0xde80cd09, "ioremap" },
	{ 0x391f429a, "class_unregister" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x63b9727f, "sysfs_remove_file_from_group" },
	{ 0x9ae776af, "usb_deregister" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0xda6d0fbb, "pci_iounmap" },
	{ 0x65929cae, "ns_to_timespec64" },
	{ 0xdbdf6c92, "ioport_resource" },
	{ 0xfb578fc5, "memset" },
	{ 0x6518059e, "_dev_warn" },
	{ 0x715e1bb1, "pci_alloc_irq_vectors_affinity" },
	{ 0x3a64010e, "pci_set_master" },
	{ 0x8b1740dd, "param_ops_charp" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x6b10bee1, "_copy_to_user" },
	{ 0x68d94ef, "dma_get_required_mask" },
	{ 0xa09b2bf, "__init_waitqueue_head" },
	{ 0xb25390c4, "rt_spin_unlock" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x88368e04, "rt_spin_lock" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x31db98fb, "dma_set_coherent_mask" },
	{ 0x319394b8, "seq_read" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x8e9f8e60, "device_create_with_groups" },
	{ 0xfa1f6f66, "dma_free_attrs" },
	{ 0x24078ee, "mutex_unlock" },
	{ 0x9fa7184a, "cancel_delayed_work_sync" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x7b8b8fd9, "pci_release_regions" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x825533a8, "driver_for_each_device" },
	{ 0x66cca4f9, "__x86_indirect_thunk_rcx" },
	{ 0x859bb66f, "__register_chrdev" },
	{ 0xa8cfc23b, "device_destroy" },
	{ 0x7f47e101, "remove_proc_entry" },
	{ 0x4bb5813b, "usb_kill_urb" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x56470118, "__warn_printk" },
	{ 0xc040e101, "seq_printf" },
	{ 0xffeedf6a, "delayed_work_timer_fn" },
	{ 0x20000329, "simple_strtoul" },
	{ 0x62170c71, "i2c_transfer" },
	{ 0x31956ca7, "pci_disable_device" },
	{ 0xc88b639e, "usb_reset_endpoint" },
	{ 0x645f2325, "single_release" },
	{ 0x654713d, "dma_set_mask" },
	{ 0x362f9a8, "__x86_indirect_thunk_r12" },
	{ 0xb1c8a784, "kmalloc_trace" },
	{ 0x77358855, "iomem_resource" },
	{ 0x960c2727, "single_open" },
	{ 0x1a976912, "pci_write_config_word" },
	{ 0xa65ac370, "pci_free_irq_vectors" },
	{ 0xf90a1e85, "__x86_indirect_thunk_r8" },
	{ 0xaad786ae, "__mutex_rt_init" },
	{ 0x8867dbd7, "class_register" },
	{ 0xc4f0da12, "ktime_get_with_offset" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xcf0061aa, "kmalloc_caches" },
	{ 0x85bd1608, "__request_region" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x6833069, "module_layout" },
};

MODULE_INFO(depends, "i2c-algo-bit");

MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000014sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000016sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000017sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000018sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000019sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000001Asv*sd*bc*sc*i*");
MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0013d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0C72p0014d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "05D9F3722F526359885CEB0");
