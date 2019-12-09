#include <linux/module.h>
#include <linux/cei_hw_id.h>

#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include <soc/qcom/smsm.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/seq_file.h>

static struct cei_hwid_data_type *cei_hwid_data;

static const char cei_proj_id_str[][CEI_HWID_TYPE_STRLEN] = {
	"BT52",
	"BT53",
	"B28A",
	"BT54",
	"Undefined"
};

static const char cei_hw_id_str[][CEI_HWID_TYPE_STRLEN] = {
	"EVT",
	"DVT1",
	"DVT2",
	"PVT",
	"MP",
	"Invalid"
};


static void cei_hwid_read_smem(void)
{
	unsigned hwid_smem_size;

	cei_hwid_data = (struct cei_hwid_data_type *)(smem_get_entry
			(SMEM_ID_VENDOR0, &hwid_smem_size, 0,
			SMEM_ANY_HOST_FLAG));

	pr_err("cei_proj_id = %ld, cei_hw_id = %ld, cei_qfuse = %ld size=%ld\n",
			cei_hwid_data->cei_proj_id,
			cei_hwid_data->cei_hw_id, cei_hwid_data->cei_qfuse,
			sizeof(struct cei_hwid_data_type));
}

long get_cei_proj_id(void)
{
	return cei_hwid_data->cei_proj_id;
}
EXPORT_SYMBOL(get_cei_proj_id);

long get_cei_hw_id(void)
{
	return cei_hwid_data->cei_hw_id;
}
EXPORT_SYMBOL(get_cei_hw_id);

long get_cei_qfuse(void)
{
	return cei_hwid_data->cei_qfuse;
}
EXPORT_SYMBOL(get_cei_qfuse);

static int cei_proj_id_read(struct seq_file *m, void *v)
{
	int proj_id;

	proj_id = get_cei_proj_id();
	if ((proj_id >= PROJ_ID_SIZE) || (proj_id < 0))
		seq_printf(m, "%s\n", cei_proj_id_str[PROJ_ID_SIZE - 1]);
	else
		seq_printf(m, "%s\n", cei_proj_id_str[proj_id]);


	return 0;
}

static int cei_hw_id_read(struct seq_file *m, void *v)
{
	int hw_id;

	hw_id = get_cei_hw_id();
	if ((hw_id >= HW_ID_SIZE) || (hw_id < 0))
		seq_printf(m, "%s\n", cei_hw_id_str[HW_ID_SIZE - 1]);
	else
		seq_printf(m, "%s\n", cei_hw_id_str[hw_id]);
	return 0;
}

static int cei_qfuse_read(struct seq_file *m, void *v)
{
	int qfuse;

	qfuse = get_cei_qfuse();
	if ((qfuse != true) && (qfuse != false))
		seq_puts(m, "unknown\n");
	else
		seq_printf(m, "%s\n", qfuse ? "true" : "false");

	return 0;
}

static int cei_proj_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_proj_id_read, NULL);
}

static int cei_hw_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_hw_id_read, NULL);
}

static int cei_qfuse_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cei_qfuse_read, NULL);
}

static const struct file_operations cei_proj_id_fops = {
	.open       = cei_proj_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations cei_hw_id_fops = {
	.open       = cei_hw_id_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static const struct file_operations cei_qfuse_fops = {
	.open       = cei_qfuse_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};
void cei_hwid_info_read(void)
{
	cei_hwid_read_smem();
	proc_create("cei_proj_id", 0, NULL, &cei_proj_id_fops);
	proc_create("cei_hw_id", 0, NULL, &cei_hw_id_fops);
	proc_create("cei_qfuse", 0, NULL, &cei_qfuse_fops);
}
EXPORT_SYMBOL(cei_hwid_info_read);

static int __init cei_hw_id_init(void)
{
	int err = 0;

	cei_hwid_info_read();
	return err;
}

static void __exit cei_hw_id_exit(void)
{
	pr_info("cei_hwid_exit enter\n");
}

arch_initcall(cei_hw_id_init);
module_exit(cei_hw_id_exit);

MODULE_DESCRIPTION("cei hardware ID driver");
MODULE_LICENSE("GPL");
