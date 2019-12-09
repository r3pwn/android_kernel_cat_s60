#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>

#include "lepton_types.h"
#include "vospiGeneric.h"

#define PACKET_SIZE (164)
#define BUFFER_SIZE (PACKET_SIZE * 63)

#define VDDP4_MIN_UV	1750000
#define VDDP4_MAX_UV	1950000
#define I2CPULL_MIN_UV	1750000
#define I2CPULL_MAX_UV	1950000
#define LEVELSHIFT_MIN_UV	1750000
#define LEVELSHIFT_MAX_UV	1950000

static irq_handler_t irq_gpio_vsync(unsigned int irq, void *dev_id, struct pt_regs *regs);

struct spi_message spi_msg;

u8 *tx_buf; //This needs to be DMA friendly buffer
u8 *rx_buf;
struct spi_test_dev	{
    struct spi_device	*spi;
    struct miscdevice	spi_test_miscdevice;
    struct timer_list	spi_test_timer;
    struct work_struct	spi_test_work;
    struct mutex         spi_mutex;
    struct regulator *vddp4;
    struct regulator *i2cpull;
    struct regulator *levelshift;
};

static struct spi_test_dev *this_spi_test_dev;
//static struct task_struct *spi_read_task, *spi_watchdog_task;
//static struct task_struct *spi_read_task;
//static struct task_struct *spi_watchdog_task;

#define FRAME_COUNTER_MASK          0xF
#define VOSPI_PACKET_HEADER_SIZE    4
static LEP_RESULT lep_result = LEP_OK;
static LEP_FRAME_STATUS_E frame_status = LEP_FRAME_NOT_READY;

static VIDEO_FORMAT_CONFIG_T videoFormatConfig;
static LEP_DEBUG_INFO_T lepDebugInfo;

static int spiRoutineDstOffset = 0;

//static int should_run_watchdog = 0;

//static time_t prevTime;

u8 *segmentBuffer;
static LEP_FRAME_DATA_T frameBuffer0;
static LEP_FRAME_DATA_T frameBuffer1;
static LEP_FRAME_DATA_T* wrBuff;
static LEP_FRAME_DATA_T* rdBuff;

static int isConfigured = 0;

/* Frame statistics */
static uint32_t numFramesPerSecond = 0; //used for fps
static uint32_t hostActivityCounter = 0;

static VOSPI_HOOK_CONFIG_T captureConfig;

//static ktime_t timestamp;
//static uint64_t timestamp_ns_last_frame_recv;
static struct timespec timespec_last_frame;


static uint32_t gpio_rest = 0;
static uint32_t gpio_pdwn = 0;
static uint32_t gpio_vsync = 0;
static uint32_t gpio_vddc_en = 0;
static uint32_t gpio_vddio_en = 0;
static uint32_t gpio_vdd_en = 0;

void swap_byte_order(uint8_t* mem, int bytes)
{
    uint8_t* memEnd = mem + bytes;
    uint8_t temp;
    while(mem < memEnd)
    {
        temp = *mem;
        *mem = *(mem+1);
        *(mem+1)=temp;
        mem += 2;
    }
}
void swap_word_order(uint8_t* mem, int bytes)
{
    uint16_t* memCurr = (uint16_t*)mem;
    uint16_t* memEnd = (uint16_t*)(mem + bytes);
    uint16_t temp;
    while(memCurr < memEnd)
    {
        temp = *memCurr;
        *memCurr = *(memCurr+1);
        *(memCurr+1)=temp;
        memCurr += 2;
    }
}



int lep_vospi_init(void)
{
   int status = 0;

    /* Set up default params */
    VOSPI_HOOK_CONFIG_T captureConfigDefault;
    captureConfigDefault.lepVersion = LEPTON_2_80x60;
    captureConfigDefault.vidFormat = VIDEO_OUTPUT_FORMAT_RAW14;
    captureConfigDefault.telemetryEnableState = TELEMETRY_ENABLED;
    lep_vospi_configure(captureConfigDefault);
    
    
    spiRoutineDstOffset = 0; 

    /* Setup SPI Device */


    return status;
}

int lep_vospi_configure(VOSPI_HOOK_CONFIG_T config)
{
    uint8_t isValid = 0;

    captureConfig = config;
    
    if( captureConfig.lepVersion == LEPTON_2_80x60 )
    {
        videoFormatConfig.expectedNumSegments = 0;
        videoFormatConfig.spiTimeoutWait = 68;
        videoFormatConfig.telemetryPacketsPerSegment = 3;
        videoFormatConfig.packetsPerSegment = 60;

        if(captureConfig.telemetryEnableState == TELEMETRY_ENABLED)
        {
            videoFormatConfig.packetsPerSegment += videoFormatConfig.telemetryPacketsPerSegment;
        }

        if(captureConfig.vidFormat == VIDEO_OUTPUT_FORMAT_RAW14)
        {
            videoFormatConfig.packetByteSize = 164;
            videoFormatConfig.packetVideoPayloadByteSize = 160;
        }
        else if(captureConfig.vidFormat == VIDEO_OUTPUT_FORMAT_RGB888)
        {
            videoFormatConfig.packetByteSize = 244;
            videoFormatConfig.packetVideoPayloadByteSize = 240;
        }
        videoFormatConfig.segmentVideoByteSize = videoFormatConfig.packetVideoPayloadByteSize * videoFormatConfig.packetsPerSegment;
        videoFormatConfig.segmentByteSize = videoFormatConfig.packetByteSize * videoFormatConfig.packetsPerSegment;
        videoFormatConfig.videoFrameByteSize = videoFormatConfig.segmentVideoByteSize;
        isValid = 1;
    }
    
    if(isValid)
    {
        rdBuff = &frameBuffer0;
        wrBuff = &frameBuffer1;

        isConfigured = 1;
    }

    return !isValid;
}


static int spi_read_bytes(u8* rx, int nBytes)
{
    int ret;
    struct spi_device *spi = this_spi_test_dev->spi;
    struct spi_message msg;
    struct spi_transfer spixfer;
    spi->mode |= SPI_NO_CS;

    spi_message_init(&msg);
    spixfer.tx_buf = tx_buf;
    spixfer.rx_buf = rx;
    spixfer.len = nBytes;
    spixfer.bits_per_word = 8;
    /*
    * Spi master driver will round the speed_hz to the lower nearest frequency in the spi clock table ftbl_blsp*_qup*_spi_apps_clk_src[] with kernel/drivers/clk/qcom/clock-gcc-(chipset).c
    */
    spixfer.speed_hz = spi->max_speed_hz;
    spi_message_add_tail(&spixfer, &msg);
    ret = spi_sync(spi, &msg);   
    return ret;
}


void lep_2_spi_routine( void )
{
	int status = 0;
	uint8_t i;

	LEP_FRAME_DATA_T* tSwap;

	status = spi_read_bytes(   (uint8_t*)segmentBuffer,
		                   (int)(videoFormatConfig.segmentByteSize));   

	for(i = 0; i < 60; i++)
	{
		if ( segmentBuffer[ (i * videoFormatConfig.packetByteSize) + 1 ] != i )
		{
			status = 1;
		}
	}
	if(status == 0)
	{
		numFramesPerSecond++;
	}
	

	//if(status == 0)
	{
            memcpy( &wrBuff->pixels[0], &segmentBuffer[0], videoFormatConfig.segmentByteSize);

	    //swap_byte_order((uint8_t*)wrBuff->pixels, FRAME_NUM_PIXEL);

	    mutex_lock(&this_spi_test_dev->spi_mutex);

	    getnstimeofday(&timespec_last_frame);

	    tSwap = rdBuff;
	    rdBuff = wrBuff;
	    wrBuff = tSwap;

	    spiRoutineDstOffset = 0;
	    //numFramesPerSecond++;
		    
	    mutex_unlock(&this_spi_test_dev->spi_mutex);
	    frame_status = LEP_FRAME_READY;

	}

}

static int lepRegulatorOn(bool on)
{
    struct spi_device *spi = this_spi_test_dev->spi;
    int rc = 0;

    printk(KERN_ERR "Louis %s:\n", __func__);
    if (!on) {
        rc = regulator_disable(this_spi_test_dev->vddp4);
        if (rc) {
            dev_err(&spi->dev, "Regulator vddp4 disable failed rc=%d\n", rc);
        }

        rc = regulator_disable(this_spi_test_dev->i2cpull);
        if (rc) {
            dev_err(&spi->dev, "Regulator i2cpull disable failed rc=%d\n", rc);
        }

        rc = regulator_disable(this_spi_test_dev->levelshift);
        if (rc) {
            dev_err(&spi->dev, "Regulator levelshift disable failed rc=%d\n", rc);
        }
    } else {
        rc = regulator_enable(this_spi_test_dev->vddp4);
        if (rc) {
            dev_err(&spi->dev, "Regulator vddp4 enable failed rc=%d\n", rc);
            regulator_disable(this_spi_test_dev->vddp4);
        }

        rc = regulator_enable(this_spi_test_dev->i2cpull);
        if (rc) {
            dev_err(&spi->dev, "Regulator i2cpull enable failed rc=%d\n", rc);
            regulator_disable(this_spi_test_dev->i2cpull);
        }

        rc = regulator_enable(this_spi_test_dev->levelshift);
        if (rc) {
            dev_err(&spi->dev, "Regulator levelshift enable failed rc=%d\n", rc);
            regulator_disable(this_spi_test_dev->levelshift);
        }
    }
    return rc;

}


static int lepRegulatorInit(bool on)
{
    struct spi_device *spi = this_spi_test_dev->spi;
    int rc;

    printk(KERN_ERR "Louis %s:\n", __func__);
    if (!on) {
        if (regulator_count_voltages(this_spi_test_dev->vddp4) > 0)
            regulator_set_voltage(this_spi_test_dev->vddp4, 0, VDDP4_MAX_UV);

        regulator_put(this_spi_test_dev->vddp4);

        if (regulator_count_voltages(this_spi_test_dev->i2cpull) > 0)
            regulator_set_voltage(this_spi_test_dev->i2cpull, 0, I2CPULL_MAX_UV);

        regulator_put(this_spi_test_dev->i2cpull);

        if (regulator_count_voltages(this_spi_test_dev->levelshift) > 0)
            regulator_set_voltage(this_spi_test_dev->levelshift, 0, LEVELSHIFT_MAX_UV);

        regulator_put(this_spi_test_dev->levelshift);
    } else {
        this_spi_test_dev->vddp4 = regulator_get(&spi->dev, "vddp4");
        if (IS_ERR(this_spi_test_dev->vddp4)) {
            rc = PTR_ERR(this_spi_test_dev->vddp4);
            dev_err(&spi->dev, "Regulator get failed vddp4 rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(this_spi_test_dev->vddp4) > 0) {
            rc = regulator_set_voltage(this_spi_test_dev->vddp4, VDDP4_MIN_UV,
            	   VDDP4_MAX_UV);
            if (rc) {
                dev_err(&spi->dev, "Regulator set failed vddp4 rc=%d\n", rc);
                goto reg_vddP4_put;
            }
        }

        this_spi_test_dev->i2cpull = regulator_get(&spi->dev, "i2cpull");
        if (IS_ERR(this_spi_test_dev->i2cpull)) {
            rc = PTR_ERR(this_spi_test_dev->i2cpull);
            dev_err(&spi->dev, "Regulator get failed i2cpull rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(this_spi_test_dev->i2cpull) > 0) {
            rc = regulator_set_voltage(this_spi_test_dev->i2cpull, I2CPULL_MIN_UV,
            	   I2CPULL_MAX_UV);
            if (rc) {
                dev_err(&spi->dev, "Regulator set failed i2cpull rc=%d\n", rc);
                goto reg_i2cpull_put;
            }
        }

        this_spi_test_dev->levelshift = regulator_get(&spi->dev, "levelshift");
        if (IS_ERR(this_spi_test_dev->levelshift)) {
            rc = PTR_ERR(this_spi_test_dev->levelshift);
            dev_err(&spi->dev,"Regulator get failed levelshift rc=%d\n", rc);
            return rc;
        }

        if (regulator_count_voltages(this_spi_test_dev->levelshift) > 0) {
            rc = regulator_set_voltage(this_spi_test_dev->levelshift, LEVELSHIFT_MIN_UV,
            	   LEVELSHIFT_MAX_UV);
            if (rc) {
                dev_err(&spi->dev, "Regulator set failed levelshift rc=%d\n", rc);
                goto reg_levelshift_put;
            }
        }
    }

    return 0;

reg_levelshift_put:
    regulator_put(this_spi_test_dev->levelshift);
reg_i2cpull_put:
    regulator_put(this_spi_test_dev->i2cpull);
reg_vddP4_put:
    regulator_put(this_spi_test_dev->vddp4);
    return rc;
}

void lepPowerOn(void)
{
    printk(KERN_ERR "Louis %s:\n", __func__);

    //Turn on power source
    lepRegulatorOn(true);
    gpio_set_value(gpio_vddc_en, 1);
    gpio_set_value(gpio_vdd_en, 1);
    
    //De-assert PDWN pin (set to high)
    gpio_set_value(gpio_pdwn, 1);
    //Assert RST pin (set to low)
    gpio_set_value(gpio_rest, 0);

    //Enable MCKL
    gpio_set_value(gpio_vddio_en, 1);

    //Wait > 5000 MCLK periods (~0.2ms)
    msleep(5);
    
    //De-assert RST pin (set to high)
    gpio_set_value(gpio_rest, 1);

    msleep(2000);

    /* Hart: This is required for robust video output. */
    spi_read_bytes((uint8_t*)segmentBuffer, (int)164); 

    //should_run_watchdog = 1;
}

void lepPowerOff(void)
{
    printk(KERN_ERR "Louis %s:\n", __func__);

    lepRegulatorOn(false);
    //Assert PWD pin (set to low)
    gpio_set_value(gpio_pdwn, 0);

    //Wait > 100 ms
    msleep(100);

    //Disable MCKL
    gpio_set_value(gpio_vddio_en, 0);

    //Turn off power source
    gpio_set_value(gpio_vddc_en, 0);
    gpio_set_value(gpio_vdd_en, 0);

    //should_run_watchdog = 0;
}

//int device_ioctl(struct inode *inode, struct file *filep, unsigned int cmd, unsigned long arg) {
long device_ioctl(struct file *filep, unsigned int cmd, unsigned long arg) {
	long len = 0;
	unsigned long copyRet;
        struct timespec current_time;

	switch(cmd) {

    case GET_FRAME_READY_STATUS_IOCTL:
        len = sizeof(LEP_FRAME_STATUS_E);
        copyRet = copy_to_user((char*)arg, &frame_status, len);
        break;

    case READ_FRAME_IOCTL:
        mutex_lock(&this_spi_test_dev->spi_mutex);        
        rdBuff->result = LEP_OK;
        len = sizeof(LEP_FRAME_DATA_T);
        copyRet = copy_to_user((char*)arg, (char*)rdBuff, len);
        mutex_unlock(&this_spi_test_dev->spi_mutex);
        frame_status = LEP_FRAME_NOT_READY;

        break;

    case START_SPI_IOCTL:
        lepPowerOn();
        len = sizeof(LEP_RESULT);
        copyRet = copy_to_user((char*)arg, &lep_result, len);
        break;

    case STOP_SPI_IOCTL:
        lepPowerOff();
        len = sizeof(LEP_RESULT);
        copyRet = copy_to_user((char*)arg, &lep_result, len);
        break;

    case GET_DEBUG_INFO_IOCTL:
        len = sizeof(LEP_DEBUG_INFO_T);

        //timestamp = ktime_get_real();
        //lepDebugInfo.timeSinceLastFrame = (uint64_t)(ktime_to_ns(timestamp) - timestamp_ns_last_frame_recv);
        getnstimeofday(&current_time);
        lepDebugInfo.timeSinceLastFrame = (uint64_t)(current_time.tv_nsec - timespec_last_frame.tv_nsec);


        copyRet = copy_to_user((char*)arg, (char*)&lepDebugInfo, len);
        break;

	default:
		return -ENOTTY;
	}
	
	if(copyRet != 0)
	{
		printk("Failed to copy_to_user %lu bytes" , copyRet);
	}
        hostActivityCounter++;

	return len;

}
static struct file_operations fops = {
	.unlocked_ioctl = device_ioctl,
};

static void spi_test_worker(struct work_struct *work)
{
    
    lepDebugInfo.gpioSignalsSinceBoot++;
    lep_2_spi_routine();
    
}

static irq_handler_t irq_gpio_vsync(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
    schedule_work(&this_spi_test_dev->spi_test_work);

    return (irq_handler_t) IRQ_HANDLED;
}

#if 0
int spi_watchdog_thread(void *data)
{
    int seconds_since_host_activity = 0;
    __set_current_state(TASK_INTERRUPTIBLE);

   // msleep(3000);

    while (!kthread_should_stop()) {

        msleep(1000);

        lepDebugInfo.framesPerSecond = numFramesPerSecond;
        

        if(should_run_watchdog == 1)
        {
#if 0
            if( numFramesPerSecond < 20 || numFramesPerSecond > 32 )
            {
                printk("lepton_drv: Abormal SPI Data, Auto-recovery initiated\n");
                should_read_spi = 0;
            msleep( videoFormatConfig.spiTimeoutWait );
                should_read_spi = 1;
            }
            else if( hostActivityCounter == 0 )
#endif
#if 1
            if( hostActivityCounter == 0 )
            {
                seconds_since_host_activity++;
                if(seconds_since_host_activity >= 5)
                {
                    //printk("lepton_drv: Host read timeout, shut down initiated\n");
                    //lepPowerOff();
                    //should_run_watchdog = 0;
                    //seconds_since_host_activity = 0;
                }
            }
            else
            {
                seconds_since_host_activity = 0;
        }
#endif
        }

        numFramesPerSecond = 0;
        hostActivityCounter = 0;
        

        
    }
    return 0;  
}
#endif
#if 0
int spi_read_thread(void *data)
{
    
    __set_current_state(TASK_INTERRUPTIBLE);
    while (!kthread_should_stop()) {
        lep_2_spi_routine();
    }
    return 0;    
}
#endif

static int spi_test_probe(struct spi_device *spi)
{
    int irq;
    int cs;
    int cpha,cpol,cs_high;
    u32 max_speed;
    
    int ret;		
    struct spi_test_dev *spi_test_dev;
    
    printk(KERN_ERR "Louis %s:\n", __func__);
    dev_err(&spi->dev, "louis - %s\n", __func__);

    //should_run_watchdog = 0;
    
    //allocate memory for transfer
    tx_buf = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
    if(tx_buf == NULL){
        dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
        return -ENOMEM;
    }
    rx_buf = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
    if(rx_buf == NULL){
        dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
        return -ENOMEM;
    }
    segmentBuffer = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
    if(segmentBuffer == NULL){
        dev_err(&spi->dev, "%s: mem alloc failed\n", __func__);
        return -ENOMEM;
    }

    //Parse data using dt.
    if(spi->dev.of_node){
        gpio_rest= of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-rest", 0, NULL);
        gpio_pdwn= of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-pdwn", 0, NULL);
        gpio_vsync = of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-vsync", 0, NULL);
        gpio_vddc_en= of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-vddc-en", 0, NULL);
        gpio_vddio_en= of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-vddcio-en", 0, NULL);
        gpio_vdd_en= of_get_named_gpio_flags(spi->dev.of_node, "lepton3-spi,gpio-vdd-en", 0, NULL);
    }
    irq = spi->irq;
    cs = spi->chip_select;
    cpha = ( spi->mode & SPI_CPHA ) ? 1:0;
    cpol = ( spi->mode & SPI_CPOL ) ? 1:0;
    cs_high = ( spi->mode & SPI_CS_HIGH ) ? 1:0;
    max_speed = spi->max_speed_hz;
    dev_err(&spi->dev, "gpio [%d] irq [%d] gpio_irq [%d] cs [%x] CPHA [%x] CPOL [%x] CS_HIGH [%x]\n",
    gpio_vsync, irq, gpio_to_irq(gpio_vsync), cs, cpha, cpol, cs_high);
    dev_err(&spi->dev, "Max_speed [%d]\n", max_speed );
    //Transfer can be done after spi_device structure is created
    spi->bits_per_word = 8;
    //dev_err(&spi->dev, "SPI sync returned [%d]\n", spi_test_transfer(spi));

    spi_test_dev = kzalloc(sizeof(*spi_test_dev), GFP_KERNEL);
    if (spi_test_dev == NULL) {
        dev_err(&spi->dev, "failed to allocate memory for module data\n");
        return -ENODEV;
    }

    spi_test_dev->spi = spi;

    spi_test_dev->spi_test_miscdevice.minor = MISC_DYNAMIC_MINOR;
    spi_test_dev->spi_test_miscdevice.name = "lepton_drv";
    spi_test_dev->spi_test_miscdevice.fops = &fops;

    ret = misc_register(&spi_test_dev->spi_test_miscdevice);

    lep_vospi_init();

    rdBuff = &frameBuffer0;
    wrBuff = &frameBuffer1;
    
    /* TODO: Replace with GPIO IRQ */
    //kernel thread
#if 0
    spi_watchdog_task = kthread_create(spi_watchdog_thread, NULL, "spi_watchdog");
    if(IS_ERR(spi_watchdog_task)) {
        dev_err(&spi->dev, "%s: kthread_create spi_watchdog_task failed\n", __func__);
        spi_watchdog_task = NULL;
    }
#endif
#if 0
    spi_read_task = kthread_create(spi_read_thread, NULL, "spi_read");
    if (IS_ERR(spi_read_task)) {
        dev_err(&spi->dev, "%s: kthread_create spi_read failed\n", __func__);
        spi_read_task = NULL;
    }
#endif
    mutex_init(&spi_test_dev->spi_mutex);
    this_spi_test_dev = spi_test_dev;

    ret = gpio_request(gpio_rest, "TH_REST");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_REST gpio PIN %d.\n", gpio_rest);
    }
    ret = gpio_request(gpio_pdwn, "TH_PWR_DWN");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_PWR_DWN gpio PIN %d.\n", gpio_pdwn);
    }
    ret = gpio_request(gpio_vsync, "TH_GPIO3_VSYNC");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_GPIO3_VSYNC gpio PIN %d.\n", gpio_vsync);
    }
    ret = gpio_request(gpio_vddc_en, "TH_VDDC_EN");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_VDDC_EN gpio PIN %d.\n", gpio_vddc_en);
    }
    ret = gpio_request(gpio_vddio_en, "TH_VDDIO_EN");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_VDDIO_EN gpio PIN %d.\n", gpio_vddio_en);
    }
    ret = gpio_request(gpio_vdd_en, "TH_VDD_EN");
    if (ret) {
        dev_err(&spi->dev, "Fail to request TH_VDD_EN gpio PIN %d.\n", gpio_vdd_en);
    }
    ret = gpio_direction_output(gpio_rest, 0);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_REST gpio.\n");
    }
    ret = gpio_direction_output(gpio_pdwn, 0);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_PWR_DWN gpio.\n");
    }
    ret = gpio_direction_input(gpio_vsync);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_GPIO3_VSYNC gpio.\n");
    }
    ret = gpio_direction_output(gpio_vddc_en, 0);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_VDDC_EN gpio.\n");
    }
    ret = gpio_direction_output(gpio_vddio_en, 0);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_VDDIO_EN gpio.\n");
    }
    ret = gpio_direction_output(gpio_vdd_en, 0);
    if (ret) {
        dev_err(&spi->dev, "Fail to set TH_VDD_EN gpio.\n");
    }

    irq = gpio_to_irq( gpio_vsync );
    ret = request_irq( irq, (irq_handler_t)irq_gpio_vsync, IRQF_TRIGGER_RISING, "flir_lepton_vsync", NULL);
    
    INIT_WORK(&spi_test_dev->spi_test_work, spi_test_worker);
    lepRegulatorInit(true);

    //wake_up_process(spi_watchdog_task);

    return 0;
}

#ifdef CONFIG_OF //Open firmware must be defined for dts useage
static struct of_device_id spidev_match_table[] = {
    { .compatible = "flir,lepton3-spi",}, //Compatible node must match
    //dts
    { },
};
#else
#define spidev_match_table NULL
#endif

static const struct spi_device_id spidev_device_id[] = {
	{ "lepton3-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, spidev_device_id);
//SPI Driver Info
static struct spi_driver spi_test_driver = {
    .driver = {
        .name = "lepton3-spi",
        .owner = THIS_MODULE,
        .of_match_table = spidev_match_table,
    },
    .probe = spi_test_probe,
    .id_table = spidev_device_id,
};

static int __init spi_test_init(void)
{
    printk(KERN_ERR "Louis %s:\n", __func__);
    return spi_register_driver(&spi_test_driver);
}

static void __exit spi_test_exit(void)
{
    printk(KERN_ERR "Louis %s:\n", __func__);
    spi_unregister_driver(&spi_test_driver);
}

module_init(spi_test_init);
module_exit(spi_test_exit);
MODULE_DESCRIPTION("SPI TEST");
MODULE_LICENSE("GPL v2");
