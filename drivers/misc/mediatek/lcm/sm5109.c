
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif
#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>	/* hwPowerOn */
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#endif
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include "lcm_define.h"
#include "lcm_drv.h"
#include "lcm_i2c.h"




/*****************************************************************************
 * Define
 *****************************************************************************/
#define LCM_GPIO_DEVICES	"lcm_mode1"
#define PC_LCM_GPIO_MODE_00	0
#define PC_MAX_LCM_GPIO_MODE	12
#define LCM_I2C_ID_NAME "sm5109"
#define LCD_BIAS_ID_STATUS_FLOAT 0x02
/*****************************************************************************
 * Function GPIO
 *****************************************************************************/
 static struct pinctrl *_lcm_gpio;
/*static struct pinctrl_state *_lcm_gpio_enp_mode;
 static struct pinctrl_state *_lcm_gpio_enn_mode;
 static struct pinctrl_state *_lcm_gpio_rst_mode;*/
 static struct pinctrl_state *_lcm_gpio_mode[PC_MAX_LCM_GPIO_MODE];
 static unsigned char _lcm_gpio_mode_list[PC_MAX_LCM_GPIO_MODE][128] = {
	 "state_enp_output0",
	 "state_enp_output1",
	 "state_enn_output0",
	 "state_enn_output1",
	 "state_reset_output0",
	 "state_reset_output1",
	 "state_tp_reset_output0",
	 "state_tp_reset_output1",
	 "state_ldo_output0",
	 "state_ldo_output1",
	 "state_led_output0",
	 "state_led_output1",
 };


 /* function definitions */
 static int __init _lcm_gpio_init(void);
 static void __exit _lcm_gpio_exit(void);
 static int _lcm_gpio_probe(struct platform_device *pdev);
 static int _lcm_gpio_remove(struct platform_device *pdev);

 static const struct of_device_id _lcm_gpio_of_idss[] = {
	 {.compatible = "mediatek,lcm_gpio",},
	 {},
 };
 //MODULE_DEVICE_TABLE(of, _lcm_gpio_of_idss);


 static struct platform_driver _lcm_gpio_driver = {
	 .driver = {
		 .name = LCM_GPIO_DEVICES,
		 .owner  = THIS_MODULE,
		 .of_match_table = of_match_ptr(_lcm_gpio_of_idss),
	 },
	 .probe = _lcm_gpio_probe,
	 .remove = _lcm_gpio_remove,
 };
 //module_platform_driver(_lcm_gpio_driver);

 /*****************************************************************************
  * Function I2C
  *****************************************************************************/

 static struct i2c_client *_lcm_i2c_client;
 static int _lcm_i2c_probe(struct i2c_client *client,
	 const struct i2c_device_id *id);
 static int _lcm_i2c_remove(struct i2c_client *client);

 struct _lcm_i2c_dev {
	 struct i2c_client *client;

 };

 static const struct of_device_id _lcm_i2c_of_match[] = {
	 {
	  .compatible = "mediatek,I2C_LCD_BIAS",
	  },
 };

 static const struct i2c_device_id _lcm_i2c_id[] = {
	 {LCM_I2C_ID_NAME, 0},
	 {}
 };
 static struct i2c_driver _lcm_i2c_driver = {
	 .id_table = _lcm_i2c_id,
	 .probe = _lcm_i2c_probe,
	 .remove = _lcm_i2c_remove,
	 .driver = {
			.owner = THIS_MODULE,
			.name = LCM_I2C_ID_NAME,
#ifdef CONFIG_MTK_LEGACY
#else
			.of_match_table = _lcm_i2c_of_match,
#endif
			},

 };
 static int _lcm_i2c_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
 {
	 pr_debug("[LCM][I2C] _lcm_i2c_probe\n");
	 pr_debug("[LCM][I2C] NT: info==>name=%s addr=0x%x\n",
		 client->name, client->addr);
	 _lcm_i2c_client = client;
	 return 0;
 }


 static int _lcm_i2c_remove(struct i2c_client *client)
 {
	 pr_debug("[LCM][I2C] _lcm_i2c_remove\n");
	 _lcm_i2c_client = NULL;
	 i2c_unregister_device(client);
	 return 0;
 }
/* static int _lcm_i2c_write_bytes(unsigned char addr, unsigned char value)
 {
	 int ret = 0;
	 struct i2c_client *client = _lcm_i2c_client;
	 char write_data[2] = { 0 };

	 if (client == NULL) {
		 pr_debug("ERROR!! _lcm_i2c_client is null\n");
		 return 0;
	 }
	 pr_debug("[LCM][I2C] __lcm_i2c_write_bytes\n");

	 write_data[0] = addr;
	 write_data[1] = value;
	 ret = i2c_master_send(client, write_data, 2);
	 if (ret < 0)
		 pr_debug("[LCM][ERROR] _lcm_i2c write data fail !!\n");

	 return ret;
 }
 static int _lcm_i2c_read_bytes(unsigned char addr, unsigned char value)
 {
	 int ret = 0;
	 struct i2c_client *client = _lcm_i2c_client;
	 char read_data[2] = { 0 };
	 pr_debug("[LCM][I2C] __lcm_i2c_read_bytes\n");

	 if (client == NULL) {
		 pr_debug("ERROR!! _lcm_i2c_client is null\n");
		 return 0;
	 }

	 read_data[0] = addr;
	 read_data[1] = value;
	 ret = i2c_master_recv(client, read_data, 2);
	 if (ret < 0)
		 pr_debug("[LCM][ERROR] _lcm_i2c read data fail !!\n");

	 return ret;
 }
 static int SM5109_REG_MASK (unsigned char addr, unsigned char val, unsigned char mask)
 {
		 unsigned char SM5109_reg=0;
		 unsigned char ret = 0;
		 pr_debug("[LCM][I2C] SM5109_REG_MASK\n");

		 ret = _lcm_i2c_read_bytes(addr, SM5109_reg);

		 SM5109_reg &= ~mask;
		 SM5109_reg |= val;

		 ret = _lcm_i2c_write_bytes(addr, SM5109_reg);

		 return ret;
 }*/

  int lcm_power_enable(void){
 	int ret=0;
	char * ptr;
	int lcd_bias_id = 0x02;

	pr_debug("[LCM]power enable\n");

	ptr = strstr(saved_command_line, "lcd_bias_id=");
	ptr += strlen("lcd_bias_id=");
	lcd_bias_id = simple_strtol(ptr, NULL, 10);

	pr_debug("kernel hd lcd_bias_id = %0d, this parameter from lk transmit to kernel through cmdline\n", lcd_bias_id);

	//from cmdline get LCD Bias IC compatible solutions value
	if( lcd_bias_id == LCD_BIAS_ID_STATUS_FLOAT )
	{
		pr_debug("This is NT bias IC NT50358A\n");
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[1]);    //enable enp
		mdelay(10);
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[3]);    //enable enn
		mdelay(5);
	}
	else
	{
		pr_debug("This is TI bias IC TPS65132S\n");
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[3]);    //enable TI tps65132s SYNC to 150mA
		mdelay(5);
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[1]);    //enable TI tps65132s EN pin
		mdelay(10);
	}

	/* set AVDD*/
	/*4.0V + 20* 100mV*/
	//ret = SM5109_REG_MASK(0x00, 20, (0x1F << 0));
	//if (ret < 0)
	//		pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x00);
	//else
	//		pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x00);

	/* set AVEE */
	/*-4.0V - 20* 100mV*/
	//ret = SM5109_REG_MASK(0x01, 20, (0x1F << 0));
	//if (ret < 0)
	//		pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x01);
	//else
	//		pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x01);

	/* enable AVDD & AVEE discharge*/
	//ret = SM5109_REG_MASK(0x03, (1<<0) | (1<<1), (1<<0) | (1<<1));
	//if (ret < 0)
	//		pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x03);
	//else
	//		pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x03);

	return ret;

 }

int lcm_power_enable_fhd(void){
 	int ret=0;
	char * ptr;
	int lcd_bias_id = 0x02;

	pr_debug("[LCM]power enable_fhd\n");

	//from cmdline get LCD Bias IC compatible solutions value
	ptr = strstr(saved_command_line, "lcd_bias_id=");
	ptr += strlen("lcd_bias_id=");
	lcd_bias_id = simple_strtol(ptr, NULL, 10);

	pr_debug("kernel fhd lcd_bias_id = %0d, this parameter from lk transmit to kernel through cmdline\n", lcd_bias_id);

	if( lcd_bias_id == LCD_BIAS_ID_STATUS_FLOAT )
	{
		pr_debug("This is NT bias IC NT50358A\n");
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[1]);    //enable enp
		mdelay(6);
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[3]);    //enable enn
		mdelay(5);
	}
	else
	{
		pr_debug("This is TI bias IC TPS65132S\n");
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[3]);    //enable TI tps65132s SYNC to 150mA
		mdelay(5);
		pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[1]);    //enable TI tps65132s EN pin
		mdelay(6);
	}

	/* set AVDD*/
	/*4.0V + 15* 100mV*/
	/*ret = SM5109_REG_MASK(0x00, 15, (0x1F << 0));
	if (ret < 0)
			pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x00);
	else
			pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x00);*/

	/* set AVEE */
	/*-4.0V - 15* 100mV*/
	/*ret = SM5109_REG_MASK(0x01, 15, (0x1F << 0));
	if (ret < 0)
			pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x01);
	else
			pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x01);*/

	/* enable AVDD & AVEE discharge*/
	/*ret = SM5109_REG_MASK(0x03, (1<<0) | (1<<1), (1<<0) | (1<<1));
	if (ret < 0)
			pr_debug("ili9881h----cmd=%0x--i2c write error----\n", 0x03);
	else
			pr_debug("ili9881h----cmd=%0x--i2c write success----\n", 0x03);*/

	return ret;

 }

 EXPORT_SYMBOL(lcm_power_enable_fhd);

int lcm_power_enable_vdd3(void){
 	int ret=0;
	pr_debug("[LCM]power enable vdd3\n");
	mdelay(1);
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[9]);    //enable vdd3.3
	return ret;

 }

 EXPORT_SYMBOL(lcm_power_enable_vdd3);

 int lcm_power_disable(void){
 	pr_debug("[LCM]power disable\n");
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[2]);    //pull down enn
	//mdelay(1);
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[0]);    //pull down enp
	return 0;
 }
 EXPORT_SYMBOL(lcm_power_disable);

 int lcm_power_disable_vdd3(void){
 	pr_debug("[LCM]power disable vdd3\n");
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[8]);    //pull down vdd3
	return 0;
 }
 EXPORT_SYMBOL(lcm_power_disable_vdd3);

 int lcm_power_enable_led(void){
	int ret=0;
	pr_debug("[LCM]backlight enable led_en pin\n");
	mdelay(80);
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[11]);    //enable led_en pin
	return ret;

 }
 EXPORT_SYMBOL(lcm_power_enable_led);

 int lcm_power_disable_led(void){
	pr_debug("[LCM]backlight disable led_en pin\n");
	pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[10]);    //disable led_en pin
	return 0;
 }
 EXPORT_SYMBOL(lcm_power_disable_led);

 enum Color {
   LOW,
   HIGH
 };
  void lcm_reset_pin(unsigned int mode)
 {
	 pr_debug("[LCM][GPIO]lcm_reset_pin mode:%d !\n",mode);
	 if((mode==0)||(mode==1)){
		 switch (mode){
			 case LOW :
				 pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[4]);
				 break;
			 case HIGH:
				 pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[5]);
				 break;
			 default:
				 break;

		 }

 	}
 }
 EXPORT_SYMBOL(lcm_reset_pin);

  void ctp_reset_pin(unsigned int mode)
 {
	 pr_debug("[LCM][GPIO]ctp_reset_pin mode:%d !\n",mode);
	 if((mode==0)||(mode==1)){
		 switch (mode){
			 case LOW :
				 pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[6]);
				 break;
			 case HIGH:
				 pinctrl_select_state(_lcm_gpio, _lcm_gpio_mode[7]);
				 break;
			 default:
				 break;

		 }

 	}
 }
 EXPORT_SYMBOL(ctp_reset_pin);

 static int _lcm_gpio_probe(struct platform_device *pdev){
		 int ret;
		 unsigned int mode;
		 struct device	 *dev = &pdev->dev;

		 pr_debug("[LCM][GPIO] enter %s, %d\n", __func__, __LINE__);

		 _lcm_gpio = devm_pinctrl_get(dev);
		 if (IS_ERR(_lcm_gpio)) {
			 ret = PTR_ERR(_lcm_gpio);
		 pr_debug("[LCM][ERROR] Cannot find _lcm_gpio!\n");
			 return ret;
		 }
		 for (mode = PC_LCM_GPIO_MODE_00; mode < PC_MAX_LCM_GPIO_MODE; mode++) {
			 _lcm_gpio_mode[mode] =
				 pinctrl_lookup_state(_lcm_gpio,
					 _lcm_gpio_mode_list[mode]);
			 if (IS_ERR(_lcm_gpio_mode[mode]))
				 pr_debug("[LCM][ERROR] Cannot find lcm_mode:%d! skip it.\n",
				 mode);
		 }
	 	pr_debug("[LCM][GPIO] exit %s, %d\n", __func__, __LINE__);
 		pr_debug("MediaTek LCM I2C driver init\n");
	 		ret = i2c_add_driver(&_lcm_i2c_driver);
	 	pr_debug("MediaTek LCM I2C driver init ret=%d\n",ret);
	 		if(ret !=0){
		 		pr_debug("[LCM][I2C] _lcm_i2c_init fail\n");
		 		return -1;
	 		}
	 return 0;
 }
  static int _lcm_gpio_remove(struct platform_device *pdev)
 {

	 return 0;
 }

 static int __init _lcm_gpio_init(void)
 {
	 pr_debug("[LCM] LCM GPIO driver init\n");
	 if (platform_driver_register(&_lcm_gpio_driver) != 0) {
		 pr_debug("[LCM]unable to register LCM GPIO driver.\n");
		 return -1;
	 }
	 return 0;
 }

 static void __exit _lcm_gpio_exit(void)
 {
	 pr_debug("MediaTek LCM GPIO driver exit\n");
	 platform_driver_unregister(&_lcm_gpio_driver);
	 pr_debug("[LCM][I2C] _lcm_i2c_exit\n");
	 i2c_del_driver(&_lcm_i2c_driver);

 }

 module_init(_lcm_gpio_init);
 module_exit(_lcm_gpio_exit);

 MODULE_LICENSE("GPL");
 MODULE_DESCRIPTION("ADD SM5109 BIAS driver");
 MODULE_AUTHOR("lianghao2@wingtech.com>");

