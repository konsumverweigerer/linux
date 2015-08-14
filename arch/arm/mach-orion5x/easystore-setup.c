/*
 * arch/arm/mach-orion5x/aspire_easystore-setup.c
 *
 * Acer Aspire EasyStore
 *
 * Maintainer: Peter Gruber <nokos@gmx.net>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/irq.h>
#include <linux/mtd/physmap.h>
#include <linux/leds.h>
#include <linux/gpio_keys.h>
#include <linux/gpio-fan.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/mv643xx_eth.h>
#include <linux/i2c.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/pci.h>
#include <mach/orion5x.h>
#include <plat/orion-gpio.h>
#include "common.h"
#include "mpp.h"

/*****************************************************************************
 * DB-88F5281 on board devices
 ****************************************************************************/

#define A_ES_GPIO_LED_RED		1
#define A_ES_GPIO_LED_BLUE		4
#define A_ES_GPIO_LED_WLAN		15
#define A_ES_GPIO_POWER_OFF		8
#define A_ES_GPIO_KEY_POWER		9
#define A_ES_GPIO_KEY_RESET		10
#define A_ES_GPIO_FAN_SPEED0		13
#define A_ES_GPIO_FAN_SPEED1		14

/*
 * 16M NAND flash on Device bus chip select 1
 */

#define A_ES_NAND_BASE		0xff000000
#define A_ES_NAND_SIZE		SZ_16M

/*
 * PCI
 */

#define A_ES_PCI_SLOT0_OFFS		7
#define A_ES_PCI_SLOT0_IRQ_PIN		2
#define A_ES_PCI_SLOT1_SLOT2_IRQ_PIN	3

/*****************************************************************************
 * 16M NAND Flash on Device bus CS2
 ****************************************************************************/

static struct mtd_partition aspire_easystore_nor_parts[] = {
	{
		.name = "MTD1",
		.offset = 0,
		.size = SZ_128K,
	}, {
		.name = "MTD2",
		.offset = SZ_128K,
		.size = SZ_128K,
	}, {
		.name = "Linux Kernel",
		.offset = SZ_256K,
		.size = (SZ_2M - SZ_512K),
	}, {
		.name = "File System",
		.offset = (SZ_2M - SZ_256K),
		.size = (SZ_16M - SZ_2M - SZ_256K),
	}, {
		.name = "u-boot",
		.offset = (SZ_16M - SZ_512K),
		.size = SZ_512K,
	},
};

static struct resource aspire_easystore_nor_resource = {
	.flags		= IORESOURCE_MEM,
	.start		= A_ES_NAND_BASE,
	.end		= A_ES_NAND_BASE + A_ES_NAND_SIZE - 1,
};

static struct physmap_flash_data aspire_easystore_nor_data = {
	.parts		= aspire_easystore_nor_parts,
	.nr_parts	= ARRAY_SIZE(aspire_easystore_nor_parts),
	.width		= 1,
};

static struct platform_device aspire_easystore_nor_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &aspire_easystore_nor_data,
	},
	.num_resources	= 1,
	.resource	= &aspire_easystore_nor_resource,
};

/****************************************************************************
 * GPIO LEDs (simple - doesn't use hardware blinking support)
 */

static struct gpio_led aspire_easystore_leds[] = {
	{
		.name = "power:red",
		.gpio = A_ES_GPIO_LED_RED,
	},
	{
		.name = "power:blue",
		.gpio = A_ES_GPIO_LED_BLUE,
	},
	{
		.name = "wlan:blue",
		.gpio = A_ES_GPIO_LED_WLAN,
	},
};

static struct gpio_led_platform_data aspire_easystore_led_data = {
	.num_leds	= ARRAY_SIZE(aspire_easystore_leds),
	.leds		= aspire_easystore_leds,
	.gpio_blink_set = orion_gpio_led_blink_set,
};

static struct platform_device aspire_easystore_gpio_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &aspire_easystore_led_data,
	},
};

/****************************************************************************
 * GPIO Attached Keys
 */

static struct gpio_keys_button aspire_easystore_buttons[] = {
	{
		.code		= KEY_POWER,
		.gpio		= A_ES_GPIO_KEY_POWER,
		.desc		= "Power Button",
	},
	{
		.code		= KEY_RESTART,
		.gpio		= A_ES_GPIO_KEY_RESET,
		.desc		= "Reset Button",
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data aspire_easystore_button_data = {
	.buttons	= aspire_easystore_buttons,
	.nbuttons	= ARRAY_SIZE(aspire_easystore_buttons),
};

static struct platform_device aspire_easystore_gpio_button = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &aspire_easystore_button_data,
	},
};

/****************************************************************************
 * GPIO fan control
 */

static struct gpio_fan_speed aspire_easystore_fan_speeds[] = {
	{
		.rpm		= 0,
		.ctrl_val	= 0,
	},
	{
		.rpm		= 500,
		.ctrl_val	= 1,
	},
	{
		.rpm		= 1500,
		.ctrl_val	= 2,
	},
	{
		.rpm		= 2000,
		.ctrl_val	= 3,
	},
};

static unsigned aspire_easystore_fan_ctrls[] = { A_ES_GPIO_FAN_SPEED0, A_ES_GPIO_FAN_SPEED1 };

static struct gpio_fan_platform_data aspire_easystore_fan_data = {
	.num_ctrl	= 2,
	.ctrl		= aspire_easystore_fan_ctrls,
        .num_speed	= ARRAY_SIZE(aspire_easystore_fan_speeds),
        .speed		= aspire_easystore_fan_speeds,
};

static struct platform_device aspire_easystore_gpio_fan = {
	.name		= "gpio-fan",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &aspire_easystore_fan_data,
	},
};

/****************************************************************************
 * GPIO Power Off
 */

static void aspire_easystore_poweroff(void)
{
	pr_info("acer-aspire: Triggering power-off...\n");
	gpio_set_value(A_ES_GPIO_POWER_OFF, 1);
	mdelay(100);
	gpio_set_value(A_ES_GPIO_POWER_OFF, 0);
	mdelay(100);
	gpio_set_value(A_ES_GPIO_POWER_OFF, 1);
}

/*****************************************************************************
 * PCI
 ****************************************************************************/

static void __init aspire_easystore_pci_preinit(void)
{
	int pin;

	/*
	 * Configure PCI GPIO IRQ pins
	 */
	pin = A_ES_PCI_SLOT0_IRQ_PIN;
	if (gpio_request(pin, "PCI Int1") == 0) {
		if (gpio_direction_input(pin) == 0) {
			irq_set_irq_type(gpio_to_irq(pin), IRQ_TYPE_LEVEL_LOW);
		} else {
			printk(KERN_ERR "aspire_easystore_pci_preinit failed to "
					"set_irq_type pin %d\n", pin);
			gpio_free(pin);
		}
	} else {
		printk(KERN_ERR "aspire_easystore_pci_preinit failed to gpio_request %d\n", pin);
	}

	pin = A_ES_PCI_SLOT1_SLOT2_IRQ_PIN;
	if (gpio_request(pin, "PCI Int2") == 0) {
		if (gpio_direction_input(pin) == 0) {
			irq_set_irq_type(gpio_to_irq(pin), IRQ_TYPE_LEVEL_LOW);
		} else {
			printk(KERN_ERR "aspire_easystore_pci_preinit failed "
					"to set_irq_type pin %d\n", pin);
			gpio_free(pin);
		}
	} else {
		printk(KERN_ERR "aspire_easystore_pci_preinit failed to gpio_request %d\n", pin);
	}
}

static int __init aspire_easystore_pci_map_irq(const struct pci_dev *dev, u8 slot,
	u8 pin)
{
	int irq;

	/*
	 * Check for devices with hard-wired IRQs.
	 */
	irq = orion5x_pci_map_irq(dev, slot, pin);
	if (irq != -1)
		return irq;

	/*
	 * PCI IRQs are connected via GPIOs.
	 */
	switch (slot - A_ES_PCI_SLOT0_OFFS) {
	case 0:
		return gpio_to_irq(A_ES_PCI_SLOT0_IRQ_PIN);
	case 1:
	case 2:
		return gpio_to_irq(A_ES_PCI_SLOT1_SLOT2_IRQ_PIN);
	default:
		return -1;
	}
}

static struct hw_pci aspire_easystore_pci __initdata = {
	.nr_controllers	= 2,
	.preinit	= aspire_easystore_pci_preinit,
	.setup		= orion5x_pci_sys_setup,
	.scan		= orion5x_pci_sys_scan_bus,
	.map_irq	= aspire_easystore_pci_map_irq,
};

static int __init aspire_easystore_pci_init(void)
{
	if (machine_is_aspire_easystore())
		pci_common_init(&aspire_easystore_pci);

	return 0;
}

subsys_initcall(aspire_easystore_pci_init);

/*****************************************************************************
 * Ethernet
 ****************************************************************************/
static struct mv643xx_eth_platform_data aspire_easystore_eth_data = {
	.phy_addr	= MV643XX_ETH_PHY_ADDR(8),
};

/*****************************************************************************
 * I2C bus
 ****************************************************************************/
static struct i2c_board_info __initdata aspire_easystore_i2c_devices[] = {
        {
                I2C_BOARD_INFO("m41t80", 0x68),
        }, {
	        I2C_BOARD_INFO("lm75", 0x48),
        },
};

/*****************************************************************************
 * General Setup
 ****************************************************************************/
static unsigned int aspire_easystore_mpp_modes[] __initdata = {
	MPP0_UNUSED,		/* unused? */
	MPP1_GPIO,		/* Red LED */
	MPP2_GPIO,		/* PCI int 1 */
	MPP3_GPIO,		/* PCI int 2 */
	MPP4_GPIO,		/* Blue LED */
	MPP5_GPIO,		/* output? */
	MPP6_GPIO,		/* GMT G751-2f overtemp (0=overtemp) */
	MPP7_GPIO,		/* M41T80 nIRQ? */
	MPP8_GPIO,		/* power off control (1=off) */
	MPP9_GPIO,		/* power switch (1=on, 0=off) */
	MPP10_GPIO,		/* reset button (0=pressed) */
	MPP11_UNUSED,		/* unused? */
	MPP12_GPIO,		/* output? */
	MPP13_GPIO,		/* FAN Speed bit 0 */
	MPP14_GPIO,		/* FAN Speed bit 1 */
	MPP15_GPIO,		/* WLAN LED */
	MPP16_UART,		/* UART1_RX */
	MPP17_UART,		/* UART1_TX */
	MPP18_UART,		/* UART1_CTSn */
	MPP19_UART,		/* UART1_RTSn */
	0,
};

static void __init aspire_easystore_init(void)
{
	/*
	 * Basic Orion setup. Need to be called early.
	 */
	orion5x_init();

	orion5x_mpp_conf(aspire_easystore_mpp_modes);
	writel(0, MPP_DEV_CTRL);		/* DEV_D[31:16] */

	/*
	 * Configure peripherals.
	 */
	orion5x_ehci0_init();
	orion5x_eth_init(&aspire_easystore_eth_data);
	orion5x_i2c_init();
	orion5x_uart0_init();
	orion5x_uart1_init();

	if (gpio_request(A_ES_GPIO_POWER_OFF, "POWEROFF") != 0 ||
	    gpio_direction_output(A_ES_GPIO_POWER_OFF, 0) != 0) {
		pr_err("acer-aspire: failed to setup power-off GPIO\n");
        } else {
            pm_power_off = aspire_easystore_poweroff;
        }

	mvebu_mbus_add_window_by_id(ORION_MBUS_DEVBUS_BOOT_TARGET,
				    ORION_MBUS_DEVBUS_BOOT_ATTR,
				    A_ES_NAND_BASE,
				    A_ES_NAND_SIZE);
	platform_device_register(&aspire_easystore_nor_flash);

	platform_device_register(&aspire_easystore_gpio_leds);
	platform_device_register(&aspire_easystore_gpio_button);
	platform_device_register(&aspire_easystore_gpio_fan);

	i2c_register_board_info(0, aspire_easystore_i2c_devices, ARRAY_SIZE(aspire_easystore_i2c_devices));
}

MACHINE_START(A_ES, "Acer Aspire EasyStore")
	.atag_offset	= 0x100,
	.init_machine	= aspire_easystore_init,
	.map_io		= orion5x_map_io,
	.init_early	= orion5x_init_early,
	.init_irq	= orion5x_init_irq,
	.init_time	= orion5x_timer_init,
	.dma_zone_size  = SZ_256M,
	.restart	= orion5x_restart,
MACHINE_END
