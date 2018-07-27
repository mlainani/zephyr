/*
 * Copyright (c) 2018 Madani Lainani.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa = device_get_binding(CONFIG_PINMUX_SAM0_A_LABEL);
	struct device *muxb = device_get_binding(CONFIG_PINMUX_SAM0_B_LABEL);
#if CONFIG_EIC_SAM0
	struct device *mux = NULL;
#endif

	ARG_UNUSED(dev);

#if CONFIG_I2C_SAM0_SERCOM0_BASE_ADDRESS
	/* I2C SERCOM0 on SDA=PA08/PAD[0], SCL=PA09/PAD[1] */
	pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 9, PINMUX_FUNC_C);
#endif

#if CONFIG_SPI_SAM0_SERCOM1_BASE_ADDRESS
	/* SPI SERCOM1 on MOSI=PA16/PAD[0], SCK=PA17/PAD[1], MISO=PA19/PAD[3] */
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 19, PINMUX_FUNC_C);
#endif

#if CONFIG_UART_SAM0_SERCOM5_BASE_ADDRESS
	/* SERCOM5 on TX=PB22, RX=PB23 */
	pinmux_pin_set(muxb, 22, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 23, PINMUX_FUNC_D);
#endif

#if CONFIG_UART_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_UART_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if CONFIG_SPI_SAM0_SERCOM0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_SPI_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if CONFIG_I2C_SAM0_SERCOM1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_I2C_SAM0_SERCOM2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_I2C_SAM0_SERCOM3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_I2C_SAM0_SERCOM4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if CONFIG_I2C_SAM0_SERCOM5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif

#if CONFIG_EIC_SAM0
	
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_4
	/* D6 - PA20 - EXTINT[4] */
	mux = device_get_binding(CONFIG_GPIO_SAM0_PORTA_LABEL);
	if (!mux) {
		printk("Crap!");
		return -1;
	}
	/*
	 * No point in testing return code as SoC API implementation for the
	 * function always return 0.
	 */
	pinmux_pin_set(mux, CONFIG_GPIO_SAM0_PORTA_EXTINT_4_PIN,
		       PINMUX_FUNC_A);
#endif

#if CONFIG_GPIO_SAM0_PORTA_EXTINT_0 || CONFIG_GPIO_SAM0_PORTB_EXTINT_0
#error Pin muxing for external interrupt 0 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_1 || CONFIG_GPIO_SAM0_PORTB_EXTINT_1
#error Pin muxing for external interrupt 1 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_2 || CONFIG_GPIO_SAM0_PORTB_EXTINT_2
#error Pin muxing for external interrupt 2 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_3 || CONFIG_GPIO_SAM0_PORTB_EXTINT_3
#error Pin muxing for external interrupt 3 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTB_EXTINT_4
#error Pin muxing for external interrupt 4 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_5 || CONFIG_GPIO_SAM0_PORTB_EXTINT_5
#error Pin muxing for external interrupt 5 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_6 || CONFIG_GPIO_SAM0_PORTB_EXTINT_6
#error Pin muxing for external interrupt 6 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_7 || CONFIG_GPIO_SAM0_PORTB_EXTINT_7
#error Pin muxing for external interrupt 7 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_8 || CONFIG_GPIO_SAM0_PORTB_EXTINT_8
#error Pin muxing for external interrupt 8 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_9 || CONFIG_GPIO_SAM0_PORTB_EXTINT_9
#error Pin muxing for external interrupt 9 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_10 || CONFIG_GPIO_SAM0_PORTB_EXTINT_10
#error Pin muxing for external interrupt 10 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_11 || CONFIG_GPIO_SAM0_PORTB_EXTINT_11
#error Pin muxing for external interrupt 11 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_12 || CONFIG_GPIO_SAM0_PORTB_EXTINT_12
#error Pin muxing for external interrupt 12 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_13 || CONFIG_GPIO_SAM0_PORTB_EXTINT_13
#error Pin muxing for external interrupt 13 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_14 || CONFIG_GPIO_SAM0_PORTB_EXTINT_14
#error Pin muxing for external interrupt 14 is not configured
#endif
#if CONFIG_GPIO_SAM0_PORTA_EXTINT_15 || CONFIG_GPIO_SAM0_PORTB_EXTINT_15
#error Pin muxing for external interrupt 15 is not configured
#endif

#endif /* CONFIG_EIC_SAM0 */

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
