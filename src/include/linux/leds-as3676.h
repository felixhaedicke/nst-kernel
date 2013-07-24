/*
 * as3676.h - platform data structure for as3676 led controller
 *
 * Copyright (C) 2010 Ulrich Herrmann <ulrich.herrmann@austriamicrosystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */

#ifndef __LINUX_AS3676_H
#define __LINUX_AS3676_H

#define AS3676_MAX_NUM_LEDS  13
#define AS3676_NUM_LEDS  2

struct as3676_platform_led {
	char *name;		/* if NULL the default name is used */
	u8 on_charge_pump;	/* Is this led connected to charge pump or
				   to DCDC/VBAT */
	u32 max_current_uA;	/* This leds maximum current in mA */
};

struct as3676_platform_data {
	struct as3676_platform_led leds[AS3676_NUM_LEDS];	/* order: curr1, curr2, curr6,
								   rgb1, rgb2, rgb3,
								   curr41, curr42, curr43,
								   curr30, curr31, curr32, curr33
								 */
	u32 step_up_frequ; /* Defines the clock frequency of the Step Up DC/DC Converter.
			0 = 1 MHz,  1 = 500 kHz */
	u32 pwronkey_shutdown_msec; /* Time to monitor for PWRON key long-press to shutdown. 0 = don't monitor */
	u8 step_up_vtuning;	/* 0 .. 31 uA on DCDC_FB */
#ifdef AS3678XM
	u8 audio_speed_down;	/* 0..3 resp. 0, 200, 400, 800ms */
	u8 audio_speed_up;	/* 0..7 resp. 0, 50, 100, 150,
				   200,250,400, 800ms */
	u8 audio_agc_ctrl;	/* 0 .. 7: 0==no AGC,
				   7 very aggressive */
	u8 audio_gain;		/* 0..7: -12, -6,  0, 6
				   12, 18, 24, 30 dB */
	u8 audio_source;	/* 0,2,3: 0=curr33, 1=DCDC_FB
				   2=GPIO1,  3=GPIO2 */
#endif /* AS3687XM */
};

#endif /* __LINUX_as3676_H */
