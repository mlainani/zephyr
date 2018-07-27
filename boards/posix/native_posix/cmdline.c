/*
 * Copyright (c) 2018 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <string.h>
#include "cmdline_common.h"
#include "zephyr/types.h"
#include "hw_models_top.h"
#include "timer_model.h"
#include "cmdline.h"
#include "toolchain.h"
#include "board.h"

static int s_argc, test_argc;
static char **s_argv, **test_argv;

static struct args_t args;

static void cmd_stop_at_found(char *argv, int offset)
{
	ARG_UNUSED(offset);
	if (args.stop_at < 0) {
		posix_print_error_and_exit("Error: stop-at must be positive "
					   "(%s)\n", argv);
	}
	hwm_set_end_of_time(args.stop_at*1e6);
}

static void cmd_realtime_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	hwtimer_set_real_time_mode(true);
}

static void cmd_no_realtime_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	hwtimer_set_real_time_mode(false);
}

static void cmd_rtcoffset_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	hwtimer_set_rtc_offset(args.rtc_offset*1e6);
}

static void cmd_rt_drift_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	if (!(args.rt_drift > -1)) {
		posix_print_error_and_exit("The drift needs to be > -1. "
					  "Please use --help for more info\n");
	}
	args.rt_ratio = args.rt_drift + 1;
	hwtimer_set_rt_ratio(args.rt_ratio);
}

static void cmd_rt_ratio_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	if ((args.rt_ratio <= 0)) {
		posix_print_error_and_exit("The ratio needs to be > 0. "
					  "Please use --help for more info\n");
	}
	hwtimer_set_rt_ratio(args.rt_ratio);
}

static void cmd_rtcreset_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	hwtimer_reset_rtc();
}

#if defined(CONFIG_FAKE_ENTROPY_NATIVE_POSIX)
extern void entropy_native_posix_set_seed(unsigned int seed_i);
static void cmd_seed_found(char *argv, int offset)
{
	ARG_UNUSED(argv);
	ARG_UNUSED(offset);
	entropy_native_posix_set_seed(args.seed);
}
#endif

#if defined(CONFIG_BT_USERCHAN)
int bt_dev_index = -1;

static void cmd_bt_dev_found(char *argv, int offset)
{
	if (strncmp(&argv[offset], "hci", 3) || strlen(&argv[offset]) < 4) {
		posix_print_error_and_exit("Error: Invalid Bluetooth device "
					   "name '%s' (should be e.g. hci0)\n",
					   &argv[offset]);
		return;
	}

	bt_dev_index = strtol(&argv[offset + 3], NULL, 10);
}
#endif

/**
 * Handle possible command line arguments.
 *
 * We also store them for later use by possible test applications
 */
void native_handle_cmd_line(int argc, char *argv[])
{
	int i;

	struct args_struct_t args_struct[] = {
		/*
		 * Fields:
		 * manual, mandatory, switch,
		 * option_name, var_name ,type,
		 * destination, callback,
		 * description
		 */
		{false, false, true,
		"rt", "", 'b',
		NULL, cmd_realtime_found,
		"Slow down the execution to the host real time, "
		"or a ratio of it (see --rt-ratio below)"},

		{false, false, true,
		"no-rt", "", 'b',
		NULL, cmd_no_realtime_found,
		"Do NOT slow down the execution to real time, but advance "
		"Zephyr's time as fast as possible and decoupled from the host "
		"time"},

		{false, false, false,
		"rt-drift", "dratio", 'd',
		(void *)&args.rt_drift, cmd_rt_drift_found,
		"Drift of the simulated clock relative to the host real time. "
		"Normally this would be set to a value of a few ppm (e.g. 50e-6"
		") "
		"This option has no effect in non real time mode"
		},

		{false, false, false,
		"rt-ratio", "ratio", 'd',
		(void *)&args.rt_ratio, cmd_rt_ratio_found,
		"Relative speed of the simulated time vs real time. "
		"For ex. set to 2 to have simulated time pass at double the "
		"speed of real time. "
		"Note that both rt-drift & rt-ratio adjust the same clock "
		"speed, and therefore it does not make sense to use them "
		"simultaneously. "
		"This option has no effect in non real time mode"
		},

		{false, false, false,
		"rtc-offset", "time_offset", 'd',
		(void *)&args.rtc_offset, cmd_rtcoffset_found,
		"At boot offset the RTC clock by this amount of seconds"
		},

		{false, false, true,
		"rtc-reset", "", 'b',
		NULL, cmd_rtcreset_found,
		"Start the simulated real time clock at 0. Otherwise it starts "
		"matching the value provided by the host real time clock"},

		{false, false, false,
		 "stop_at", "time", 'd',
		(void *)&args.stop_at, cmd_stop_at_found,
		"In simulated seconds, when to stop automatically"},

#if defined(CONFIG_FAKE_ENTROPY_NATIVE_POSIX)
		{false, false, false,
		  "seed", "r_seed", 'u',
		(void *)&args.seed, cmd_seed_found,
		"A 32-bit integer seed value for the entropy device, such as "
		"97229 (decimal), 0x17BCD (hex), or 0275715 (octal)"},
#endif

#if defined(CONFIG_BT_USERCHAN)
		{ false, true, false,
		  "bt-dev", "hciX", 's',
		  NULL, cmd_bt_dev_found,
		"A local HCI device to be used for Bluetooth (e.g. hci0)" },
#endif

		{true, false, false,
		 "testargs", "arg", 'l',
		(void *)NULL, NULL,
		"Any argument that follows will be ignored by the top level, "
		"and made available for possible tests"},

		ARG_TABLE_ENDMARKER
	};

	s_argv = argv;
	s_argc = argc;

	cmd_args_set_defaults(args_struct);

	for (i = 1; i < argc; i++) {

		if ((cmd_is_option(argv[i], "testargs", 0))) {
			test_argc = argc - i - 1;
			test_argv = &argv[i+1];
			break;
		}

		if (!cmd_parse_one_arg(argv[i], args_struct)) {
			cmd_print_switches_help(args_struct);
			posix_print_error_and_exit("Incorrect option '%s'\n",
						   argv[i]);
		}
	}

#if defined(CONFIG_BT_USERCHAN)
	if (bt_dev_index < 0) {
		posix_print_error_and_exit("Error: Bluetooth device missing. "
					   "Specify one using --bt-dev=hciN\n");
	}
#endif
}

/**
 * The application/test can use this function to inspect all the command line
 * arguments
 */
void native_get_cmd_line_args(int *argc, char ***argv)
{
	*argc = s_argc;
	*argv = s_argv;
}

/**
 * The application/test can use this function to inspect the command line
 * arguments received after --testargs
 */
void native_get_test_cmd_line_args(int *argc, char ***argv)
{
	*argc = test_argc;
	*argv = test_argv;
}
