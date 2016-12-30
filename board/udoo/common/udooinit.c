/*
 * Copyright (C) UDOO Team
 *
 * Author: Christian Ege <k4230r6@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include "udooinit.h"
#include <common.h>

/**
 * helper function to set the set fdt_file name
 */
int udooinit_setenv(const char* modelfdt)
{
	char* video_part = "-hdmi";
	char* video = getenv("video_output");

	if (video) {
		video = trim(video);
		if (strcmp(video, "lvds7") == 0) {
			video_part = "-lvds7";
		} else if (strcmp(video, "lvds15") == 0) {
			video_part = "-lvds15";
		}
	}

	char* actual_fdt = getenv("fdt_file");
	if (actual_fdt) {
		actual_fdt = trim(actual_fdt);
		if (strcmp(actual_fdt, "autodetect") != 0) {
			// if fdt_file is already set, do not overwrite it!
			return 0;
		}
	}

	char fdt_file[100];
	char* customdtb = getenv("use_custom_dtb");
	sprintf(fdt_file, "/boot/%s%s.dtb", modelfdt, video_part);
	if (customdtb) {
		customdtb = trim(customdtb);
		if (strcmp(customdtb, "true") == 0 || strcmp(customdtb, "yes") == 0 || strcmp(customdtb, "enabled") == 0) {
			char* dir_part = "dts-overlay";
			sprintf(fdt_file, "/boot/%s/%s%s.dtb", dir_part, modelfdt, video_part);
		}
	}

	printf("Device Tree: %s\n", fdt_file);
	setenv("fdt_file", fdt_file);

	return 0;
}

