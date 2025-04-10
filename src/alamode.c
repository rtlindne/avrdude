

	/*
	* avrdude - A Downloader/Uploader for AVR device programmers
	* Copyright (C) 2009 Lars Immisch
	*
	* This program is free software; you can redistribute it and/or modify
	* it under the terms of the GNU General Public License as published by
	* the Free Software Foundation; either version 2 of the License, or
	* (at your option) any later version.
	*
	* This program is distributed in the hope that it will be useful,
	* but WITHOUT ANY WARRANTY; without even the implied warranty of
	* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	* GNU General Public License for more details.
	*
	* You should have received a copy of the GNU General Public License
	* along with this program; if not, write to the Free Software
	* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
	*/

	/*
	* avrdude interface for alamode-Arduino programmer
	*
	* The Alamode programmer is mostly a STK500v1, just the signature bytes
	* are read differently. We are replacing DTR RTS with a GPIO Twiddle
	*/

#include <ac_cfg.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include "avrdude.h"
#include "libavrdude.h"
#include "stk500_private.h"
#include "stk500.h"

	// for GPIO code
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
	// Access from ARM Running Linux

#define BCM2836_PERI_BASE        0x3F000000
#define BCM2835_PERI_BASE        0x20000000

	static volatile uint32_t gpio_base;

#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

void alamode_reset();
static void setup_io();

static int alamode_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
	int attempts;
	int rv = 0;
	bool help = 0;
	my.autoreset = true;
	for(LNODEID ln = lfirst(extparms); ln; ln = lnext(ln)) {
		const char *extended_param = ldata(ln);

		if(sscanf(extended_param, "attempts=%i", &attempts) == 1) {
			my.retry_attempts = attempts;
			pmsg_info("setting number of retry attempts to %d\n", attempts);
			continue;
		}

		if(str_eq(extended_param, "noautoreset")) {
			pmsg_info("no autoreset");
			my.autoreset = false;
			continue;
		}

		if(str_eq(extended_param, "help")) {
			help = true;
			rv = LIBAVRDUDE_EXIT;
		}

		if(!help) {
			pmsg_error("invalid extended parameter -x %s\n", extended_param);
			rv = -1;
		}
		msg_error("%s -c %s extended options:\n", progname, pgmid);
		msg_error("  -x attempts=<n> Specify the number <n> of connection retry attempts\n");
		msg_error("  -x noautoreset  Don't toggle RTS/DTR lines on port open to prevent a hardware reset\n");
		msg_error("  -x help         Show this help menu and exit\n");
		return rv;
	}
	return rv;
}

// Read signature bytes - alamode version
static int alamode_read_sig_bytes(const PROGRAMMER * pgm, const AVRPART *p, const AVRMEM * m)
{
	unsigned char buf[32];

	// Signature byte reads are always 3 bytes

	if (m->size < 3) {
		pmsg_error("%s: memsize too small for sig byte read", progname);
		return -1;
	}

	buf[0] = Cmnd_STK_READ_SIGN;
	buf[1] = Sync_CRC_EOP;

	serial_send(&pgm->fd, buf, 2);

	if (serial_recv(&pgm->fd, buf, 5) < 0)
		return -1;
	if (buf[0] == Resp_STK_NOSYNC) {
		pmsg_error("%s: stk500_cmd(): programmer is out of sync\n",
			progname);
		return -1;
	} else if (buf[0] != Resp_STK_INSYNC) {
		msg_error("\n");
		pmsg_error(
			"%s: alamode_read_sig_bytes(): (a) protocol error, "
			"expect=0x%02x, resp=0x%02x\n",
			progname, Resp_STK_INSYNC, buf[0]);
		return -2;
	}
	if (buf[4] != Resp_STK_OK) {
		msg_error("\n");
		pmsg_error(
			"%s: alamode_read_sig_bytes(): (a) protocol error, "
			"expect=0x%02x, resp=0x%02x\n",
			progname, Resp_STK_OK, buf[4]);
		return -3;
	}

	m->buf[0] = buf[1];
	m->buf[1] = buf[2];
	m->buf[2] = buf[3];

	return 3;
}

static int alamode_open(PROGRAMMER * pgm, const char * port)
{
	union pinfo pinfo;

	pgm->port = port;
	pinfo.serialinfo.baud = pgm->baudrate? pgm->baudrate: 115200;
	pinfo.serialinfo.cflags = SERIAL_8N1;
	if(serial_open(port, pinfo, &pgm->fd) == -1) {
		return -1;
	}
	//alamode always needs reset for some reason its false even though params dont set it false
	if(true || my.autoreset) 
	{
		/* Clear GPIO18  to unload the RESET capacitor 
		* (for example in Alamode) 
		*/
		alamode_reset();
	}

	// Drain any extraneous input
	stk500_drain(pgm, 0);

	if (stk500_getsync(pgm) < 0)
		return -1;

	return 0;
}

static void alamode_close(PROGRAMMER * pgm)
{
	serial_close(&pgm->fd);
	pgm->fd.ifd = -1;
}

const char alamode_desc[] = "Alamode programmer for bootloading";

void alamode_initpgm(PROGRAMMER * pgm)
{
	/* This is mostly a STK500; just the signature is read
	differently than on real STK500v1 
	and the DTR signal is set when opening the serial port
	for the Auto-Reset feature */
	setup_io();
	stk500_initpgm(pgm);

	strcpy(pgm->type, "Alamode");
	pgm->read_sig_bytes = alamode_read_sig_bytes;
	pgm->open = alamode_open;
	pgm->close = alamode_close;
	pgm->parseextparams = alamode_parseextparms;

	cx->avr_disableffopt = 1;     // Disable trailing 0xff removal
}

// GPIO functions
//
// determine Raspberry Pi revision
//
uint32_t gpioHardwareRevision(void)
{
	FILE * filp;
	char buf[512];
	uint32_t piPeriphBase = 0x20000000;
	uint32_t piModel = 1;

	piModel = 0;

	filp = fopen ("/proc/cpuinfo", "r");

	if (filp != NULL)
	{
		while (fgets(buf, sizeof(buf), filp) != NULL)
		{
			if (piModel == 0)
			{
				if (strstr (buf, "ARMv6") != NULL)
				{
					piModel = 1;
					piPeriphBase = 0x20200000;
				}
				else if (strstr (buf, "ARMv7") != NULL)
				{
					piModel = 2;
					piPeriphBase = 0x3F200000;
				}
				else if (strstr (buf, "ARMv8") != NULL)
				{
					piModel = 4;
					piPeriphBase = 0xFE200000;
				}
				else if (strstr (buf, "Raspberry Pi 5") != NULL)
				{
					piModel = 5;
					piPeriphBase = 0x402D0000;
				}
				
			}

		}

		fclose(filp);
	}
	return piPeriphBase ;
}

// Set up a memory regions to access GPIO
//
static void setup_io()
{
	uint PAGE_SIZE = (4*1024);
	if(sysconf(_SC_PAGESIZE) > 0)
	{
		PAGE_SIZE = sysconf(_SC_PAGESIZE);
	}
	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		pmsg_error("can't open /dev/mem \n");
		exit (-1);
	}

	/* mmap GPIO */
	// determine Pi revision
	
	gpio_base = gpioHardwareRevision(); 
	// Allocate MAP block
	if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
		pmsg_error("allocation error \n");
		exit (-1);
	}

	// Make sure pointer is on 4K boundary
	if ((unsigned long)gpio_mem % PAGE_SIZE)
		gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

	// Now map it
	gpio_map = mmap(
		(caddr_t)gpio_mem,
		BLOCK_SIZE,
		PROT_READ|PROT_WRITE,
		MAP_SHARED|MAP_FIXED,
		mem_fd,
		gpio_base
		);

	if ((long)gpio_map < 0) {
		pmsg_error("mmap error %s(%d) offset 0x%x page_size %d\n", 
			strerror(errno), errno, gpio_base, PAGE_SIZE);
		exit (-1);
	}

	// Always use volatile pointer!
	gpio = (volatile unsigned *)gpio_map;


} // End setup_io

void alamode_reset(){
	// must use INP_GPIO before OUT_GPIO
	INP_GPIO(18);
	OUT_GPIO(18);

	// set GPIO 18 Low
	GPIO_CLR = 1 << 18;
	usleep(50*1000);
	// set GPIO 18 High
	GPIO_SET = 1 << 18;
	usleep(50*1000);
}

