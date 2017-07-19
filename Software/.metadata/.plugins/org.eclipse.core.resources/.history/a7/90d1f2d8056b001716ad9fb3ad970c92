/****************************************************************************
 *  Copyright (C) 2008-2012 by Michael Fischer.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the author nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 *  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 *  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGE.
 *
 ****************************************************************************
 *  History:
 *
 *  07.11.2008  mifi  First Version, based on FatFs example.
 *  11.02.2012  mifi  Tested with EIR.
 *  23.08.2012  mifi  Tested with an Altera DE1.
 ****************************************************************************/
#define __MAIN_C__

/*=========================================================================*/
/*  Includes                                                               */
/*=========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <system.h>
#include <sys/alt_alarm.h>
#include <io.h>

#include "fatfs.h"
#include "diskio.h"

#include "ff.h"
#include "monitor.h"
#include "uart.h"

#include "alt_types.h"

#include <altera_up_avalon_audio.h>
#include <altera_up_avalon_audio_and_video_config.h>

#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"

#include "helpers.c"

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/
uint8_t left_buffer[8192], right_buffer[8192];
#define PLAY 1
#define PAUSE 2
#define SEEK_FORWARD 3
#define SEEK_BACKWARD 4
#define SEARCH_FORWARD 5
#define SEARCH_BACKWARD 6

/*=========================================================================*/
/*  DEFINE: Macros                                                         */
/*=========================================================================*/

#define PSTR(_a)  _a

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/
// Playing variables
uint32_t ofs = 0;   // file pointer
int read_amt = 512;  // number of bytes to read at a time
int num_bytes;		// number of bytes left to play

// File interaction information
int file_index = 0;
char* file_name[20];
long file_size[20];

// State machine
int *run_state;

// LCD
FILE *lcd;

static alt_alarm alarm;
static unsigned long Systick = 0;
static volatile unsigned short Timer; /* 1000Hz increment timer */

unsigned int l_buf;
unsigned int r_buf;

int fifospace;
char *ptr, *ptr2;
long p1, p2, p3;
uint8_t res, b1, drv = 0;
uint16_t w1;
uint8_t Buff[8192] __attribute__ ((aligned(4))); /* Working buffer */
uint32_t s1, s2, cnt, blen = sizeof(Buff);
static const uint8_t ft[] = { 0, 12, 16, 32 };
uint32_t sect = 0, blk[2];
FATFS *fs; /* Pointer to file system object */

alt_up_audio_dev * audio_dev;

/*=========================================================================*/
/*  DEFINE: Definition of all local Procedures                             */
/*=========================================================================*/

/***************************************************************************/
/*  TimerFunction                                                          */
/*                                                                         */
/*  This timer function will provide a 10ms timer and                      */
/*  call ffs_DiskIOTimerproc.                                              */
/*                                                                         */
/*  In    : none                                                           */
/*  Out   : none                                                           */
/*  Return: none                                                           */
/***************************************************************************/
static alt_u32 TimerFunction(void *context) {
	static unsigned short wTimer10ms = 0;

	(void) context;

	Systick++;
	wTimer10ms++;
	Timer++; /* Performance counter for this module */

	if (wTimer10ms == 10) {
		wTimer10ms = 0;
		ffs_DiskIOTimerproc(); /* Drive timer procedure of low level disk I/O module */
	}

	return (1);
} /* TimerFunction */

/***************************************************************************/
/*  IoInit                                                                 */
/*                                                                         */
/*  Init the hardware like GPIO, UART, and more...                         */
/*                                                                         */
/*  In    : none                                                           */
/*  Out   : none                                                           */
/*  Return: none                                                           */
/***************************************************************************/
static void IoInit(void) {
	uart0_init(115200);

	/* Init diskio interface */
	ffs_DiskIOInit();

	//SetHighSpeed();

	/* Init timer system */
	alt_alarm_start(&alarm, 1, &TimerFunction, NULL);

} /* IoInit */

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

uint32_t acc_size; /* Work register for fs command */
uint16_t acc_files, acc_dirs;
FILINFO Finfo;
#if _USE_LFN
char Lfname[512];
#endif

char Line[256]; /* Console input buffer */

FATFS Fatfs[_VOLUMES]; /* File system object for each logical drive */
FIL File1, File2; /* File objects */
DIR Dir; /* Directory object */

static void put_rc(FRESULT rc) {
	const char *str =
			"OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
					"INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
					"INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
					"LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
	FRESULT i;

	for (i = 0; i != rc && *str; i++) {
		while (*str++)
			;
	}
	xprintf("rc=%u FR_%s\n", (uint32_t) rc, str);
}

void disk_init(int disk_num) {
	//if (!xatoi(&ptr, &p1))
	//				break;
	//			xprintf("rc=%d\n", (uint16_t) disk_initialize((uint8_t ) p1));
	//			break;

	xprintf("rc=%d\n", (uint16_t) disk_initialize((uint8_t ) disk_num));
}

void filesys_init(int file_num) {
	//if (!xatoi(&ptr, &p1))
	//					break;
	put_rc(f_mount((uint8_t) file_num, &Fatfs[file_num]));
}

void file_open(int mode, char* file_name) {
	//if (!xatoi(&ptr, &p1))
	//	break;
	//while (*ptr == ' ')
	//	ptr++;
	put_rc( f_open(&File1, file_name, (uint8_t) mode) );

}

int is_wav( char* filename ) {
	if (strstr(filename, ".WAV")) {
		return 1;
	}
	else {
		return 0;
	}
}

void play_file( ) {

	int j;
	if ((uint32_t) num_bytes >= read_amt) {
		cnt = read_amt;
		num_bytes -= read_amt;
	} else {
		cnt = num_bytes;
		num_bytes = 0;
	}
	res = f_read(&File1, Buff, cnt, &cnt);
	if (res != FR_OK) {
		put_rc(res);
		return;
	}
	if (!cnt)
		return;
	ofs += read_amt;

	for (j = 0; j < read_amt; j += 4) {
		l_buf = 0;

		l_buf = Buff[j + 1];
		l_buf = Buff[j] | l_buf << 8;
		r_buf = Buff[j + 3];
		r_buf = Buff[j + 2] | r_buf << 8;

		while (alt_up_audio_write_fifo_space(audio_dev, ALT_UP_AUDIO_RIGHT)
				< 0) // check if space is available
		{
		}
		alt_up_audio_write_fifo(audio_dev, &(r_buf), 1, ALT_UP_AUDIO_RIGHT);
		alt_up_audio_write_fifo(audio_dev, &(l_buf), 1, ALT_UP_AUDIO_LEFT);
	}

}

// Prints the track index and name to the LCD
void lcd_print( int file_index, char* file_name ) {
	fprintf(lcd, "%d: %s", file_index, file_name);
}


void button_ISR ( void* context, alt_u32 id ) {
	int button_value = IORD( BUTTON_PIO_BASE, 0 );

	//printf("%d\n", button_value); fflush(stdout);

	switch (button_value) {
		// leftmost down: 			reverse
		case 7:
			printf("pressed reverse\n");
			if ( *run_state == PLAY ) {
				*run_state = SEEK_BACKWARD;
			}
			else if ( *run_state == PAUSE ) {
				*run_state = SEARCH_BACKWARD;
			}
			break;
		// second from left down:	stop
		case 11:
			printf("pressed stop\n");
			*run_state = PAUSE;
			File1.fptr = 0;
			num_bytes = file_size[file_index];
			break;
		// second from right down:	play/pause
		case 13:
			printf("pressed play/pause\n");
			if (*run_state == PAUSE) {
				*run_state = PLAY;
			}
			else {
				*run_state = PAUSE;
			}
			break;
		// rightmost down:			forward
		case 14:
			printf("pressed forward\n");
			if ( *run_state == PLAY ) {
				*run_state = SEEK_FORWARD;
			}
			else if ( *run_state == PAUSE ) {
				*run_state = SEARCH_FORWARD;
			}
			break;
		// any up
		case 15:
			if ( *run_state == SEEK_FORWARD || *run_state == SEEK_BACKWARD ) {
				*run_state = PLAY;
				printf("released button%d\n", *run_state);
			}
			break;
	}

	// Reset the ISR
	IOWR(BUTTON_PIO_BASE, 3, 0x0);
}

/***************************************************************************/
/*  main                                                                   */
/***************************************************************************/
int main(void) {
	// LCD screen reference for writing track information
	lcd = fopen("/dev/lcd_display", "w");

	/* used for audio record/playback */
	// open the Audio port
	audio_dev = alt_up_audio_open_dev("/dev/Audio");
	if (audio_dev == NULL)
		alt_printf("Error: could not open audio device \n");
	else
		alt_printf("Opened audio device \n");

	IoInit();

	// Initialize ISRs
	IOWR(BUTTON_PIO_BASE, 2, 0xF);
	alt_irq_register( BUTTON_PIO_IRQ, (void *)0, button_ISR );
	alt_irq_enable(BUTTON_PIO_IRQ);

	xputs(PSTR("FatFs module test monitor\n"));
	xputs(_USE_LFN ? "LFN Enabled" : "LFN Disabled");
	xprintf(", Code page: %u\n", _CODE_PAGE);

#if _USE_LFN
	Finfo.lfname = Lfname;
	Finfo.lfsize = sizeof(Lfname);
#endif

	disk_init(0);
	filesys_init(0);

	// Initializing the file list
	res = f_opendir(&Dir, ptr);
	if (res) {
		put_rc(res);
	}
	p1 = s1 = s2 = 0;

	for (;;) {
		res = f_readdir(&Dir, &Finfo);
		if ((res != FR_OK) || !Finfo.fname[0])
			break;
		if (Finfo.fattrib & AM_DIR) {
			s2++;
		} else {
			if( is_wav(&(Finfo.fname[0])) ) {
				s1++;
			}
			p1 += Finfo.fsize;
		}

		if ( is_wav(&(Finfo.fname[0])) ){
			file_name[s1] = &(Finfo.fname[0]);
			file_size[s1] = Finfo.fsize;
			printf("%d %d %s\n", (int)s1, (int)file_size[s1], file_name[s1]);
		}
	}

	// File list initialized, beginning main loop

	*run_state = PLAY;
	file_index = 1;
	//lcd_print(file_index, file_name[file_index]);

	for(file_index = 1; file_index <= 14; file_index++) {
		printf("%d %d %s\n", file_index, (int)file_size[file_index], file_name[file_index]);
		file_open(1, file_name[file_index]);
	}

	num_bytes = file_size[file_index];

	file_open(1, "LONG.WAV");
	ofs = File1.fptr;
	num_bytes = 74926244;

	while (1) {
		//printf("%d:%d ", *run_state, file_index);
		switch (*run_state){
			case SEEK_FORWARD:

				break;
			case PLAY:
				if(num_bytes) {
					play_file();
				}
				break;
			case SEEK_BACKWARD:

				break;
			case PAUSE:
				//printf("paused%d ", file_index);
				break;
			case SEARCH_FORWARD:
				file_index++;
				printf("searching forward, track %d: %s\n ", file_index, file_name[file_index]);
				file_open(1, file_name[file_index]);
				ofs = File1.fptr;
				num_bytes = file_size[file_index];
				lcd_print(file_index, file_name[file_index]);
				*run_state = PAUSE;
				break;
			case SEARCH_BACKWARD:
				if(file_index > 1) {
					file_index--;
				}
				printf("searching backward, track %d: %s\n ", file_index, file_name[file_index]);
				file_open(1, file_name[file_index]);
				ofs = File1.fptr;
				num_bytes = file_size[file_index];
				lcd_print(file_index, file_name[file_index]);
				*run_state = PAUSE;
				break;
		}
	}

	printf("Exiting main loop\n");
	return 0;
}
