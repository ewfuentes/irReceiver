/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "ch.h"
#include "stdlib.h"
/*
 * Thread 1.
 */
THD_WORKING_AREA(waThread1, 128);
THD_FUNCTION(Thread1, arg) {

  (void)arg;

  while (true) {
    palSetLine(LINE_LED_BLUE);
    chThdSleepMilliseconds(200);
    chThdSleepMilliseconds(200);
    chThdSleepMilliseconds(100);
    palClearLine(LINE_LED_BLUE);
    chThdSleepMilliseconds(200);
    chThdSleepMilliseconds(200);
    chThdSleepMilliseconds(100);
  }
}

/*
 * Thread 2.
 */
THD_WORKING_AREA(waThread2, 128);
THD_FUNCTION(Thread2, arg) {

  (void)arg;

  while (true) {
    palSetLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(250);
    palClearLine(LINE_LED_GREEN);
    chThdSleepMilliseconds(250);
  }
}

typedef enum {
	irButton_0,
	irButton_1,
	irButton_2,
	irButton_3,
	irButton_4,
	irButton_5,
	irButton_6,
	irButton_7,
	irButton_8,
	irButton_9,
	irButton_volUp,
	irButton_volDown,
	irButton_mute,
	irButton_chUp,
	irButton_chDown,
	irButton_power,
	irButton_last,
	irButton_lang,
	irButton_enter,
	irButton_info,
	irButton_invalid = 0xFF
} eIrButton;

static EXTConfig extcfg;	
static virtual_timer_t rxTimeout;

typedef struct {
	size_t numSamples;
	systime_t edgeTimes[32];
} sIrPacket;

msg_t mbData[8];
static MAILBOX_DECL(packetMailbox, mbData, 8);
sIrPacket packets[8];
uint8_t currIdx = 0;

void timeoutCallback(void *arg) {
	chSysLockFromISR();
	chMBPostI(&packetMailbox, (msg_t)arg);
	chSysUnlockFromISR();
}

void extCallback(EXTDriver *extp, expchannel_t ch) {
	(void)extp;
	(void)ch;
	systime_t t = chVTGetSystemTimeX();
	chSysLockFromISR();
	if (!chVTIsArmedI(&rxTimeout)) {
		currIdx = (currIdx + 1) & 0x07;
		packets[currIdx].numSamples = 0;
		chVTSetI(&rxTimeout, MS2ST(50), timeoutCallback, &packets[currIdx]);
	}
	size_t numSamples = packets[currIdx].numSamples;
	packets[currIdx].edgeTimes[numSamples] = t;
	packets[currIdx].numSamples++;
	chSysUnlockFromISR();
}

static eIrButton processPacket(sIrPacket *p) {
	// Make sure we found the correct number of transitions
	if (p->numSamples != 17) {
		__asm__("bkpt #1");
		return irButton_invalid;
	}
	
	int32_t minSpace = p->edgeTimes[2];
	int32_t maxSpace = p->edgeTimes[3];
	int32_t spacing = (maxSpace - minSpace) / 15;
	int32_t halfSpace1 = p->edgeTimes[4];
	int32_t halfSpace2 = p->edgeTimes[5];
	int32_t oneBelowHalfSpace = p->edgeTimes[6];
	int32_t oneBelowFullSpace = p->edgeTimes[7];
	
	if (abs(maxSpace - halfSpace1 - halfSpace2) > spacing ||
		abs(oneBelowFullSpace - oneBelowHalfSpace - halfSpace2) > spacing) {
		__asm__("bkpt #1");
		return irButton_invalid;
	}

	__asm__("bkpt #0");

	for (uint8_t i = 0; i < p->numSamples; i++) {
		int32_t remainder = p->edgeTimes[i] % spacing;
		if (remainder > spacing / 2) {
			remainder -= spacing;	
		}

		if (abs(remainder) > spacing * 4 / 10) {
			__asm__("bkpt #1");
			return irButton_invalid;
		}

		p->edgeTimes[i] = p->edgeTimes[i] / spacing + remainder < 0 ? 1 : 0;
	}
	__asm__("bkpt #0");

	return irButton_invalid; 
}

THD_WORKING_AREA(waThread3, 128);
THD_FUNCTION(Thread3, arg) {
	(void) arg;
	chVTObjectInit(&rxTimeout);
	EXTChannelConfig chCfg = {EXT_CH_MODE_FALLING_EDGE | EXT_MODE_GPIOB, extCallback};
	extSetChannelMode(&EXTD1, PAL_PAD(LINE_IR_RECEIVER), &chCfg);
	extChannelEnable(&EXTD1, PAL_PAD(LINE_IR_RECEIVER));
	msg_t msg;
	sIrPacket *p;
	eIrButton b;
	while (true) {
    	chMBFetch(&packetMailbox, &msg, TIME_INFINITE);
		p = (sIrPacket *)msg;
		p->numSamples--;
		for (uint8_t i = 0; i < p->numSamples; i++) {
			p->edgeTimes[i] = p->edgeTimes[i+1] - p->edgeTimes[i];
		}
	
		b = processPacket(p);	
	}
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

	extStart(&EXTD1, &extcfg);

  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);
  /* This is now the idle thread loop, you may perform here a low priority
     task but you must never try to sleep or wait in this loop. Note that
     this tasks runs at the lowest priority level so any instruction added
     here will be executed after all other tasks have been started.*/
  while (true) {
		chThdSleepMilliseconds(100);
  }
}
