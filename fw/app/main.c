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

#define REPEAT_FLAG 0x80

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
    currIdx++;
    if (currIdx > 7) {
        currIdx = 0;
    }
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

static msg_t determineLevels(sIrPacket *p) {
    // Make sure we found the correct number of transitions
    if (p->numSamples != 17) {
        return MSG_RESET;
    }

    // Shift up the samples by three bits to get better resolution
    for (uint8_t i = 0; i < p->numSamples; i++) {
        p->edgeTimes[i] <<=3;
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
        return MSG_RESET;
    }

    for (uint8_t i = 0; i < p->numSamples; i++) {
        int32_t remainder = p->edgeTimes[i] % spacing;
        if (remainder > spacing / 2) {
            remainder -= spacing;
        }

        if (abs(remainder) > spacing * 4 / 10 &&
                p->edgeTimes[i] < 2 * maxSpace) {
            return MSG_RESET;
        }

        p->edgeTimes[i] = p->edgeTimes[i] / spacing + (remainder < 0 ? 1 : 0);
        p->edgeTimes[i] -= 7;
    }
    return MSG_OK;
}

static eIrButton decodeButton(sIrPacket *p) {
    // Sanity check, there should be 17 elements and the last 8 should sum to
    // a multiple of 16
    systime_t *buttonLevels = &p->edgeTimes[9];
    if (p->numSamples != 17) {
        return irButton_invalid;
    }
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum +=buttonLevels[i];
    }
    if ((sum & 0xF) != 0) {
        return irButton_invalid;
    }

    systime_t val = buttonLevels[5];
    eIrButton ret;
    if (buttonLevels[4] == 0) {
        // The enums are arranged such that if buttonLevels[4] is zero,
        // the value of val is the same as the enum, so we can just return it
        ret = val;
    } else {
        switch(val) {
        case 1:
            ret = irButton_last;
            break;
        case 2:
            ret = irButton_lang;
            break;
        case 5:
            ret = irButton_enter;
            break;
        case 6:
            ret = irButton_info;
            break;
        }
    }
    return ret | (buttonLevels[2] == 8 ? REPEAT_FLAG : 0);
}

static eIrButton processPacket(sIrPacket *p) {
    if (determineLevels(p)) {
        return irButton_invalid;
    }
    return decodeButton(p);
}

eIrButton unlockSequence[] = {
    irButton_1,
    irButton_5,
    irButton_9,
    irButton_enter
};

typedef struct {
    uint8_t currState;
} sStateMachineContext;

static uint8_t feedStateMachine(sStateMachineContext *ctx, eIrButton b) {
    b = b & 0x7F;
    if (b == unlockSequence[ctx->currState]) {
        ctx->currState++;
        if (ctx->currState >= sizeof(unlockSequence) / sizeof(eIrButton)) {
            ctx->currState = 0;
            return 1;
        }
    } else if (b == irButton_invalid) {
        
    } else if (ctx->currState > 0 && 
               b == unlockSequence[ctx->currState-1]){
        
    } else {
        ctx->currState = 0;
    }   
    return 0;
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
    sStateMachineContext ctx = {0};
    while (true) {
        chMBFetch(&packetMailbox, &msg, TIME_INFINITE);
        p = (sIrPacket *)msg;
        p->numSamples--;
        for (uint8_t i = 0; i < p->numSamples; i++) {
            p->edgeTimes[i] = p->edgeTimes[i+1] - p->edgeTimes[i];
        }
        b = processPacket(p);
        if (feedStateMachine(&ctx, b)) {
            for (uint8_t i = 0; i < 4; i++) {
                palSetLine(LINE_LED_BLUE);
                chThdSleepMilliseconds(100);
                palClearLine(LINE_LED_BLUE);
                chThdSleepMilliseconds(100);
            }
        }
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
