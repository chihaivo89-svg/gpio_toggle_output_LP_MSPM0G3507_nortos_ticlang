#ifndef HEADING_TUNE_H
#define HEADING_TUNE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Temporary UART command frames all start with "@H," and end with newline.
 * This prefix keeps the framework isolated from the existing one-byte rear
 * wheel diagnostic commands.  Examples:
 *   @H,STATUS
 *   @H,KP,0.10
 *   @H,KD,0.02
 *   @H,DB,1.0
 *   @H,OFFSET,1
 *   @H,TARGET,18
 *   @H,DURATION,5000
 *   @H,LIVE,1
 *   @H,ARM       (optional; KEY1 can also start a default heading test)
 *   @H,RUN       (remote rack test)
 *   @H,STOP
 *   @H,DUMP
 */

void HeadingTune_Init(void);

/* Called from the existing diagnostic UART RX ISR. */
bool HeadingTune_HandleRxByte(uint8_t byte);

/* Parses frames and transmits non-real-time status/dump data in main context. */
void HeadingTune_Process(void);

bool HeadingTune_TakeArmRequest(void);
bool HeadingTune_TakeRemoteRunRequest(void);
bool HeadingTune_TakeStopRequest(void);

void HeadingTune_Arm(void);
void HeadingTune_Disarm(void);
bool HeadingTune_IsArmed(void);
bool HeadingTune_IsActive(void);
int32_t HeadingTune_GetTarget(void);

/* Called only after speed control and heading control both start successfully. */
bool HeadingTune_BeginRun(bool remoteStart);
void HeadingTune_Abort(void);

/* Called from the 20 ms speed-control scheduling slot after the speed update. */
void HeadingTune_Record20ms(int32_t leftActual, int32_t rightActual);

void HeadingTune_ReportStartFailure(const char *reason);

#endif /* HEADING_TUNE_H */
