/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * CRTP commander with additional torque packet (SI units):
 *   data[0] = type  = 0x0A
 *   data[1..4]   = thrust [N]
 *   data[5..8]   = tx [Nm]
 *   data[9..12]  = ty [Nm]
 *   data[13..16] = tz [Nm]
 */

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "crtp_commander.h"
#include "cfassert.h"
#include "commander.h"
#include "crtp.h"
#include "debug.h"     // <== needed for DEBUG_PRINT

#define CRTP_COMMANDER_TORQUE 0x0A   // NEW TORQUE PACKET TYPE

static bool isInit;
static void commanderCrtpCB(CRTPPacket* pk);


/* ------------------------------------------------------------------------- */
/*                     TORQUE PACKET STRUCTURE + DECODER                      */
/* ------------------------------------------------------------------------- */

struct torqueSetpointPacket {
    uint8_t type;   // always CRTP_COMMANDER_TORQUE
    float thrust;   // N
    float tx;       // Nm
    float ty;       // Nm
    float tz;       // Nm
} __attribute__((packed));

static void crtpCommanderTorqueDecode(setpoint_t *sp, const CRTPPacket *pk)
{
    const struct torqueSetpointPacket *p =
        (const struct torqueSetpointPacket *)pk->data;

    DEBUG_PRINT("TORQUE PKT: thrust=%f tx=%f ty=%f tz=%f\n",
                p->thrust, p->tx, p->ty, p->tz);

    sp->timestamp = xTaskGetTickCount();

    // SI units from Python
    sp->thrust  = p->thrust;   // N
    sp->torqueX = p->tx;       // Nm
    sp->torqueY = p->ty;       // Nm
    sp->torqueZ = p->tz;       // Nm

    // Disable attitude-based control when using torque packet
    sp->mode.roll  = modeDisable;
    sp->mode.pitch = modeDisable;
    sp->mode.yaw   = modeDisable;
}


/* ------------------------------------------------------------------------- */
/*                          COMMANDER INIT FUNCTION                           */
/* ------------------------------------------------------------------------- */

void crtpCommanderInit(void)
{
  if (isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);

  isInit = true;
}


/* ------------------------------------------------------------------------- */
/*                         META-COMMAND HANDLING                              */
/* ------------------------------------------------------------------------- */

enum metaCommand_e {
  metaNotifySetpointsStop = 0,
  nMetaCommands,
};

struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));

static void notifySetpointsStopDecoder(const void *data, size_t len)
{
  (void)data;
  (void)len;
  commanderRelaxPriority();
}

typedef void (*metaCommandDecoder_t)(const void *, size_t);

static const metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};

enum crtpSetpointGenericChannel {
  SET_SETPOINT_CHANNEL = 0,
  META_COMMAND_CHANNEL = 1,
};


/* ------------------------------------------------------------------------- */
/*                           MAIN CRTP CALLBACK                               */
/* ------------------------------------------------------------------------- */

static void commanderCrtpCB(CRTPPacket* pk)
{
  DEBUG_PRINT("CRTP: port=%d channel=%d size=%d firstByte=%d\n",
              pk->port, pk->channel, pk->size, pk->data[0]);

  static setpoint_t sp;

  /* ---------------------------------------------------------------------- */
  /*                          PRIMARY SETPOINT PORT                         */
  /* ---------------------------------------------------------------------- */
  if (pk->port == CRTP_PORT_SETPOINT && pk->channel == 0) {

      // Our torque packet: data[0] = 0x0A, total size = 1 + 4*4 = 17
      if (pk->data[0] == CRTP_COMMANDER_TORQUE &&
          pk->size == sizeof(struct torqueSetpointPacket))
      {
          crtpCommanderTorqueDecode(&sp, pk);
      }
      else
      {
          // Legacy RPYT decoding
          crtpCommanderRpytDecodeSetpoint(&sp, pk);
      }

      commanderSetSetpoint(&sp, COMMANDER_PRIORITY_CRTP);
      return;
  }

  /* ---------------------------------------------------------------------- */
  /*                    CRAZYSWARM GENERIC SETPOINT PORT                    */
  /* ---------------------------------------------------------------------- */
  if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {

    switch (pk->channel) {

      case SET_SETPOINT_CHANNEL:
        crtpCommanderGenericDecodeSetpoint(&sp, pk);
        commanderSetSetpoint(&sp, COMMANDER_PRIORITY_CRTP);
        break;

      case META_COMMAND_CHANNEL:
      {
        uint8_t meta = pk->data[0];
        if (meta < nMetaCommands && metaCommandDecoders[meta]) {
          metaCommandDecoders[meta](pk->data + 1, pk->size - 1);
        }
      }
      break;

      default:
        // ignore
        break;
    }
  }
}
