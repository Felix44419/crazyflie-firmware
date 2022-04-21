/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2020, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* app_channel.c: App realtime communication channel with the ground */

#include "app_channel.h"
#include "commander.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include <time.h>
#include <stdio.h>
#include "platformservice.h"
#include <stdlib.h>
#include "debug.h"
#include "log.h"
#include "param.h"
#include "math.h"

#include "../../hal/interface/ledseq.h"
#include "radiolink.h"

static SemaphoreHandle_t sendMutex;

static xQueueHandle  rxQueue;

static bool overflow;

static int sendDataPacket(void* data, size_t length, const bool doBlock);

FlightState state;

static const uint16_t radius = 300;

static float height_sp = 0.30f;
static const float velMax = 0.7f;
short dataList[11];

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

/*
  Communication methods using the CRTP
*/

struct packetRX {
  int commandTag;
} __attribute__((packed));

struct packetTX {
  int replyCode;
} __attribute__((packed));

int appchannelSendDataPacket(void* data, size_t length)
{
  return sendDataPacket(data, length, false);
}

void appchannelSendDataPacketBlock(void* data, size_t length)
{
  sendDataPacket(data, length, true);
}

size_t appchannelReceiveDataPacket(void* buffer, size_t max_length, int timeout_ms) {
  static CRTPPacket packet;
  int tickToWait = 0;

  if (timeout_ms < 0) {
    tickToWait = portMAX_DELAY;
  } else {
    tickToWait = M2T(timeout_ms);
  }

  int result = xQueueReceive(rxQueue, &packet, tickToWait);

  if (result == pdTRUE) {
    int lenghtToCopy = (max_length < packet.size)?max_length:packet.size;
    memcpy(buffer, packet.data, lenghtToCopy);
    return lenghtToCopy;
  } else {
    return 0;
  }
}

bool appchannelHasOverflowOccured()
{
  bool hasOverflowed = overflow;
  overflow = false;

  return hasOverflowed;
}

void appchannelInit()
{
  sendMutex = xSemaphoreCreateMutex();

  rxQueue = xQueueCreate(10, sizeof(CRTPPacket));

  overflow = false;
  state = idle;
}

void appchannelIncomingPacket(CRTPPacket *p)
{
  int res = xQueueSend(rxQueue, p, 0);

  if (res != pdTRUE) {
    overflow = true;
  }
}

static int sendDataPacket(void* data, size_t length, const bool doBlock)
{
  static CRTPPacket packet;

  xSemaphoreTake(sendMutex, portMAX_DELAY);

  packet.size = (length > APPCHANNEL_MTU)?APPCHANNEL_MTU:length;
  memcpy(packet.data, data, packet.size);

  // CRTP channel and ports are set in platformservice
  int result = 0;
  if (doBlock)
  {
    result = platformserviceSendAppchannelPacketBlock(&packet);
  } else {
    result = platformserviceSendAppchannelPacket(&packet);
  }

  xSemaphoreGive(sendMutex);

  return result;
}

/*
*  The command handler receives a tag from the server
*  This method updates our flight state accordingly or blinks the LEDs
*/
int commandHandler(int commandTag){
      ledseqEnable(true);
      int replyCode;
      // Identify
      if (commandTag == 01){   
        ledseqRun(&seq_testPassed);
        vTaskDelay(M2T(1000));
        ledseqStop(&seq_testPassed);
        //state = idle;
        replyCode = 0101;
      }
      // Start mission
      if (commandTag == 02){
        // We start the mission
        state = takeOff;
        replyCode = 0102;
      }
      // Stop mission
      if (commandTag == 03){
        // We stop the mission
        state = emergencyStop;

        replyCode = 0103;
      }
      if (commandTag == 04){
        // We stop the mission
        ledseqRun(&seq_testPassed);
        vTaskDelay(M2T(2000));
        ledseqStop(&seq_testPassed);
        state = returnToBase;

        replyCode = 0104;
      }
      ledseqEnable(false);
      return replyCode;
}
/*
void p2pcallbackHandler(P2PPacket *p)
{
  // Parse the data from the other crazyflie and print it
  uint8_t other_id = p->data[0];
  static char msg[MESSAGE_LENGHT + 1];
  memcpy(&msg, &p->data[1], sizeof(char)*MESSAGE_LENGHT);
  msg[MESSAGE_LENGHT] = 0;
  uint8_t rssi = p->rssi;

}*/

/*
  Methods and attributes of the drone flight control
  */
static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = x;
  setpoint->velocity.y = y;
    /*
  if(returnToBase){
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->position.x = x;
    setpoint->position.y = y;
  } else {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = x;
    setpoint->velocity.y = y;
  }*/
  setpoint->velocity_body = true;  
}
/*
static void dodgeObstacle(setpoint_t *setpoint, float velFront, float velSide, float cmdHeight){
  
  int turnDirection = 0;
  turnDirection = rand()%2;
  float randomNumber = (float)rand()/(float)(RAND_MAX/150.0f);
  float yawrateComp = 0.0;

  if (turnDirection==1){
    yawrateComp= randomNumber + 30.0f;
    setHoverSetpoint(&setpoint, velFront, velSide, cmdHeight, yawrateComp);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(1000));
  } else {
    yawrateComp= -randomNumber + 30.0f;
    setHoverSetpoint(&setpoint, velFront, velSide, cmdHeight, yawrateComp);
    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(1000));
  }
}
*/
void appMain()
{
  ledseqEnable(true);
 
        ledseqRun(&seq_testPassed);
        vTaskDelay(M2T(1000));
        ledseqStop(&seq_testPassed);
        ledseqEnable(false);
}
LOG_GROUP_START(app)
LOG_ADD(PARAM_UINT8, state, &state)
LOG_GROUP_STOP(app)