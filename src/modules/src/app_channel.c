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

#include "platformservice.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "../../hal/interface/ledseq.h"

static SemaphoreHandle_t sendMutex;

static xQueueHandle  rxQueue;

static bool overflow;

static int sendDataPacket(void* data, size_t length, const bool doBlock);

FlightState state;
static setpoint_t setpoint;

static const float velMax = 0.3f;
static const uint16_t radius = 300;

static const float height_sp = 0.2f;

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
        replyCode = 420;
      }
      // Start mission
      if (commandTag == 02){

        ledseqRun(&seq_missionStart);
        vTaskDelay(M2T(450));
        ledseqStop(&seq_missionStart);
        // We start the mission
        state = takeOff;

        replyCode = 9000;
      }
      // Stop mission
      if (commandTag == 03){

        ledseqRun(&seq_missionStop);
        vTaskDelay(M2T(1250));
        ledseqStop(&seq_missionStop);
        // We stop the mission
        state = emergencyStop;

        replyCode = 9001;
      }
            // Drone status
      if (commandTag == 04){
        if (state == idle){
          replyCode = 0201;
        }
        if (state == takeOff){
          replyCode = 0202;
        }
        if (state == exploring){
          replyCode = 0203;
        }
        if (state == emergencyStop){
          replyCode = 0204;
        }
        if (state == returnToBase){
          replyCode = 0205;
        }
      }
      ledseqEnable(false);
      return replyCode;
}
/*
  Methods and attributes of the drone flight control
  


*/
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

static bool takeOffDrone(){
  if((setpoint.position.z-2.0f)<0.01f) return false;
  setHoverSetpoint(&setpoint, 0, 0, 2.0, 0);
  commanderSetSetpoint(&setpoint, 3);
  state = exploring;
  return true;
}  

static bool land(){
  if((setpoint.position.z<0.01f) return false;
  setHoverSetpoint(&setpoint, 0, 0, 0, 0);
  commanderSetSetpoint(&setpoint, 3);
  return true;
}

static void goForward(){
  setHoverSetpoint(&setpoint, 1, 0, 0, 0);
  commanderSetSetpoint(&setpoint, 3);
}

static void obstacleDodge(){
  setHoverSetpoint(&setpoint, 0, 0, setpoint.position.z, (rand()%60) + 30);
  commanderSetSetpoint(&setpoint,3);
  vTaskDelay(M2T((rand()%1000)+1000));
}

// TODO get a time reading of the total delay to run this method.
// The delay shall be used to measure how long we can wait for a packet 
void flightControl(){

  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");
  
  //paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  //paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  float factor = velMax/radius;

  //DEBUG_PRINT("%i", idUp);
  
  vTaskDelay(M2T(10));
  //DEBUG_PRINT(".");

  //uint8_t positioningInit = paramGetUint(idPositioningDeck);
  //uint8_t multirangerInit = paramGetUint(idMultiranger);

  uint16_t up = logGetUint(idUp);

  if (state == takeOff) {
    uint16_t left = logGetUint(idLeft);
    uint16_t right = logGetUint(idRight);
    uint16_t front = logGetUint(idFront);
    uint16_t back = logGetUint(idBack);

    uint16_t left_o = radius - MIN(left, radius);
    uint16_t right_o = radius - MIN(right, radius);
    float l_comp = (-1) * left_o * factor;
    float r_comp = right_o * factor;
    float velSide = r_comp + l_comp;
    
    uint16_t front_o = radius - MIN(front, radius);
    uint16_t back_o = radius - MIN(back, radius);
    float f_comp = (-1) * front_o * factor;
    float b_comp = back_o * factor;
    float velFront = b_comp + f_comp;

    uint16_t up_o = radius - MIN(up, radius);
    float height = height_sp - up_o/1000.0f;
    if (velSide) {
      velSide = velFront + height + 1;
    }
    takeOffDrone();
  }
  if (state == exploring){
      goForward();
    if (front_o!=0){
      obstacleDodge();
    }
    goForward();
  }
  if (state == emergencyStop) {
    land();
  }
}

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  struct packetRX rxPacket;
  struct packetTX txPacket;

  while(1) {
    // The drone moves depending on its state, wether we have a packet incoming or not
    flightControl();

    // We continuously call this method to ensure the rxQueue does not overflow and to update our status
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 50)) {
      txPacket.replyCode = commandHandler(rxPacket.commandTag);
      appchannelSendDataPacket(&txPacket, sizeof(txPacket));
    }
  }
}