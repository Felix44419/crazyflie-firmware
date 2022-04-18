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
  static setpoint_t setpoint;
  //static setpoint_t startpoint;
  state = idle;
  float startPosX = 0.0f;
  float startPosY = 0.0f;

  DEBUG_PRINT("Waiting for activation ...\n");
  vTaskDelay(M2T(3000));
  struct packetRX rxPacket;
  struct packetTX txPacket;
  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");
  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t bat = logGetVarId("pm", "vbat");

  logVarId_t idPosX = logGetVarId("stateEstimate", "x");
  logVarId_t idPosY = logGetVarId("stateEstimate", "y");
  logVarId_t idYaw = logGetVarId("stabilizer", "yaw");

  float vbat = logGetUint(bat);

  float factor = velMax/radius;
  DEBUG_PRINT("Waiting for activation ...\n");
  while(1) {
    float vbat1 = logGetUint(bat);
    // We continuously call this method to ensure the rxQueue does not overflow and to update our status
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 0)) {
      txPacket.replyCode = commandHandler(rxPacket.commandTag);
      appchannelSendDataPacket(&txPacket, sizeof(txPacket));
    }

    // The drone moves depending on its state, wether we have a packet incoming or not
    vTaskDelay(M2T(50));
    //logVarId_t idUp = logGetVarId("range", "up");
    

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    DEBUG_PRINT("Position init value: %i", positioningInit);
    DEBUG_PRINT("Multiranger init value: %i", multirangerInit);
    float vbat2 = logGetUint(bat);
/*
    if( positioningInit && multirangerInit){}
      if (state == exploring && vbat<3.77f){
        state = emergencyStop;
    }
*/
    if (state == takeOff && positioningInit && multirangerInit /*&& vbat>3.77f*/) {
      setHoverSetpoint(&setpoint, 0, 0, height_sp, 0);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(10));

      // We read the current position as the start position of the drone
      startPosX = logGetFloat(idPosX);
      startPosY = logGetFloat(idPosY);

      state = exploring;
    } else if (state == idle) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
    }
    float vbat3 = logGetUint(bat);

    if (state == exploring){
      uint16_t left = logGetUint(idLeft);
      uint16_t right = logGetUint(idRight);
      uint16_t front = logGetUint(idFront);
      uint16_t back = logGetUint(idBack);
      uint16_t up = logGetUint(idUp);

      uint16_t left_o = radius - MIN(left, radius);
      uint16_t right_o = radius - MIN(right, radius);
      uint16_t front_o = radius - MIN(front, radius);
      uint16_t back_o = radius - MIN(back, radius);
      uint16_t up_o = radius - MIN(up, radius);
      float b_comp = back_o * factor;
      float l_comp = (-1) * left_o * factor;
      float r_comp = right_o * factor;
      float f_comp = (-1) * front_o * factor;

      float velSide = r_comp + l_comp;
      float velFront = b_comp + f_comp;
      float cmdHeight = height_sp - up_o / 1000.0f;
      float yawrateComp = 0.0f;
      if (cmdHeight < height_sp - 0.2f)
      {
        state = emergencyStop;
      }
      if ( (front_o ) != 0 ){
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
        velFront = 0;
      }
      if ( (front_o ) == 0 ){
        yawrateComp= 0;
        velFront = 0.2f;
      }

      setHoverSetpoint(&setpoint, velFront, velSide, cmdHeight, yawrateComp);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(10));
      float vbat4 = logGetUint(bat);
      float vbat12 = MAX(vbat1,vbat2);
      float vbat23 = MAX(vbat12,vbat3);
      float vbat34 = MAX(vbat23,vbat4);
      vbat = vbat34;
      vbat=vbat;
    }

    if (state == emergencyStop) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
        state=idle;
    }   

    if (state == returnToBase) {
      uint16_t left = logGetUint(idLeft);
      uint16_t right = logGetUint(idRight);
      uint16_t front = logGetUint(idFront);
      uint16_t back = logGetUint(idBack);

      uint16_t left_o = radius - MIN(left, radius);
      uint16_t right_o = radius - MIN(right, radius);
      uint16_t front_o = radius - MIN(front, radius);
      uint16_t back_o = radius - MIN(back, radius);

      float b_comp = back_o * factor;
      float l_comp = (-1) * left_o * factor;
      float r_comp = right_o * factor;
      float f_comp = (-1) * front_o * factor;

      float velSide = r_comp + l_comp;
      float velFront = b_comp + f_comp;
      float cmdHeight = height_sp;
      float yawrateComp = 0.0f;

      if ( (front_o ) != 0 ){
        int turnDirection = 0;
        turnDirection = rand()%2;
        float randomNumber = (float)rand()/(float)(RAND_MAX/150.0f);
        yawrateComp = 0.0f;

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
        velSide = 0;
        velFront = 0;
      }
      // We get the current position
      float currentPosX = logGetFloat(idPosX);
      float currentPosY = logGetFloat(idPosY);
      float currentYaw = logGetFloat(idYaw);

      // We want the dot product, yet the yawInit vector is just 1;0 therefore the dot product is the X component of the vectorR2B
      float vectorReturnToBaseX = startPosX - currentPosX; 
      float vectorReturnToBaseY = startPosY - currentPosY;
      // Vectors length
      float lengthVectorR2B = sqrt(pow( vectorReturnToBaseX,2 ) + pow( vectorReturnToBaseY, 2));
      // Extracting the angle from the dot product formula
      float initAngleOffset = acos(vectorReturnToBaseX/lengthVectorR2B);
      
      yawrateComp = currentYaw + initAngleOffset;
      if( yawrateComp > 5){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, cmdHeight, yawrateComp);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(1000));
      }

      if ( (front_o) == 0 ){
        yawrateComp= 0;
        velFront = 0.15f;
      }
      setHoverSetpoint(&setpoint, velFront, velSide, cmdHeight, 0);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(10));

      // If we arrive less than 1m from the base we have reached our destination and can land safely
      // We use the Euclidean distance 
      float distanceToBase = sqrt(pow( (currentPosX - startPosX),2 ) + pow( (currentPosY - startPosY), 2));
      if (distanceToBase < 1.0f){
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
        state=idle;
      }
    }
  }
}