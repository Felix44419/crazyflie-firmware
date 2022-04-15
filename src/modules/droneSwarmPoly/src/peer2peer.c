#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "../../hal/interface/ledseq.h"

float *distanceFromBase;

void p2pcallbackHandler(P2PPacket *p)
{
  // Parse the data from the other crazyflie
  float peerDistance = p->data[1];
    // if the other drone is closer to the base
    if (peerDistance > *distanceFromBase){
        ledseqStop(&seq_linkDown);
        vTaskDelay(M2T(10));
        ledseqRun(&seq_alive);
    } else if (peerDistance < *distanceFromBase){
        ledseqStop(&seq_alive);
        vTaskDelay(M2T(10));
        ledseqRun(&seq_linkDown);
    } else {
        ledseqStop(&seq_alive);
        ledseqStop(&seq_linkDown);
        ledseqRun(&seq_charged);
    }
}

void appMain()
{
    // Initialize the p2p packet 
    static P2PPacket p_reply;
    float posX = 0.0f;
    float posY = 0.0f;
    p_reply.port=0x00;

    logVarId_t idPosX = logGetVarId("stateEstimate", "x");
    logVarId_t idPosY = logGetVarId("stateEstimate", "y");

    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;

    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);

  while(1) {
    // Send a message every 2 seconds
    posX = logGetFloat(idPosX);
    posY = logGetFloat(idPosY);
    *distanceFromBase = sqrt(pow( posX,2 ) + pow( posY, 2));

    p_reply.data[1]=*distanceFromBase;

    vTaskDelay(M2T(500));
    radiolinkSendP2PPacketBroadcast(&p_reply);
  }
}

