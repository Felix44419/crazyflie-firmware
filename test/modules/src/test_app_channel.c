//#include "app_channel.h"
#include "unity.h"
#include "mock_app_channel.h"

CRTPPacket replyPk;

void setUp(void)
{
}

void tearDown(void)
{
}

struct packetRX {
  int commandTag;
} __attribute__((packed));

struct packetTX {
  int replyCode;
} __attribute__((packed));

void testAppchannelShouldReceiveIdentifyReturnReply() {
    struct packetRX testPacketRX;
    struct packetTX testPacketTX;
    testPacketRX.commandTag=1;
    mock_app_channel_Init();
    
    appchannelReceiveDataPacket_CMockExpectAndReturn(145,&testPacketRX, sizeof(testPacketRX), APPCHANNEL_WAIT_FOREVER,181);
    
  //Test if the identify returns the reply code 420
  TEST_ASSERT_EQUAL(420, testPacketTX.replyCode);
}