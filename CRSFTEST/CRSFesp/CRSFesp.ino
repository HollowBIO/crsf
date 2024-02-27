#include "CrsfSerial.h"

//Pins
#define LED_SIG     PD2
#define BOARD_USART1_TX_PIN   PTX0
#define BOARD_USART1_RX_PIN   PRX0
#define PACKET_PERIOD 5

//HardwareSerial Serial1(BOARD_USART1_RX_PIN, BOARD_USART1_TX_PIN);
CrsfSerial crsf(Serial1);

uint32_t last_packet = 0;
uint16_t j = 0;

void PacketChannels()
{

}

void LinkUp()
{
  digitalWrite(LED_SIG, 0);
}

void LinkDown()
{
  digitalWrite(LED_SIG, 1);
}


void setup()
{
  //Pins
  pinMode(LED_SIG, OUTPUT);
  delay(200);
  digitalWrite(LED_SIG, 1);

  //Interfaces
  Serial.begin(115200);
  Serial1.setRx(BOARD_USART1_RX_PIN);
  Serial1.setTx(BOARD_USART1_TX_PIN);
  crsf.begin(420000);

  //Callbacks
  crsf.onPacketChannels = &PacketChannels;
  crsf.onLinkUp = &LinkUp;
  crsf.onLinkDown = &LinkDown;


}

void loop()
{
  crsf.loop();

  if(millis() - last_packet > PACKET_PERIOD)
  {
    crsf_channels_t crsfPack = {0};

    crsfPack.ch0 = map(1000 + j++%1000, 1000, 2000, 191, 1792);
    crsfPack.ch1 = map(1337, 1000, 2000, 191, 1792);

    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfPack, sizeof(crsfPack));
    last_packet = millis();
  }

}
