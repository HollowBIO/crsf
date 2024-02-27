#include <CrsfSerial.h>

//Pins
#define LED_SIG     PC13
#define BOARD_USART1_TX_PIN   PB6
#define BOARD_USART1_RX_PIN   PB7

//HardwareSerial Serial1(BOARD_USART1_RX_PIN, BOARD_USART1_TX_PIN);
CrsfSerial crsf(Serial1);

void PacketChannels()
{
  for(int i = 0; i < 16; i++)
  {
    Serial.print("ch"); Serial.print(i+1); Serial.print(":"); Serial.print(crsf.getChannel(i+1)); Serial.print(" ");
  }
  Serial.println();
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
}
