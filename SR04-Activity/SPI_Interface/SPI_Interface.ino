#include<SPI.h>
volatile boolean received;
volatile uint8_t  slaveReceived;
void setup()
{
  Serial.begin(9600);
  pinMode(MISO,OUTPUT);                   //Sets MISO as OUTPUT

  SPCR |= _BV(SPE);                       //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt();                  //Interuupt ON is set for SPI commnucation

}

ISR (SPI_STC_vect)                        //Inerrrput routine function
{
    slaveReceived = SPDR;                 // Value received from master if store in variable slaveReceived
    received = true;                        //Sets received as True

}

void loop()
{
  if(received)
  {
      if (slaveReceived == 1)             //Switch: ON  |  Pot: >50%  |  Distance: <4 inches
      {
        Serial.println("Object is closer than 4 inches");
      }
      else if(slaveReceived == 2)         //Switch: ON  |  Pot: <50%  |  Distance: <4 inches
      {
        Serial.println("Pot value is less than 50%");
      }
      else if(slaveReceived == 3)         //Switch: ON  |  Pot: >50%  |  Distance: don't care
      {
        Serial.println("Object is farther than 4 inches");
      }
      else                                //Switch: OFF  |  Pot: don't care  |  Distance: don't care
      {
        Serial.println("Switch not enabled");
      }
   }
   else
   {
      Serial.println("***SPI connection not established***");
   }
}
