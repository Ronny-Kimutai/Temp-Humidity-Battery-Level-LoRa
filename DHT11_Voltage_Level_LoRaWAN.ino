 //Include the Libraries to be used for the LoRaWAN Transmission
#include<lmic.h>
#include<SPI.h>
#include<hal/hal.h>

//Include the library to be used for the DHT11 Sensor the library is a DHT11 Sensor Library by adafruit
#include "DHT.h"

//Define the Digital pin where the data pin of the DHT sensor will be connected to the board
#define DHTPIN 5

//Define the type of DHT Sensor to be used
#define DHTTYPE DHT11

//Define the DHT Parameter so as to initialize the DHT Sensor
DHT dht(DHTPIN, DHTTYPE);

//VOLTAGE LEVEL
int getAccurateVoltage() {
  readVcc();
  return readVcc();
}

int readVcc(void) {

  int result;

   ADCSRA = (1<<ADEN); 
   /*ADEN is the ADC enable Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off. Turning the ADC off while a conversion is 
     in progress, will terminate this conversion*/
   ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
   /*These bits determine the division factor between the system clock frequency and input clock to the ADC
   As per the datasheet, the ADC must be read used with a clock between 50 and 200Khz 
   The system clock frequency for ATmega 328p is 8Megahertz hence when ADPS2:0 are set to
   111 the division factor is 128 and 8000/128 gives you 62.5Kilo hertz which is within the specified range*/

  // set the reference to Vcc and the measurement to the internal 1.1V reference

   ADMUX = (1<<REFS0) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
   /*REFS1:0 - select the voltage reference of the ADC - when S1 is set to 0
   and S0 set to 1 the voltage reference used is AVcc with external capacitor at AREF pin
   MUX3:0 - the value of these bits selects which analog inputs are connected to the ADC
   when they are set to 1110 respectively the single ended input is 1.1V(Bandgap voltage)
   Due to the presence of the equal sign the other missing registers in the equation are set to 0*/
   
   delay(1); // Wait for ADC and Vref to settle

   ADCSRA |= (1<<ADSC); // Start conversion
   /*ADSCRA=ADSCRA|0100 0000 - Ensure the ADSC is set to 1*/
   //ADSC - ADC Start conversion is set to 1 so as to start each conversion
   while (bit_is_set(ADCSRA,ADSC)); // wait until done
   //Checks whether the ADSC is set to 1 using the while loop if it is 1 conversion occurs
   result = ADC;
  

 //The internal voltage reference was measured and obtained to be 1.076V
 //The formula is given on the datasheet
 //1126400 = 1.076*1024*1000 = 1101824
  result = 1101824UL / (unsigned long)result; 
  return result; // Vcc in millivolts
}

//APPEUI, DEVEUI and APPKEY
//LITTLE ENDIAN FORMAT - The APPEUI should be in little endian format where the LSB comes first
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

//LITTLE ENDIAN FORMAT - The DEVEUI should be in little endian format where the LSB comes first
static const u1_t PROGMEM DEVEUI[8] = { 0xD5, 0xD4, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

//BIG ENDIAN FORMAT - The APPKEY should be in big endian format where the MSB comes first
static const u1_t PROGMEM APPKEY[16] = { 0x92, 0x29, 0x16, 0x4A, 0xB2, 0x03, 0x86, 0xF4, 0xB0, 0xDC, 0x7A, 0x0F, 0x35, 0xF4, 0x7F, 0xC8 };
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

//Payloads to be sent to the Things Network Gateway
static uint8_t payload[6];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

//Pin Mapping
//The Board used is a WaziDev 1.3 hence the Pin Mapping is as shown below
const lmic_pinmap lmic_pins = {
  .nss = 10, // It is connected to the Micro-controller
  .rxtx = LMIC_UNUSED_PIN, //Unused
  .rst = 4, // It is connected to the Micro-controller reset pin
  .dio = {2,3,LMIC_UNUSED_PIN},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i) {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
        }
        LMIC_setLinkCheckMode(0);
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.print(F(" bytes of payload: "));
        }
        //Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
        
        default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}
void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else {
      //Humidity values are read from the DHT11 Sensor and are multiplied by 100 to fit the specified range
        uint16_t humidity = dht.readHumidity(false)*100;
        
      //Temperature values are read from the DHT11 Sensor and are multiplied by 100 to fit the specified range
        uint16_t temperature = dht.readTemperature(false)*100;

      //Voltage 
        uint16_t voltage = getAccurateVoltage();

      //Bytes are placed into the payload for transmission
        byte payload[6];
        payload[0] = highByte(humidity);
        payload[1] = lowByte(humidity);
        payload[2] = highByte(temperature);
        payload[3] = lowByte(temperature);
        payload[4] = highByte(voltage);
        payload[5] = lowByte(voltage);

        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        
        LMIC_setTxData2(1, (byte*)payload, sizeof(payload), 0);

        //Measured Temperature, Humidity and Voltage level are displayed on the serial monitor
        Serial.println("The Humidity is : " + String(humidity/100)+" %");
        Serial.println("The Temperature is : " + String(temperature/100)+" degrees celcius");
        Serial.println("The Voltage is : " + String(voltage/1000) + " Volts");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    delay(5000);
    while (! Serial);
    Serial.begin(9600);
    Serial.println(F("Starting"));

    dht.begin();

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    //The Arduino Pro mini is running at 8MHz and is relatively slow for LMIC
    //and needs an adjustment by relaxing the timing
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {

  os_runloop_once();
}
