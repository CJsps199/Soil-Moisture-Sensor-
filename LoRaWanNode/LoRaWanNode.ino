/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include <CayenneLPP.h>
#include <LowPower.h>


boolean sleep_counter = 0;

CayenneLPP lpp(51);





// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xF0, 0xFF, 0x25, 0x4F, 0x6B, 0x0D, 0x74, 0x2D, 0x84, 0x17, 0x48, 0xEF, 0x6B, 0xD4, 0xBB, 0x42 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x85, 0x97, 0x8D, 0xF1, 0x2B, 0x9F, 0xA7, 0xC0, 0x94, 0x2D, 0xEF, 0xEA, 0xED, 0x4B, 0x47, 0xA1 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26061493 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
//void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
//void os_getDevKey (u1_t* buf) { }

static uint8_t payload[8];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 6;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

DHT dht(9, DHT11);

void onEvent (ev_t ev) {
    
    switch(ev) {
   
        case EV_TXCOMPLETE:
//            Serial.println("tx complete");
           
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
             
            break;
       
}
}

void do_send(osjob_t* j){
//while (!Serial);
// Serial.println("starting job");
    

    // Check if there is not a current TX/RX job running
   if (LMIC.opmode & OP_TXRXPEND) {
        delay(50);
        
   } else {

    if (sleep_counter == 1){
//    Serial.println("going to sleep");
    for (byte i = 0; i <= 6; i++){
      
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
      
      }
      delay(50);
//      Serial.println("Waking Up");
    } 
   
    sleep_counter = 1;
    

    float temperature = dht.readTemperature();
    
        

        // read the humidity from the DHT22
    byte rHumidity = dht.readHumidity();

     int BatR = analogRead(A1);
float BatV = BatR * 0.00499;   
        
    digitalWrite(10, HIGH);     
    delay(5);
    
    byte SoilMoistPercent = map(analogRead(A0), 0, 1023, 0, 100);

    digitalWrite(10, LOW);

    delay(5);

       
    lpp.reset();
        
    lpp.addTemperature(1, temperature);
    lpp.addRelativeHumidity(2, rHumidity);
    lpp.addAnalogInput(3, SoilMoistPercent);
    lpp.addAnalogInput(4, BatV);
      
        // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
//      Serial.println("data set");  
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
//   Serial.begin(115200);
    dht.begin();
//    Serial.println("starting");
delay(100);
    

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automaticall

    LMIC_setupChannel(0, 868100000, DR_SF7,  BAND_CENTI);      // g-band
    
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF7;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,20);

    // Start job
    do_send(&sendjob);
    
}

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }
  
    os_runloop_once();
    
}
