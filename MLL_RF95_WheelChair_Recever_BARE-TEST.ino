#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_Si7021.h"

/*  Feather M0 RFM9x
#define RFM95_CS	8        // Feather LoRa Native is A8
#define RFM95_INT	3
#define RFM95_RST	4
*/

#define RFM95_CS	5		// Breakout LoRa Module
#define RFM95_INT	6
#define RFM95_RST	11

#define RF95_FREQ 438.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct  LoRa_Data {				// 28 Byte Data Packet from WheelChair
    int		wheelchair;			// WheelChair #
	int		command;			// Command from Controller
    float	measuredVbat ;		// Battery Voltage
    float	homeLightFloat ;	// Light value
    float	homeTempFloat ;		// Temperature value
    float	homePressureFloat ; // ** Not Used
    float	homeHumidityFloat ;	// Humidity value
    float	spare ;
} ;
LoRa_Data  lora_data  {0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} ;

int	mywheelchair = 1 ;



Adafruit_Si7021		sensor = Adafruit_Si7021();

#define vbatPin A7              // A7 for Battery Voltage
#define v3_3    A5              // A5 for the 3.3 Volt regulator
#define LED_PIN 13              // A13 for On-Board LED (Red, next to the USB port)
#define LED_External 13			// A1 External LED for WheelChair
#define Buzzer_PWM 12 			// Buzzer pin

int Buzzer_Tone = 4500 ;		// Buzzer Tone in Hz
int Buzzer_Duration = 100 ;		// Buzzer Duration in ms
int Buzzer_Delay = 2000;		// Buzzer Delay in ms

int sensorPin = A0;             // A0 for the Lux Sense Pin (GA1A12S202)
float logLux;

int sram = 0;
int rawValue;
float rawRange = 1305;          // 3.3v 
float logRange = 5.0;           // 3.3v = 10^5 lux
float measuredvbat;

unsigned long previousMillis = 0;         // will store last time LED was updated
unsigned long interval = 5000;          // interval (milliseconds) - set to 120 sec
unsigned long targetMillis ;              // Target Time for next sensor acquisition
unsigned long currentMillis;
unsigned long totalIntervals = 0 ;        // Total number of Intervals since RESET (we want 30*60*12) for 12 hours

int temp = 0 ;
int sleepMS = 0;
int x = 0;

//*
//*
//***************************************************************
//  
//  VOID setup()
//    
//***************************************************************
void setup()  {
    Serial.begin(9600);
    delay(3000);
    
    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, 
    //    Cr = 4/5, Sf = 128chips/symbol, CRC on

    // Manually Reset the RF95 module
        Serial.println("Reset the RF95 module");
        digitalWrite(RFM95_RST, LOW);
        delay(10);
        digitalWrite(RFM95_RST, HIGH);
        delay(10);

        Serial.println("Initialize the RF95") ;
		
    if (!rf95.init()) {
        Serial.println("RF95 Initialization FAILED");  
        delay(500);
    }
        
    if(!rf95.setFrequency(RF95_FREQ) ) {
        Serial.println("Frequency Change FAILED");
    
    }
        Serial.print("RF95 Initialization SUCCESSFUL = ");
        Serial.println(RF95_FREQ) ;
        
        Serial.println("RF Power SetPoint");
        rf95.setTxPower(5, false);
        Serial.println("RF Power Setting - COMPLETE");
        
        Serial.println("Start Command Processing");
        Serial.println("-------------------------");
        Serial.println();
        delay(2000) ;
        digitalWrite(LED_PIN, LOW);			//  Initialization is done, turn OFF LED
    
		tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ;		//  Quick "Chirp" of the Buzzer on Start-up
		delay(250);
		tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ;		//  Quick "Chirp" of the Buzzer on Start-up
		
    interval = 5000 ;
    previousMillis = millis();
}

//
//***************************************************************
//  
//  void loop() 
//
//***************************************************************
void loop()  {
//	Listen for Commands from Controller
	if(rf95.available() ) {		//	is there a Command being transmitted ?
		digitalWrite(LED_PIN, HIGH);
		Serial.println("Command Detected");
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);   
    
		if (rf95.recv(buf, &len)) {
			RH_RF95::printBuffer("Data Received : ", buf, len);  //	get the Command being transmitted
			Serial.print("Buffer Length = "); Serial.println(len);
			
			memcpy(&lora_data, &buf, 28)  ;    //  copy the buffer to the lora.Struct
			Serial.println();
			Serial.print("Wheelchair	= "); Serial.println(lora_data.wheelchair);
			Serial.print("Command		= "); Serial.println(lora_data.command);

			Serial.println("Command Received - determining if for THIS Wheelchair");
			Serial.print("lora_data.wheelchair = "); Serial.println(lora_data.wheelchair);
			Serial.print("mywheelchair # - "); Serial.println(mywheelchair);
				
			switch(lora_data.command) {  			//	Execute COMMAND based on Switch/Case		
				case 1:		// Reply-to-Inquiry 
					if(lora_data.wheelchair == mywheelchair) {	//	test if it's for THIS WheelChair
						lora_data.homeTempFloat = sensor.readTemperature() ;
						lora_data.homeHumidityFloat = sensor.readHumidity() ;

						//	Obtain Battery Voltage
						measuredvbat = analogRead(vbatPin);
						measuredvbat *= 2;      // we divided by 2, so multiply back
						measuredvbat *= 3.3;    // Multiply by 3.3V, our reference voltage
						measuredvbat /= 1024;   // convert to voltage
						lora_data.measuredVbat = measuredvbat ;

						//	Obtain Light Value
						rawValue = analogRead(sensorPin);   // read the raw value from the sensor:
						lora_data.homeLightFloat = RawToLux(rawValue);
						
						if(rf95.isChannelActive() ) {
							delay(500) ;
							Serial.println("Channel is active - Waiting 500ms intervals") ;
						}
						//  Channel is CLEAR - Start Transmission
						rf95.send((uint8_t*)&lora_data, 28) ;
						//delay(10) ;
						Serial.println("Waiting for packet to complete sending...");
						
						//	Wait until Packet is sent before continuing
						rf95.waitPacketSent() ;
						Serial.println("Packet SENT");
						Serial.println() ;
					}
					return ;
					
				case 2:		// Chirp-and-LED for 16-sec
					if(lora_data.wheelchair == mywheelchair) {	//	test if it's for THIS WheelChair
						Serial.println();
						Serial.println("CHIRP-and-LED for 16-seconds");

						// Now start the CHIRP process
						digitalWrite(LED_External, HIGH) ;	// Turn-ON the LED
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						digitalWrite(LED_External, LOW) ;	// Turn-OFF the LED
						
						Serial.println("CHIRP-and-LED COMPLETE - 16-seconds");
						Serial.println();
					}
					return ;
					
				case 3:		// Chirp-and-LED for 32-sec
					if(lora_data.wheelchair == mywheelchair) {	//	test if it's for THIS WheelChair
						Serial.println();
						Serial.println("CHIRP-and-LED for 32-seconds");						
						digitalWrite(LED_External, HIGH) ;	// Turn-ON the LED
												
						// Now start the CHIRP process
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
						
						digitalWrite(LED_External, LOW) ;	// Turn-OFF the LED
												
						Serial.println("CHIRP-and-LED COMPLETE - 32-seconds");
						Serial.println();
					}
					return ;
					
				case 4:		// Exit-Alert
					//	This Routine applies to ALL Wheelchairs - so no checking of the Unit #
					Serial.println("Exit-Alert");
					digitalWrite(LED_External, HIGH) ;	// Turn-ON the LED
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					tone(Buzzer_PWM, Buzzer_Tone, Buzzer_Duration) ; delay(Buzzer_Delay);
					digitalWrite(LED_External, LOW) ;	// Turn-OFF the LED
					return ;
				}
			}
		}

/*	If we reach the Interval time - transmit a status packet  
	if(millis() > previousMillis + interval) {
		previousMillis = millis() ;
		digitalWrite(LED_PIN, HIGH);    //  Flash the LED ON-OFF
		delay(250);
        
		//  Obtain OUTSIDE Temperature and Humidity Values
		lora_data.homeTempFloat = sensor.readTemperature() ;
		lora_data.homeHumidityFloat = sensor.readHumidity() ;

		//	Obtain Battery Voltage
		measuredvbat = analogRead(vbatPin);
		measuredvbat *= 2;      // we divided by 2, so multiply back
		measuredvbat *= 3.3;    // Multiply by 3.3V, our reference voltage
		measuredvbat /= 1024;   // convert to voltage 
		lora_data.measuredVbat = measuredvbat ;

		//	Obtain Light Value   
		rawValue = analogRead(sensorPin);   // read the raw value from the sensor:
		lora_data.homeLightFloat = RawToLux(rawValue);

		Serial.print("WheelChair #  = ");
		Serial.println(lora_data.wheelchair) ;
		Serial.print("Battery = ");
		Serial.println(lora_data.measuredVbat);
		Serial.print("Light   = ");
		Serial.println(lora_data.homeLightFloat);
		Serial.print("Temp    = ");
		Serial.println(lora_data.homeTempFloat);
		Serial.print("Humid   = ") ;
		Serial.println(lora_data.homeHumidityFloat) ;

		//	Get Ready to Transmit - but Check Channel First
		if(rf95.isChannelActive() ) {
			delay(500) ;
			Serial.println("Channel is active - Waiting 500ms intervals") ;
		}
		//  Channel is CLEAR - Start Transmission
		rf95.send((uint8_t*)&lora_data, 28) ;
		//delay(10) ;
		Serial.println("Waiting for packet to complete sending..."); 
    
		//	Wait until Packet is sent before continuing
		rf95.waitPacketSent() ;
		Serial.println("Packet SENT");
		Serial.println() ;

		digitalWrite(LED_PIN, LOW);   //  Turn-OFF the Acquisition LED
  
	} */
}


 
//**************************************************************************
//
//  RawToLux Function for GA1A12 Analog Lux Sensor
//
//**************************************************************************
float RawToLux(int raw) { 
  // float logLux = raw * logRange / rawRange;    //  moved float definition to Mainline
  logLux = raw * logRange / rawRange; 
  return pow(10, logLux); 
}






