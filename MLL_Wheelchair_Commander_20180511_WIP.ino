
/****************************************************************
	  CCF WheelChair Tracking System
	    Controller Module - 05/14/2018
	
		Sends Commands to WheelChair Module Module and Receives Data:
		1) Reply-to-Inquiry - send Inquiry and Receive Response Data
		2) Chirp-Short - Chirp 3 times and LED(ON)
		3) Chirp-Constant - continuous Chirp/LED(ON)
		4) Chirp-OFF - turn-off Chirp/LED
		5) Exit-Alert - Trigger Chirp-Constant at Exit point

		WheelChair Modules have :
	    Temperature/Humidity Sensor (Si7021)
	    GA1A12S202 External LUX Sensor - INSIDE
	    Resistor Divider for Voltage Sensing
	
	    A0  = Lux Sense Pin (GA1A12S202)
		A5  = Analog Pin used to read the 3.3v regulator
	    A7  = Analog Pin used to read the LiPoly battery voltage
	      A13 = Default LED

		Controller Module has :
			LoRa Transceiver (Feather LoRa 433)
			OLED to display WheelChair data
			RTC/SD Module to record Data and Time-Stamps
			3 Momentary Push buttons (on the OLED Display Board)
				Button-A	SHORT = Wheelchair++		LONG = WheelChair reset to #1
				Button-B	SHORT = Chirp-LED			LONG = Chirp-LED for 30-sec
				Button-C	SHORT = Reply-to-Inquiry	LONG = Exit-Alert
		
		Device Mapping
			unassigned	= 2   reserved
			unassigned  = 3   reserved
			unassigned	= 4	  reserved
			Button_C	= 5   OLED Display, Button-C
			Button_B	= 6   OLED Display, Button-B
			unassigned	= 7   reserved
			unassigned	= 8   reserved
			V-Divider   = 9   Voltage Divider on the 3.3v line
			Button_A	= 9	  OLED Display, Button-A (Disable Pull-up before Reading)
			SD/RTC-CS   = 10  FeatherWing with M0
			RF95 RST    = 11  Breakout board     
			unassigned  = 12  reserved
			Onboard LED = 13  red LED
		
		WheelChair Numbers (and Special Designations)
			0		-	not used
			1-20	-	Assignable Wheelchairs
			99		-	ALL Wheelchairs - broadcast message (used for Exit-Alert)
		
*/
#include <Adafruit_GFX.h>      // Graphics library
#include <Adafruit_SSD1306.h>   // Display Driver
#include <Adafruit_FeatherOLED.h> // OLED Driver
#include <SD.h>
#include "RTClib.h"					// Combined SD and RTC FeatherWing using I2C
#include <RH_RF95.h>				// Uses SPI

int button1	= 9;
int button2	= 6 ;
int button3	= 5;

boolean	button1Active = false ;
boolean	button2Active = false ;
boolean	button3Active = false ;

boolean LED1State = false;
boolean LED2State = false;

long button1Timer = 0;
long button2Timer = 0;
long button3Timer = 0;
long longPressTime = 250;

boolean buttonActive = false;
boolean longPressActive = false;

Adafruit_SSD1306 display = Adafruit_SSD1306();
const int sdChipSelect = 10 ;		//  for the SD Card
RTC_PCF8523   rtc ;					//  Instantiate the RTC/SD Wing

//-----------------------------
//  Feather M0-WiFi with STAND-ALONE RFM9x Break-Out Board
/*
#define RFM95_CS 5
#define RFM95_INT 6
#define RFM95_RST 11
#define RF95_FREQ 433.0
*/
//  Pin configuration for Feather M0 with built-in LoRa Module
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 438.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct  LoRa_Data {					//  28-byte record for SHORT transmissions
	int		wheelchair ;            //  which node being received
	int		command ;				//  Commands:	1=Reply-to-Inquiry,	2 = Chirp-LED06
	float	measuredVbat ;			//				3=Chip-LED30		4 = Exit-Alert (all wheelchair modules)
	float	homeLightFloat ;
	float	homeTempFloat ;
	float	homePressureFloat ;
	float	homeHumidityFloat ;
	float	spare ;
} ;
LoRa_Data  lora_data  {5, 1, 0.0, 0.0, 0.0, 0.0, 0.0} ;


void setup() {
	pinMode(button1, INPUT_PULLUP);
	pinMode(button2, INPUT_PULLUP);
	pinMode(button3, INPUT_PULLUP);  
  
	Serial.begin(9600);               //  Don't Wait, since we may be Stand-Alone
	delay(5000) ;
	Serial.println("Serial Port             ACTIVE") ;
  
	//  Start the SD and RTC Functions to Enable Logging
	if (!SD.begin(sdChipSelect)) {
		Serial.println("SD Card                 FAILED, OR NOT INSERTED");
	}
	else Serial.println("SD Card Initialization  PASSED") ;
	File dataFile = SD.open("wheelchairlog.csv", FILE_WRITE);
	
	if (! rtc.begin())
	Serial.println("RTC Initialization FAILED - writing BLANK Dates");
	Serial.println("RTC Initialitaion       PASSED") ;
	Serial.println("RTC to Computer Time    PASSED") ;
	rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
	
	//  Set-up the LoRa Receiver - We are using 438 MHz simplex
	if (!rf95.init()) {
		Serial.println("RF95 Initialization failed");
		delay(500);
	}
	rf95.setTxPower(5, false) ;
	Serial.println("RF95 Power Level Adjusted");
	
	if(!rf95.setFrequency(RF95_FREQ)) {
		Serial.println("Frequency Change FAILED");
	}
	Serial.print("RF95 Frequency =        ") ;
	Serial.println(RF95_FREQ) ;
	
	Serial.println("RF95 Initialization     PASSED");
	  
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
	Serial.println("OLED begun");          // init done
	// Clear the buffer.
	display.clearDisplay();
	display.display();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	Serial.println("System Reset Message") ;
	display.println("System Reset");
	display.display() ;
	delay(2000);
	display.clearDisplay();
	display.display();
	
	// Splash Screen on the OLED
	Serial.println("Writing CCF Splash to the OLED");
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.println("    CCF Host Team");
	display.println("  Wheelchair Locater");
	display.println(" ");
	display.println("  Serve Faithfully") ;
	display.setCursor(0,0);
	display.display();            // Display the 4-lines
	delay(3000) ;
	
	display.clearDisplay();
	display.display();
	display.setCursor(0,0);
	display.println("Current Status");
	display.println();
	display.print("Wheelchair = ");
	display.println(lora_data.wheelchair);
	display.display();
	
	Serial.println("Waiting for Input");
  
}


void loop() {
	
//=============================================================
//	Button-1 Processing
//	
	if (digitalRead(button1) == LOW) {
		if (button1Active == false) {
		  button1Active = true;
		  button1Timer = millis();
		}
		if ((millis() - button1Timer > longPressTime) && (longPressActive == false)) {
			longPressActive = true;         //  LONG PRESS
			Serial.println("LONG PRESS - #1");
			lora_data.wheelchair = 1 ; //  Reset the WheelChair Number
			Serial.println("Reset Wheelchair to 1") ;
			Serial.println();
			display.clearDisplay() ;
			display.display() ;
			display.setCursor(0,0);
			display.println("Reset Wheelchair") ;
			display.println();
			display.print("Wheelchair = ") ;
			display.println(lora_data.wheelchair);
			display.display() ;
			buttonActive == false ;
			button1Active == false ;
		}
	} 
	else {
		if (button1Active == true) {
			if (longPressActive == true) {
				longPressActive = false;
			} 
			else {
				//  SHORT PRESS
				Serial.println("Increment Wheelchair #") ;
				
				if (lora_data.wheelchair > 14) lora_data.wheelchair = 0 ;
				lora_data.wheelchair++ ; //  Increment the WheelChair Number
				display.clearDisplay();
				display.display();
				display.setCursor(0,0);
				display.println("Increment Chair #") ;
				display.println();
				display.print("Wheelchair = ") ;
				display.print(lora_data.wheelchair) ;            //  Update the OLED display
				display.display() ;
			}
			button1Active = false;
		}
	}  

//=============================================================
//	Button-2 Processing
//
	if (digitalRead(button2) == LOW) {
		if (button2Active == false) {
			button2Active = true;
			button2Timer = millis();
		}
		if ((millis() - button2Timer > longPressTime) && (longPressActive == false)) {
			longPressActive = true;         //  LONG PRESS
			// LONGPress #2 - Chirp/LED for 32-seconds   (CMD=3)
			Serial.println("Chirp-LED - 32-sec") ;
			display.clearDisplay();
			display.display();
			display.setCursor(0,0);
			display.println("Chirp-LED 32-sec") ;
			display.println();
			display.print("Wheelchair - ") ; display.println(lora_data.wheelchair);
			display.display() ;
			lora_data.command = 3 ;
			
			Serial.print("Wheelchair	= "); Serial.println(lora_data.wheelchair);
			Serial.print("Command		= "); Serial.println(lora_data.command);
			Serial.println("Packet SENT");
			
		}
	}
	else {
		if (button2Active == true) {
			if (longPressActive == true) {
				longPressActive = false;
			}
			else {				
				// SHORT Press #2 - Chirp/LED for 16-seconds   (CMD=2)
				Serial.println("Chirp-LED - 16-sec") ;
				display.clearDisplay();
				display.display();
				display.setCursor(0,0);
				display.println("Chirp-LED 16-sec") ;
				display.println();
				display.print("Wheelchair - ") ; display.println(lora_data.wheelchair);
				display.display() ;
				lora_data.command = 2 ;	
				
				//Channel is CLEAR - Start Transmission
				Serial.println("Sending Data via RF95 LoRa");	
				rf95.send((uint8_t*)&lora_data, 28) ;		
				
				//	Wait until Packet is sent before continuing
				rf95.waitPacketSent() ;					//	this is Synchronous - blocked
				delay(500); 
				Serial.print("Wheelchair	= "); Serial.println(lora_data.wheelchair);
				Serial.print("Command		= "); Serial.println(lora_data.command);
				Serial.println("Packet SENT");
			}
			button2Active = false;
		}
	} 
	
//=============================================================
//	Button-3 Processing
//
	if (digitalRead(button3) == LOW) {
		if (button3Active == false) {
			button3Active = true;
			button3Timer = millis();
		}
		if ((millis() - button3Timer > longPressTime) && (longPressActive == false)) {
			longPressActive = true;         //  LONG PRESS
			display.clearDisplay();
			display.display();
			display.setCursor(0,0);
			display.println(" Button-3") ;
			display.println("LONG PRESS") ;            //  Update the OLED display
			display.display() ;
			Serial.println("Button-3 LONG PRESS");
		}
	}
	else {
		if (button3Active == true) {
			if (longPressActive == true) {
				longPressActive = false;
			}
			else {
				//  SHORT PRESS
				display.clearDisplay();
				display.display();
				display.setCursor(0,0);
				display.println(" Button-3") ;
				display.println("SHORT PRESS") ;
				display.display();
				Serial.println("Button-3 SHORT PRESS");
			}
			button3Active = false;
		}
	}

}		// END-OF-LOOP








