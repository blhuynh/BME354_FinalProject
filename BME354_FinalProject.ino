/*********************
 * BME 354 S2014 Final Project - Pulse Plethysmography
 * Brian Huynh, Michael Rees
 * 
 * This code measures the pulsatile volume changes of blood 
 * in the vascular system. 
 * 
 * Last Updated: Thu 1-May-2014 6:11 AM PST
 * 
 **********************/

 // include the library code:
 #include <Wire.h>
 #include <Adafruit_MCP23017.h>
 #include <Adafruit_RGBLCDShield.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// set backlight color presets
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// custom heart character
 byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

// initialize variables
unsigned long currentTime;
unsigned long lastUpdated;
unsigned long lastPeakTime=0;
const int samplingRate=10; // (Hz)
const int samplingPeriod=1e3/samplingRate; // (ms)
float HR=0;
float meanHR=0;
int numBeats=0;

uint8_t buttons;
int buttonState=10; // 10-startup, 0-just HR, 1-time, 2-meanHR, 3-LED modulation, 4-just O2 sat

const int inputPin=1;
const int pinLED=9;
const int pinIR=10;
const int bright_max=255;
const int bright_mod=25;
int bright_current=bright_max/2; 
int amplitudes[20]; // used for LED modulation. size is 2*samplingRate.

int lastValue=0; // last analogRead input
int thisValue=0; // current analogRead input
float firstDifference=0;

void setup(){
	// initialize I/O pins
	pinMode(inputPin,INPUT);
	pinMode(pinLED,OUTPUT);
	pinMode(pinIR,OUTPUT);

	// initialize serial monitor
	Serial.begin(9600);

	// initialize LCD shield
	lcd.begin(16,2);
	lcd.setBacklight(WHITE);
	lcd.clear();

	// create custom heart character
	lcd.createChar(0,heart);

	// delay before beginning loop
	delay(2500);
}

void loop(){

	// time when entered loop
	currentTime=millis(); 

	if (currentTime > lastUpdated+samplingPeriod) {
		// check current button states
		buttons = lcd.readButtons();
			
			if (!buttons){
				// Clear LCD screen if no buttons being pressed and previous state was NOT HR. buttonState = 0
				if (buttonState!=0) {lcd.clear();}

				int val = 0; // 0-only LED, 1-only IR, 2-alternating
				switch (val) {
				    case 0:
				    	// Serial.println("ONLY LED");
				    	analogWrite(pinLED,bright_current);
				    	analogWrite(pinIR,0);
				    	readSignal();
				    	detectBeats();
				    	break;

				    case 1:
				    	analogWrite(pinLED,0);
				    	analogWrite(pinIR,bright_current);
				    	readSignal();
				    	detectBeats();
				    	break;

				    case 2:

				    	break;
				}

				// update buttonState
				buttonState=0;

			} else if (buttons == BUTTON_DOWN){
				modulateLED();

			} else if (buttons == BUTTON_LEFT){
				// clear LCD screen if BUTTON_LEFT is pressed and previous state was NOT time elapsed. buttonState = 1
				if (buttonState!=1){lcd.clear();}
				
				// update LCD screen
				lcd.setCursor(0,0);	lcd.print("Time elapsed: ");
				lcd.setCursor(0,1);	lcd.print((unsigned int) millis()/1000); lcd.print("s");

				// update buttonState
				buttonState=1;

			} else if (buttons == BUTTON_RIGHT){
				// clear LCD screen if BUTTON_RIGHT is pressed and previous state was NOT mean HR
				if (buttonState!=2){lcd.clear();}

				// update LCD screen
				lcd.setCursor(0,0);	lcd.print("Mean HR (bpm): ");
				lcd.setCursor(0,1);	lcd.print(meanHR);

				// update buttonState
				buttonState=2; 

			} else if (buttons == BUTTON_UP){
				// clear LCD screen if BUTTON_UP is pressed and previous state was NOT o2 sat
				if (buttonState!=4){lcd.clear();}

				// calculate O2Sat
				float O2Sat = findO2Sat(); 

				// update LCD screen
				lcd.clear();
				lcd.setCursor(0,0); lcd.print("O2 Saturation: ");
				lcd.setCursor(0,1); lcd.print(O2Sat*100); lcd.print("%"); 

				delay(5000);

				// update buttonState
				buttonState=4;
			}

		// update time
		lastUpdated=currentTime;
	}
}


	

void readSignal(){
	// analogRead from input pin (range from 0 to 1023)
	lastValue=thisValue;
	thisValue=analogRead(inputPin); 
}

void detectBeats(){
	// update LCD screen
	lcd.setCursor(0,0); lcd.print("Heart Rate: ");

	// clear heart symbol to show "beats"
	lcd.setCursor(0,1); lcd.print(" ");

	// negative peak detection
	firstDifference=thisValue-lastValue;
	int cutoff = -50;

	// Serial.println(firstDifference);

	if (firstDifference < cutoff){
		HR=60e3/(millis()-lastPeakTime);
		
		// Clear LCD screen if no buttons being pressed and previous state was NOT HR. buttonState = 0
		if (!lcd.readButtons() && buttonState!=0){lcd.clear();}

		if (HR>0 && HR<250){
			lcd.setCursor(0,1); lcd.write(0);
			lcd.setCursor(2,1); lcd.print(HR); lcd.print(" bpm ");
			updateMeanHR();
		}

		lastPeakTime=currentTime;

	}

}

void updateMeanHR(){
	numBeats++;
	meanHR = numBeats / (millis()/60e3); // beats/ms to bpm
}

void modulateLED(){
	if (buttons == BUTTON_DOWN){
		// clear LCD screen if BUTTON_DOWN being pressed and previous state was NOT modulating LED. buttonState = 3
		if (buttonState!=3){lcd.clear();}

		// update LCD screen
		lcd.setCursor(0,0); lcd.print("Push again ");
		lcd.setCursor(0,1); lcd.print("to calibrate.");

		int exitCount=0;
		boolean exitStatus=false;

		while (buttons!=BUTTON_DOWN && exitCount<5){
			buttons=lcd.readButtons();
			delay(1000); 
			exitCount++;
			if (exitCount==5){exitStatus=true;}
		}

		// update LCD screen
		if (!exitStatus){
			// Calibration initialized. 
			lcd.setCursor(0,0); lcd.print("Calibrating...");
			lcd.setCursor(0,1); lcd.print("Do not move.");

			int maxAmplitude=0;
			int minAmplitude=1023;

			// collect 2 seconds of data and determine min and max amplitudes in that time.
			for (int k=0; k<samplingRate; k++){
				amplitudes[k]=analogRead(inputPin);
				if (maxAmplitude<amplitudes[k]){maxAmplitude=amplitudes[k];}
				if (minAmplitude>amplitudes[k]){minAmplitude=amplitudes[k];}
				delay(samplingPeriod);
			}

			if (maxAmplitude-minAmplitude<10 && bright_current<250){
				// difference in BP deflection less than 0.1V, increase LED brightness
				bright_current+=bright_mod;
			} else if (maxAmplitude-minAmplitude>10 && bright_current>=0){
				// difference in BP deflection is greater than 1V, decrease LED brightness
				bright_current-=bright_mod;
			}

			analogWrite(pinLED,bright_current);

			// Calibration successful.
			lcd.clear();
			lcd.setCursor(0,0); lcd.println("Calibration");
			lcd.setCursor(0,1); lcd.println("successful.");
		} else {
			// Calibration failed. 
			lcd.clear();
			lcd.setCursor(0,0); lcd.println("Calibration failed.");
		}
		
	}

	// update buttonState
	buttonState=3;
}

float findO2Sat(){	
	// duration of window to collect data
	unsigned int duration = 5000; // (ms)

	// LED Data
	analogWrite(pinLED,bright_current);
	analogWrite(pinIR,0);

	unsigned int startTime; 
	unsigned int endTime;
	float avgPeakValLED  = 0;
	float totPeakValLED  = 0;
	float avgPeakValLED2 = 0;
	float totPeakValLED2 = 0;

	float avgPeakValIR  = 0;
	float totPeakValIR  = 0;
	float avgPeakValIR2 = 0;
	float totPeakValIR2 = 0;

	int count=0;
	int count2=0;
	int cutoff = -35;

	lcd.clear();
	lcd.setCursor(0,0); lcd.print("Finding");
	lcd.setCursor(0,1); lcd.print("O2 Saturation");
	delay(1000);

	startTime = millis();
	endTime = startTime+duration;
	Serial.println("");
	Serial.println("BEGIN LED DATA COLLECTION.");
	Serial.print("Start time: "); Serial.println(startTime);
	Serial.print("duration: "); Serial.println(duration);
	Serial.print("current time: "); Serial.println(millis());
	Serial.print("collection end time: "); Serial.println(endTime);

	while(millis() < endTime){
		lcd.clear();
		lcd.setCursor(0,0); lcd.print("Collecting");
		lcd.setCursor(0,1); lcd.print("LED data. ");

		currentTime = millis();
		readSignal();

		// negative peak detection
		firstDifference=thisValue-lastValue;

		if (firstDifference < cutoff){
			count++; 
			totPeakValLED+=thisValue;
			avgPeakValLED=totPeakValLED/count;
		}

		// positive peak detection
		if (firstDifference > -1*cutoff){
			count2++;
			totPeakValLED2+=thisValue;
			avgPeakValLED2=totPeakValLED2/count2;
		}

		delay(samplingPeriod);
	}

	// IR Data
	count = 0; count2=0;
	startTime = millis();
	endTime=startTime+duration;
	analogWrite(pinLED,0);
	analogWrite(pinIR,bright_current);

	delay(1000);

	Serial.println("");
	Serial.println("BEGIN IR DATA COLLECTION.");
	Serial.print("Start time: "); Serial.println(startTime);
	Serial.print("duration: "); Serial.println(duration);
	Serial.print("current time: "); Serial.println(millis());
	Serial.print("collection end time: "); Serial.println(endTime);
        Serial.println("");

	while(millis() < endTime){
		lcd.clear();
		lcd.setCursor(0,0); lcd.print("Collecting");
		lcd.setCursor(0,1); lcd.print("IR data.");

		currentTime=millis();
		readSignal();

		// negative peak detection
		firstDifference=thisValue-lastValue;
		
		if (firstDifference<cutoff){
			count++;
			totPeakValIR+=thisValue;
			avgPeakValIR=totPeakValIR/count;
		}

		// positive peak detection
		if (firstDifference>-1*cutoff){
			count2++;
			totPeakValIR2+=thisValue;
			avgPeakValIR2=totPeakValIR2/count2;
		}

		delay(samplingPeriod);
	}

	analogWrite(pinLED,0);
	analogWrite(pinIR,bright_current);

	Serial.println("LED Data");
	Serial.print("Average max: "); Serial.println(avgPeakValLED2);
	Serial.print("Average min: "); Serial.println(avgPeakValLED);
	Serial.print("Vpp: "); Serial.println(avgPeakValLED2-avgPeakValLED);
	Serial.println("");

	Serial.println("IR Data");
	Serial.print("Average max: "); Serial.println(avgPeakValIR2);
	Serial.print("Average min: "); Serial.println(avgPeakValIR);
	Serial.print("Vpp: "); Serial.println(avgPeakValIR2-avgPeakValIR);
	Serial.println("");

	Serial.println("Ratio");
	Serial.println((avgPeakValLED2-avgPeakValLED) / (avgPeakValIR2-avgPeakValIR));
	Serial.println("");

	return (float) (avgPeakValLED2-avgPeakValLED) / (avgPeakValIR2-avgPeakValIR) * 0.5 + 0.7; 
}
