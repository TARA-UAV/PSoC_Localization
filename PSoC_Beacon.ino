/* 
 Written for an Arduino Pro Mini 5v 16MHz 328 
 Multiple speaker 40kHz beacon 
 Attach speakers to Arduino Digital Pin 5, 6, 7, 8, 9, and GND 
 Drives speakers with a 5V square wave at 40kHz, maximum 40mA
 
 Adding UART Rx to get 0xFF command from PSoC to trigger the speakers for a limited time.
 
 2016-11-13 SNR Use tone to generate wave for single sonar
                No RF check
 2016-11-13 SNR Tone with multiple sonars
*/ 


#define LEDPin 13 // Onboard LED pin
#define UARTRxConfirm 4// UART Confirmation LED
 

void setup() { 
  
  Serial.begin(115200);


  /* Pin Setup */ 
    pinMode(5, OUTPUT);
    pinMode(LEDPin, OUTPUT); // LED indicator set to output 
    pinMode(UARTRxConfirm, OUTPUT);
} 
 
 
void loop() { 
/* Each channel is driven with a square wave of 5V, 40mA maximum.  
   A pause is observed... then the next channel is driven */  
   
 char RF_Command = 'S';
 
   
   if (Serial.available() > 0) {
   char myData;
   myData = Serial.read();
   digitalWrite(LEDPin, HIGH); //LED on during each driving pulse

   
   if (myData==RF_Command) {  
   digitalWrite(UARTRxConfirm, HIGH);
   
    // Play 40 kHz tone
     tone(5,40000);
   
   // Play tone for 200 microseconds
   delayMicroseconds(200);
   
   // Stop tone
     noTone(5);
   
   // Wait for 500 millisecond
   delay(100);
 
   digitalWrite(UARTRxConfirm, LOW);  
   }// end of if - myData check
 
 digitalWrite(LEDPin, LOW); // LED off during pause between pulses  
 }// end of if - Serial available
 
} // end of loop
