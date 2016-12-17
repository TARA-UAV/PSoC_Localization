/* ================================================================================
* FILE: PSoC_Receiver.c
* 
* DESCRIPTION: Receiver code for sonar-based localization system.
* Sends a synchronization signal to initiate sonar pulse transmission by the beacon and 
* determines the time required for the pulse to be detected by each of the 3 receivers. 
* The beacon position is then estimated using trilateration.
*
* REVISIONS:
* 2016-11-06 SNR 
*	         Fixed trilateration calculations
*			 I2C full precision timestamp data 
* 2016-11-07 SNR
*			 Use LSB of timetable data which contains timestamp
* 2016-11-08 SNR
*            Timestamp data test
*			 bestStamp includes uL
* 2016-11-10 SNR
*            Revert timestamp & bestStamp
*            Use string for RF_UART, not char
* 2016-11-13 SNR
*            Add sphereRadius variable and send to Axon
*			 Updated SPEED_SOUND from 340 to 340.29
* 2016-11-14 SNR
*            Updating code to fix trilateration to match Matlab
* 2016-12-05 SNR
*            Sonar data filter created
*            Added Xbee delay to remove from timestamp   
* ================================================================================
*/
#include <project.h>
#include <cydevice.h>
#include <math.h>
#include <stdio.h>

void dmaConfigure(void);
void refreshDMA(void);
void stampFilter(int32 line[][3], double weights[], int32 output[]);
void setupOLED(void);
void commandOLED(uint8 comm);
void dataOLED(uint8 data);
void textOLED32(uint8 text[]);

#define PEAKS_TO_CAPTURE_PER_CHANNEL 1
#define CH_SIZE 2 * PEAKS_TO_CAPTURE_PER_CHANNEL // The number of entries needed in the channel arrays.
#define LOCATION_LINE_LENGTH 10 // Length of location line for velocity filter.

uint16 destCH0[CH_SIZE] = {0};
uint16 destCH1[CH_SIZE] = {0};
uint16 destCH2[CH_SIZE] = {0};

uint8 stampControl_stampCH_0_chDMA_Chan;
uint8 stampControl_stampCH_0_chDMA_TD[1];
uint8 stampControl_stampCH_1_chDMA_Chan;
uint8 stampControl_stampCH_1_chDMA_TD[1];
uint8 stampControl_stampCH_2_chDMA_Chan;
uint8 stampControl_stampCH_2_chDMA_TD[1];
uint8 RF_DMA_Chan;
uint8 RF_DMA_TD[1];


char RF_Command = 'S';
uint8 RF_Address = 0xFF;
uint8 bufferOLED[2] = {0};
uint8 stringOLED[32] = {"cm X  | Y  | Z  > VERIFICATION <"};
uint8 const slaveOLED = 0b0111100;

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
START OF MAIN PROGRAM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

int main()
{


	// Trilateration variables
	double const p1[3] = {0, 0, 0}; 
	double const p2[3] = {0.76, 0, 0};
	double const p3[3] ={0.38, 0.38, 0};

	// Receiver co-ordinates (x,y,z) relative to CHO
	double const rxPosition[3][3] = { { 0.0, 0.0,  0.0},
									{ 0.0, 0.76, 0.0},    
									{ 0.0, 0.38, 0.38}}; 

	double const SPEED_SOUND = 340.29; // Speed of sound for calculation
	double const sampleCLK = 1000000.0; // Sampling clock frequency
	double const PI_VALUE = 3.14159;

	double newLocation[3] = {0}; // Calculated position
	double newVelocity[3]= {0};  // Calculated velocity
	double newAngle[2] = {0};
	double newRadius = 0;

	// Filter variables
	double newLocationPrev[3] = {0};
	double sphereRadiusPrev[3] = {0}; 
	double sphereRadiusPrev2[3] = {0};

	// Communication variables
	int8 bufferI2C[17] = {0};                    // I2C Buffer visible to PixHawk
	uint8 termUART[30] = {'x','=',' ','0','.','2','2','2',',',' ',
		'y','=',' ','0','.','2','2','2',',',' ',
		'z','=',' ','0','.','2','2','2',13,10};               
	int8 octUART[6] = {0};
	//int8 axonUART[9] = {0}; //SNR
	int axonUART[9] = {0}; //SNR
	uint8 testUART[63] =  {' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ',
		' ',' ',' ',' ',' ',' ','_',13,10};

	double sphereRadius[3] = {0};
	double sphereRadiusSquared[3] = {0};

	int8 state = 0; 
	uint8 validChannel = 0; // DMA completion status
	uint8 i,j;                                            
	int32 timeTable[PEAKS_TO_CAPTURE_PER_CHANNEL][3]; // Place to store the time stamps
	int32 softwareDelay = 4015; //4000(xbee)+15(software) clock cycles to subtract from timestamps.
	int32 bestStamp[3]; // Final calculated stamp
	uint8 inactiveCount = 0;
	uint8 thermalMode = 0;
	uint8 modeOLED = 0;
	uint8 uartFlag = 0;
	uint8 uartMode = 5;
	uint8 volatile alertRegister = 0;
	float thermMeasured;
	char axonString[35];
	uint8 axonBytesWritten;

	CyGlobalIntEnable; // Enable global interrupts

	dmaConfigure(); // Configure the DMA channels for the 5 RX. Also starts FIFOs
	ClockSample_Start();  // Start the 12MHz clock 

	//############################### Initializations ##################################################################
	I2C_Start();                                // Start the I2C Interface
	I2C_SetBuffer1(12, 0, &bufferI2C);          // Set the accessible read area for the I2C interface
	I2C_Local_Start();


	UART_Start(0, UART_5V_OPERATION);           //USB data
	//Init UART_Axon
	UART_Axon_Start();

	if (modeOLED) {
		setupOLED();
		stringOLED[6] = 0x2Au;
		stringOLED[11] = 0x2Au;
		textOLED32(stringOLED);
	}

	if (thermalMode) {
		thermADC_Start();
		CyDelay(10);
		thermADC_StartConvert();
		CyDelay(10);
		if (thermADC_IsEndConversion(thermADC_WAIT_FOR_RESULT)) {
			thermMeasured = (thermADC_CountsTo_mVolts(thermADC_GetResult16()) - 400.0)/19.5;
			SPEED_SOUND = 331.4 + 0.6 * thermMeasured;   
		}
		thermADC_StopConvert();
		thermADC_Stop();
	}

	LEDReg_Write(0x04u);
	
	RF_UART_Start();

	softRST_Write((uint8)0x1u); //This is the default state of the register anyway.
	CyDelay((uint32)0xFFu); //Wait 255 ms to ensure the hardware is in a known state (the reset state)    
	softRST_Write((uint8)0x0u); // Pull the system out of reset

	RF_UART_PutChar(RF_Command);

	//############################### ForEver Loop Start ###########################################################
	for(;;)
	{
		alertRegister = stampControl_validReg_Read();
		validChannel = alertRegister | validChannel;

		if ((validChannel & 0x07u) == 0x07u) { //All channel DMAs have finished -> times captured. Timer reset regardless of whether it has finished.
			inactiveCount = 0;
			softRST_Write((uint8)0x1u); // Put hardware in reset. Note that when the timer has expired nothing more can happen anyway.
			refreshDMA(); // Clear the FIFOs and reset the DMA to their original state. The DMAs should be disabled - this function re-enables them.
			memset(timeTable, 0, sizeof(timeTable)); // Clear the timeTable

			i = 0; // Find the peak time stamp and place in register in one move.
			for (i = 0; i < PEAKS_TO_CAPTURE_PER_CHANNEL; i++) {
				timeTable[i][0] =  (int32)(0.5 * ((int32)destCH0[2*i] + (int32)destCH0[2*i+1]) - softwareDelay);
				timeTable[i][1] =  (int32)(0.5 * ((int32)destCH1[2*i] + (int32)destCH1[2*i+1]) - softwareDelay); 
				timeTable[i][2] =  (int32)(0.5 * ((int32)destCH2[2*i] + (int32)destCH2[2*i+1]) - softwareDelay); 

			}

			memset(destCH0, 0, sizeof(destCH0)); // Clear destination array
			memset(destCH1, 0, sizeof(destCH1)); // Clear destination array
			memset(destCH2, 0, sizeof(destCH2)); // Clear destination array

			softRST_Write((uint8)0x0u); // Take hardware out of reset

			memset(bestStamp, 0, sizeof(bestStamp)); // Clear the deltaT array  

			// Update filter variables
			for (i = 0; i < 3; i++) {
				sphereRadiusPrev2[i] = sphereRadiusPrev[i];
				sphereRadiusPrev[i] = sphereRadius[i];
				newLocationPrev[i] = newLocation[i];
			}

			for (i = 0; i < 3; i++) {
				bestStamp[i] = timeTable[0][i];
				//Calculate the radius of each sphere
				sphereRadius[i] = (SPEED_SOUND * bestStamp[i] / sampleCLK);
				sphereRadiusSquared[i] = (sphereRadius[i] * sphereRadius[i]);
			} 


			// Determine beacon location
			newLocation[1] = // Code redacted for confidentiality
			newLocation[2] = // Code redacted for confidentiality
			newLocation[0] = // Code redacted for confidentiality


			// *****************************************************************
			//                   Sonar data filter
			// *****************************************************************   
			// Code redacted for confidentiality


			// *****************************************************************
			//                   Calculate the angles
			// *****************************************************************    
			if (newLocation[0] >0.05) {
				newAngle[0] = atan(newLocation[1]/newLocation[0])*180.0/PI_VALUE;
				newAngle[1] = atan(newLocation[2]/newLocation[0])*180.0/PI_VALUE;
				newRadius = sqrt(sphereRadiusSquared[0]);
			}

			// *****************************************************************
			//                    Determine State
			// *****************************************************************
			state = 3;

			if (I2C_GetActivity() != I2C_STATUS_BUSY) {
				bufferI2C[0] = (int)newLocation[0]; //Location XYZ
				bufferI2C[1] = (int)newLocation[1];
				bufferI2C[2] = (int)newLocation[2];
				bufferI2C[3] = (int)round(100.0*(newLocation[0] - (double)bufferI2C[0]));
				bufferI2C[4] = (int)round(100.0*(newLocation[1] - (double)bufferI2C[1]));
				bufferI2C[5] = (int)round(100.0*(newLocation[2] - (double)bufferI2C[2]));

				bufferI2C[6] = (int)newVelocity[0]; //Velocity XYZ
				bufferI2C[7] = (int)newVelocity[1];
				bufferI2C[8] = (int)newVelocity[2];
				bufferI2C[9] = (int)round(100.0*(newVelocity[0] - (double)newVelocity[0]));
				bufferI2C[10] = (int)round(100.0*(newVelocity[1] - (double)newVelocity[1]));
				bufferI2C[11] = (int)round(100.0*(newVelocity[2] - (double)newVelocity[2]));

				bufferI2C[12] = (int)newAngle[0]; //Angle azimuth, elevation
				bufferI2C[13] = (int)newAngle[1];
				bufferI2C[14] = (int)round(100.0*(newAngle[0] - (double)newAngle[0]));
				bufferI2C[15] = (int)round(100.0*(newAngle[1] - (double)newAngle[1]));

				bufferI2C[16] = state;

			}

			// *****************************************************************
			//                    UART SECTION
			// *****************************************************************
			//Forming the PSOC Position Data String
			axonUART[0] = (int)newLocation[0];
			axonUART[1] = (int)newLocation[1];
			axonUART[2] = (int)newLocation[2];
			axonUART[3] = (int)round(100.0*(newLocation[0] - (double)axonUART[0]));
			axonUART[4] = (int)round(100.0*(newLocation[1] - (double)axonUART[1]));
			axonUART[5] = (int)round(100.0*(newLocation[2] - (double)axonUART[2]));

			// sphereRadius data 
			axonUART[6] = (int)(sphereRadius[0]*100.0);
			axonUART[7] = (int)(sphereRadius[1]*100.0);
			axonUART[8] = (int)(sphereRadius[2]*100.0);


			axonBytesWritten = sprintf(axonString,"$PSOC,%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d,E\n",axonUART[0],axonUART[3],axonUART[1],axonUART[4],axonUART[2],axonUART[5],axonUART[6],axonUART[7],axonUART[8]);
			UART_Axon_PutString(axonString);




			if(UART_bGetConfiguration()) {
				//Check if UART is (was) configured. If not, initialize it.
				if (!uartFlag) {
					UART_CDC_Init();
					uartFlag = 1;
				} else if (UART_CDCIsReady()) { //The code only executed if the UART is configured and ready to send more info.
					switch(uartMode) {
							case 1:
								//Nothing happens as this is mode 1 (Non debug)
								break;
							case 2:
								//Output location in terminal readable form
								sprintf(&termUART[2], "%+6.3f", newLocation[0]);
								sprintf(&termUART[12], "%+6.3f", newLocation[1]);
								sprintf(&termUART[22], "%+6.3f", newLocation[2]);
								termUART[28] = 13;
								UART_PutData(&termUART, 30);
								break;
							case 3:
								//Output in form compatible with Octave program
								octUART[0] = (int)newLocation[0];
								octUART[1] = (int)newLocation[1];
								octUART[2] = (int)newLocation[2];
								octUART[3] = (int)round(100.0*(newLocation[0] - (double)octUART[0]));
								octUART[4] = (int)round(100.0*(newLocation[1] - (double)octUART[1]));
								octUART[5] = (int)round(100.0*(newLocation[2] - (double)octUART[2]));
								UART_PutData(&octUART, 6);
								break;
							case 4:
								//Output location, state, and timestamps for test purposes
								sprintf(&testUART[0], "%5.2f", newAngle[0]);
								sprintf(&testUART[5], "%5.2f", newAngle[1]);
								sprintf(&testUART[11], "%5.2f", newRadius);
								sprintf(&testUART[16], "%6u", (uint16)timeTable[0][0]);
								sprintf(&testUART[22], "%6u", (uint16)timeTable[0][1]);
								sprintf(&testUART[28], "%6u", (uint16)timeTable[0][2]);
								sprintf(&testUART[46], "%4.1f", newVelocity[0]);
								sprintf(&testUART[50], "%2u", state);
								testUART[61]=13;
								UART_PutData(&testUART, 63);
								break;

					}//End of Switch
				}//End of else if (UART_CDCIsReady())
			}//End of if(UART_bGetConfiguration())

			// *****************************************************************
			//                    DISPLAY SECTION
			// *****************************************************************
			if (modeOLED == 2) {
				sprintf(&stringOLED[16], "%6.0f", 100*newLocation[0]);
				sprintf(&stringOLED[23], "%4.0f", 100*newLocation[1]);
				sprintf(&stringOLED[28], "%4.0f", 100*newLocation[2]);

				stringOLED[22] = ' ';
				stringOLED[27] = ' ';

				textOLED32(stringOLED);
			}
			// *****************************************************************
			//                    LED SECTION
			// *****************************************************************
			//RED SOLID - Data available since last calculation
			LEDReg_Write(0x03u);
			validChannel = 0;
			CyDelay(100);
			//RF_UART_PutChar(RF_Address);
			RF_UART_PutChar(RF_Command); //SNR


		} else if ((validChannel & 0x08u) == 0x08u) { //tc has been reached without all timestamps captured - reset hardware.
			inactiveCount = 0;
			softRST_Write((uint8)0x1u);                     // Put hardware in reset. Note that when the timer has expired nothing more can happen anyway.
			refreshDMA();                                   // Clear the FIFOs and reset the DMA to their original state. The DMAs should be disabled - this function re-enables them.
			memset(destCH0, 0, sizeof(destCH0));    // Clear destination array
			memset(destCH1, 0, sizeof(destCH1));    // Clear destination array
			memset(destCH2, 0, sizeof(destCH2));    // Clear destination array

			softRST_Write((uint8)0x0u);                     // Take hardware out of reset.
			LEDReg_Write(0x01u);
			validChannel = 0;
			CyDelay(55);
			//RF_UART_PutChar(RF_Address);
			RF_UART_PutChar(RF_Command); //SNR

		} 
		else {
			CyDelay(1);
			inactiveCount++;
			if (inactiveCount > 99) {
				//RF_UART_PutChar(RF_Address);
				RF_UART_PutChar(RF_Command); //SNR
				inactiveCount = 0;
				validChannel = 0;
			}
		}

	} //End of Forever Loop
} //End of Main


/* ================================================================================
* FUNCTION: dmaConfigure
* DESCRIPTION: Configures DMA for stamp control
* INPUT: None
* OUTPUT: None
* ================================================================================*/
void dmaConfigure(void) {

	/* Defines for stampControl_stampCH_0_chDMA */
	#define stampControl_stampCH_0_chDMA_BYTES_PER_BURST 2
	#define stampControl_stampCH_0_chDMA_REQUEST_PER_BURST 1
	#define stampControl_stampCH_0_chDMA_SRC_BASE (CYDEV_PERIPH_BASE)
	#define stampControl_stampCH_0_chDMA_DST_BASE (CYDEV_SRAM_BASE)

	/* DMA Configuration for stampControl_stampCH_0_chDMA */
	stampControl_stampCH_0_chDMA_Chan = stampControl_stampCH_0_chDMA_DmaInitialize(stampControl_stampCH_0_chDMA_BYTES_PER_BURST, stampControl_stampCH_0_chDMA_REQUEST_PER_BURST, 
		HI16(stampControl_stampCH_0_chDMA_SRC_BASE), HI16(stampControl_stampCH_0_chDMA_DST_BASE));
	stampControl_stampCH_0_chDMA_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(stampControl_stampCH_0_chDMA_TD[0], stampControl_stampCH_0_chDMA_BYTES_PER_BURST*CH_SIZE, CY_DMA_DISABLE_TD, stampControl_stampCH_0_chDMA__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(stampControl_stampCH_0_chDMA_TD[0], LO16((uint32)stampControl_stampCH_0_FIFO_FIFO16_PTR), LO16((uint32)&destCH0));
	CyDmaChSetInitialTd(stampControl_stampCH_0_chDMA_Chan, stampControl_stampCH_0_chDMA_TD[0]);
	CyDmaChEnable(stampControl_stampCH_0_chDMA_Chan, 1);

	/* Defines for stampControl_stampCH_1_chDMA */
	#define stampControl_stampCH_1_chDMA_BYTES_PER_BURST 2
	#define stampControl_stampCH_1_chDMA_REQUEST_PER_BURST 1
	#define stampControl_stampCH_1_chDMA_SRC_BASE (CYDEV_PERIPH_BASE)
	#define stampControl_stampCH_1_chDMA_DST_BASE (CYDEV_SRAM_BASE)

	/* DMA Configuration for stampControl_stampCH_1_chDMA */
	stampControl_stampCH_1_chDMA_Chan = stampControl_stampCH_1_chDMA_DmaInitialize(stampControl_stampCH_1_chDMA_BYTES_PER_BURST, stampControl_stampCH_1_chDMA_REQUEST_PER_BURST, 
		HI16(stampControl_stampCH_1_chDMA_SRC_BASE), HI16(stampControl_stampCH_1_chDMA_DST_BASE));
	stampControl_stampCH_1_chDMA_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(stampControl_stampCH_1_chDMA_TD[0], stampControl_stampCH_1_chDMA_BYTES_PER_BURST*CH_SIZE, CY_DMA_DISABLE_TD, stampControl_stampCH_1_chDMA__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(stampControl_stampCH_1_chDMA_TD[0], LO16((uint32)stampControl_stampCH_1_FIFO_FIFO16_PTR), LO16((uint32)&destCH1));
	CyDmaChSetInitialTd(stampControl_stampCH_1_chDMA_Chan, stampControl_stampCH_1_chDMA_TD[0]);
	CyDmaChEnable(stampControl_stampCH_1_chDMA_Chan, 1);

	/* Defines for stampControl_stampCH_2_chDMA */
	#define stampControl_stampCH_2_chDMA_BYTES_PER_BURST 2
	#define stampControl_stampCH_2_chDMA_REQUEST_PER_BURST 1
	#define stampControl_stampCH_2_chDMA_SRC_BASE (CYDEV_PERIPH_BASE)
	#define stampControl_stampCH_2_chDMA_DST_BASE (CYDEV_SRAM_BASE)

	/* DMA Configuration for stampControl_stampCH_2_chDMA */
	stampControl_stampCH_2_chDMA_Chan = stampControl_stampCH_2_chDMA_DmaInitialize(stampControl_stampCH_2_chDMA_BYTES_PER_BURST, stampControl_stampCH_2_chDMA_REQUEST_PER_BURST, 
		HI16(stampControl_stampCH_2_chDMA_SRC_BASE), HI16(stampControl_stampCH_2_chDMA_DST_BASE));
	stampControl_stampCH_2_chDMA_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(stampControl_stampCH_2_chDMA_TD[0], stampControl_stampCH_2_chDMA_BYTES_PER_BURST*CH_SIZE, CY_DMA_DISABLE_TD, stampControl_stampCH_2_chDMA__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(stampControl_stampCH_2_chDMA_TD[0], LO16((uint32)stampControl_stampCH_2_FIFO_FIFO16_PTR), LO16((uint32)&destCH2));
	CyDmaChSetInitialTd(stampControl_stampCH_2_chDMA_Chan, stampControl_stampCH_2_chDMA_TD[0]);
	CyDmaChEnable(stampControl_stampCH_2_chDMA_Chan, 1);


	stampControl_stampCH_0_FIFO_Start();
	stampControl_stampCH_1_FIFO_Start();
	stampControl_stampCH_2_FIFO_Start();

}

/* ================================================================================
* FUNCTION: refreshDMA
* DESCRIPTION: Reset DMA for stamp control
* INPUT: None
* OUTPUT: None
* ================================================================================*/
void refreshDMA(void) {
	stampControl_stampCH_0_FIFO_ClearFIFO();
	stampControl_stampCH_1_FIFO_ClearFIFO();
	stampControl_stampCH_2_FIFO_ClearFIFO();

	CyDmaChEnable(stampControl_stampCH_0_chDMA_Chan, 1);
	CyDmaChEnable(stampControl_stampCH_1_chDMA_Chan, 1);
	CyDmaChEnable(stampControl_stampCH_2_chDMA_Chan, 1);

}

/* ================================================================================
* FUNCTION: setupOLED
* DESCRIPTION: OLED setup
* INPUT: None
* OUTPUT: None
* ================================================================================*/
void setupOLED(void) {


	OLEDReg_Write(0x01u); //Release Reset
	CyDelay(10);
	commandOLED(0x2A);  //function set (extended command set)
	commandOLED(0x71);  //function selection A, disable internal Vdd regualtor
	dataOLED(0x5C);
	commandOLED(0x28);  //function set (fundamental command set)
	commandOLED(0x08);  //display off, cursor off, blink off
	commandOLED(0x2A);  //function set (extended command set)
	commandOLED(0x79);  //OLED command set enabled
	commandOLED(0xD5);  //set display clock divide ratio/oscillator frequency
	commandOLED(0x70);  //set display clock divide ratio/oscillator frequency
	commandOLED(0x78);  //OLED command set disabled
	commandOLED(0x08);  //extended function set (4-lines)
	commandOLED(0x06);  //COM SEG direction
	commandOLED(0x72);  //function selection B, disable internal Vdd regualtor
	dataOLED(0x00);     //ROM CGRAM selection
	commandOLED(0x2A);  //function set (extended command set)
	commandOLED(0x79);  //OLED command set enabled
	commandOLED(0xDA);  //set SEG pins hardware configuration
	commandOLED(0x10);  //set SEG pins hardware configuration   ////////////////////////////////////0x10 on other slim char OLEDs
	commandOLED(0xDC);  //function selection C
	commandOLED(0x00);  //function selection C
	commandOLED(0x81);  //set contrast control
	commandOLED(0x7F);  //set contrast control
	commandOLED(0xD9);  //set phase length
	commandOLED(0xF1);  //set phase length
	commandOLED(0xDB);  //set VCOMH deselect level
	commandOLED(0x40);  //set VCOMH deselect level
	commandOLED(0x78);  //OLED command set disabled
	commandOLED(0x28);  //function set (fundamental command set)
	commandOLED(0x01);  //clear display
	commandOLED(0x80);  //set DDRAM address to 0x00
	commandOLED(0x0C);  //display ON
	CyDelay(100);
}

/* ================================================================================
* FUNCTION: commandOLED
* DESCRIPTION: Write OLED command through I2C
* INPUT: comm - the command
* OUTPUT: None
* ================================================================================*/
void commandOLED(uint8 comm) {
	bufferOLED[0] = 0x00;
	bufferOLED[1] = comm;
	I2C_Local_MasterClearStatus();
	I2C_Local_MasterWriteBuf(slaveOLED, bufferOLED, 2, I2C_Local_MODE_COMPLETE_XFER);
	while(!(I2C_Local_MasterStatus() & I2C_Local_MSTAT_WR_CMPLT)){};
}

/* ================================================================================
* FUNCTION: dataOLED
* DESCRIPTION: Write OLED data through I2C
* INPUT: comm - the data
* OUTPUT: None
* ================================================================================*/
void dataOLED(uint8 data) {
	bufferOLED[0] = 0x40;
	bufferOLED[1] = data;
	I2C_Local_MasterClearStatus();
	I2C_Local_MasterWriteBuf(slaveOLED, bufferOLED, 2, I2C_Local_MODE_COMPLETE_XFER);
	while(!(I2C_Local_MasterStatus() & I2C_Local_MSTAT_WR_CMPLT)){};
}

/* ================================================================================
* FUNCTION: textOLED32
* DESCRIPTION: Display text
* INPUT: text - the text
* OUTPUT: None
* ================================================================================*/
void textOLED32(uint8 text[]){
	commandOLED(0x01);  //clear display
	commandOLED(0x02);  //Return Home
	CyDelay(2);
	commandOLED(0x80);  //set DDRAM address to 0x00 (First row)
	uint8 i = 0;
	for (i = 0; i < 16; i++) {
		dataOLED(text[i]);
		//dataOLED(0x1F);
	}
	commandOLED(0xC0);  //set DDRAM address to 0x40 (Second row)
	i = 16;
	for (i = 16 ; i < 32; i++) {
		dataOLED(text[i]);
	}

	CyDelay(100);
}

/* [] END OF FILE */
