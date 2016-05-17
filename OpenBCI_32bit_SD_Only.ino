/* derniere modif 18/01/2016
 *
 *  >>>> THIS CODE USED TO STREAM OpenBCI V3_32 DATA TO DONGLE <<<<
 *  >>>> SUPPORTS 16 CHANNELS WITH DAISY MODULE EXPANSION CARD <<<<
 * 2000Hz
 * This code is written to target a PIC32MX250F128B with UDB32-MX2-DIP bootloader
 * To Program, user must manually reset the PIC32 on the OpenBCI 32bit Board
 * press RST, then press PROG, then release RST, then release PROG
 * Adjust as needed if you are testing on different hardware.
 *
 * Use the custom libraries that should accompany this code.
 * download and place the libraries inside your: User/Documents/Arduino/libraries folder.
 * 
 * You need the latest version of the chipKIT-core hardware files. 
 * Go to the chipKIT website http://chipkit.net/ 
 * or directly to their core Wiki http://chipkit.net/wiki/index.php?title=ChipKIT_core
 * and follow the directions to install chipKIT board files into the latest Arduino IDE.
 * Then, when you open up Arduino IDE, you can select 'OpenBCI 32' from the boards menu!
 *
 * Any SDcard code is based on RawWrite example in SDFat library
 * 
 *         Be careful some "good" SD cards are not accepted by the OBCI Card.
 *
 *
 
 *      This version, called OpenBCI_32_SD_Only has added features to optionally :
 *
 *      - Log data to the SD card only. 
 *        To do this, send an 'N' over the serial port.
 *       
 *      - Toggle between hex or binary format for the SD data. 
 *        To do this, send a single 'B' over the serial port.
 *                 extention of the created files will be then 'bin' or 'hex'.
 *                 bin format yields 32 bytes by sample ,  hex format is estimated to 80 bytes...
 *                 binary raw data from the A/D are 8 24bits signed Big Endian,  
 *                 auxiliary data from the Pic32 are 4 16 bits Litlle endian...
 *                 to get auxiliary data (or anything else) in the 4 16 words you need to customize this firmware.
 *       
 *      - Set the sample rate to 250, 500, 1000, 2000Hz. 
 *        To do this, send an '.' followed by a char in '1' to '4' corresponding to 250..2000.
 *        ex : '.2' sets to 500Hz
 *
 *      - Define the duration of the record session. 
 *        To do this, send an 'hxxhxx>' .
 *        ex : 'h1h>' 	  sets to 1 hour
 *             'h2h30>'	  sets to 2 hours 30 minutes.
 *             'hh5>'     sets to          5 minutes.
 *             'hh99h'	  sets to 1 hour  39 minutes.
 *             'hh125>'   sets to 2 hours  5 minutes.
 *             'hh1234>'  sets to 2 hours 34 minutes!!
 *             'hh12345>' sets to 7 hours 45 minutes!!! The code is minimalist, so stay inside ' xxhxx '
 *         this command echoes 
 *             - the duration formated as 'xxhxx'
 *             - the corresponding closest number of blocks according to the current sample rate and the recording format (hex/bin)
 *
 *      - Define the maximum size of the file created on the SD card only. 
 *        To do this, send an 'mn>' , n = number of 10000 Blocks; a block = 512 chars ; '>' to end the command.
 *        ex : 'm100>' sets the maximum size to 100 x 10000 x 512 Chars equals 512M bytes
 *             'm0>'   sets the size to   512 Blocks.
 *             'm1>'   sets the size to 10000 Blocks.
 *          If the recording session duration corresponds to more blocks then new files are incremently created until the end of the session.
 *          Take in account that during file Creating/opening which duration is 1 to 2 seconds, acquired data are lost...
 * 
 *      - Define the starting number nnnn of the SD file name which format is FI_nnnn . 
 *        To do this, send an 'n' , followed by a char in '0' to '9' corresponding to 0000..9000.
 *        ex : 'n0' sets the first file name to FI_0000.bin/hex
 *             'n1' sets the first file name to FI_1000.bin/hex
 *          The idea was to not have identical file name when working with several stations
 *          On reflexion it can be different and improved...
 *
 *      - Last but not least, overruns caused by busy states of the SD interface have been overcome using pooling of the A/D converter and a circular buffer.
 *        When AD convertion is completed reading of a hard counter of the Pic32 constitutes a "time stamp" stored in the first 16 bits word of the auxiliary field.
 *        The clock period that increments this  hard counter is 256x25 nano seconds.  
 *        At a sample rate of 2kHz , the difference between two successive reading is equal to around 78 that corresponds to 0.4992 micro seconds.
 *        This way one can verify if overruns occured or not.
 *
 * ASCII commands are received on the serial port to configure and control
 * Serial protocol uses '+' immediately before and after the command character
 * We call this the 'burger' protocol. the '+' are the buns. Example:
 * To begin streaming data, send '+b+'
 * The OpenBCI Dongle firmware takes care of the burger protocol
 *
 * This software is provided as-is with no promise of workability
 * Use at your own risk, wysiwyg.

 * Made by Joel Murphy, Luke Travis, Conor Russomanno Summer, 2014. ;  y.j. 2015
 */

#include <OBCI32_SD.h>
#include <DSPI.h>
#include <EEPROM.h>
#include "OpenBCI_32_Daisy.h"


//------------------------------------------------------------------------------
//  << SD CARD BUSINESS >>
boolean		 Bin_format = true ; // true ;
boolean      SDfileOpen = false;
uint32_t 	 SessionSize = 0 ;
uint32_t	 SessionTime = 120 ;	// 120 minutes
char         fileSize = '0';  // SD file size indicator
int		 	 max_block_count = 512 ;   // voir EEPROM

unsigned int blockCounter = 0;
boolean      writeToSDonly = false;
boolean      SDcardWriteComplete = false;
byte         SDsampleCounter = 0;

//------------------------------------------------------------------------------
//  << OpenBCI BUSINESS >>
boolean is_running = false;    // this flag is set in serialEvent on reciept of ascii prompt
OpenBCI_32_Daisy OBCI;         //Uses SPI bus and pins to say data is ready.

// these are used to change individual channel settings from PC
char currentChannelToSet;      // keep track of what channel we're loading settings for
boolean getChannelSettings = false; // used to receive channel settings command
int channelSettingsCounter;    // used to retrieve channel settings from serial port
int leadOffSettingsCounter;
boolean getLeadOffSettings = false;
// these are all subject to the radio requirements: 31byte max packet length (maxPacketLength - 1 for packet checkSum)
#define OUTPUT_NOTHING (0)        // quiet
#define OUTPUT_8_CHAN (1)         // not using Daisy module
#define OUTPUT_16_CHAN (2)        // using Daisy module
int outputType = OUTPUT_8_CHAN;   // default to 8 channels

uint8_t Sample_Rate_SD_Only = 0b110 ;  // 250Hz
//------------------------------------------------------------------------------
//  << LIS3DH Accelerometer Business >>
boolean addAccelToSD = false;   // this flag get's set and cleared in the code below
//------------------------------------------------------------------------------
//  << PUT FILTER STUFF HERE >>
boolean useFilters = false;     // add DSP if you like, and set this true to turn them on
//------------------------------------------------------------------------------
//  << BASIC BOARD STUFF >>
int LED = 11;     // alias for the blue LED
int PGCpin = 12;  // PGC pin goes high when PIC is in bootloader mode
//------------------------------------------------------------------------------
//  << AUXILIARY DATA STUFF >>
boolean addAuxToSD = false;     // use this to write auxiliary data to SD if you like

// byte boardChannelDataRaw[24]; 

//------------------------------------------------------------------------------
void setup(void) 
{

  Serial0.begin(115200);  // using hardware uart number 0
  pinMode(LED, OUTPUT); digitalWrite(LED,HIGH);    // blue LED
  pinMode(PGCpin,OUTPUT); digitalWrite(PGCpin,LOW);// used to tell RFduino if we are in bootloader mode NOT IMPLEMENTED
  delay(1000);            // take a break

  OBCI.Sample_Rate    = 0b110 ;
  Sample_Rate_SD_Only = 0b110 ;
  max_block_count  =   512 ;
  { uint8_t i ; 
  i = EEPROM.read(2); if ( 0b011 <= i && i <= 0b110  ) Sample_Rate_SD_Only = i       ; 		// if (value red from the eeprom is coherent) ...
  i = EEPROM.read(3); if (     0 <= i                )   max_block_count   = i*10000 ;      // 500MB       900.000  8 heures a 500 hz
  } ; 
	
  startFromScratch();     // initialize OpenBCI, read device IDs
  // you can set EITHER useAccel or useAux to true
  // if you want both, you MUST set and clear one of the variables every sample
  OBCI.useAccel = true;      // option to add/remove accelerometer data to stream
  OBCI.useAux   = false;     // option to add/remove auxiliary data to stream
  
// ===== yj == input capture stuff========= data ready falling edge captures a 16bits counter ==========
//   this counter value can be written on te SD and used as time stamp to verify is some overrun occured or not...

//      T3CONbits.ON = 0;

/*
#define PPSUnlock {SYSKEY=0x0; SYSKEY=0xAA996655; SYSKEY=0x556699AA; CFGCONbits.IOLOCK=0;            } 
#define PPSLock   {SYSKEY=0x0; SYSKEY=0xAA996655; SYSKEY=0x556699AA; CFGCONbits.IOLOCK=1; SYSKEY=0x0;}

   PPSUnlock;
    INT4R=0; //assign pin 9 (RA0) to external interrupt 4 
   PPSLock;
*/

T2CONbits.TON    = 0 ; // Stop any 16/32-bit Timer2 operation
T3CONbits.TON    = 0 ; // Stop any 16-bit Timer3 operation
T2CONbits.T32    = 0 ; // =1; Enable 32-bit Timer mode
T3CONbits.TCS    = 0 ; // Select internal instruction cycle clock
T3CONbits.TGATE  = 0 ; // Disable Gated Timer mode
T3CONbits.TCKPS  = 7 ; // Select 1:256 Prescaler

TMR3 = 0x00;       // Clear 32-bit Timer       (lsw) 4 et (msw) 5
PR3  = 0xffff; // Load 32-bit period value (lsw) 4 et (msw) 5 ffff

	IC4CON          &= 0x7fff ;	 // Disable and reset module, disable clocks, disable interrupt generation and allow SFR modifications
	IC4CONbits.ICM   =      0 ;  // 010 = capture falling edge 
	IC4CONbits.C32   =      0 ;  // use 16 bit=0 ;32=1
	IC4CONbits.ICTMR =      0 ;  // use T3=0 ; t2=1
																// IC1CONbits.FEDGE = 1;  // capture rising edge first all others after ; inutile si ICM = 010 = On falling edge
	IC4CONbits.ICM   =      2 ;  // 010 = capture falling edge 
																// IC1CONbits.ICI   = 0;  // interrupt on every  capture event
																// IPC1bits.ICOV    = 0;  // 0 = No input capture overflow occurred
																// IPC1bits.IC1IP   = 3;  // Input Capture 1 Interrupt Priority bits
																// IPC1bits.IC1IS   = 0;  // Input Capture 1 Interrupt Subpriority bits
																// IEC0bits.IC1IE   = 0;  // 0= Disable ; 1=enable IC1 interrupt

   
   IC4CON |= 0x8000 ; // turn on IC4    //   IC4CONbits.ON = 1; "do not compile" // bit 15 = ON bit
 T3CONbits.TON = 1;
 
//    T1CONbits.TCS     = 0 ; // Timer Clock Source Select bit                    ; Internal peripheral clock = 0
//    T1CONbits.TCKPS   = 3 ; // <1:0>: Timer Input Clock Prescale Select bits    ; 01 = 1:8 prescale value
//    T1CONbits.SIDL    = 0 ; // Stop in Idle Mode bit                            ; 0 = Continue module operation when the device enters Idle mode
//    T1CONbits.TON    = 1   ; // Timer On bit                                     ; 1 = Timer is enabled ; 0 = Timer is disabled

T4CONbits.TON    = 0 ; // Stop any 16/32-bit Timer4 operation
T5CONbits.TON    = 0 ; // Stop any 16-bit Timer5 operation
T4CONbits.T32    = 1 ; // =1; Enable 32-bit Timer mode
T4CONbits.TCS    = 0 ; // Select internal instruction cycle clock
T4CONbits.TGATE  = 0 ; // Disable Gated Timer mode
T4CONbits.TCKPS  = 7 ; // Select 1:256 Prescaler

TMR4 = 0x00;       // Clear 32-bit Timer       (lsw) 4 et (msw) 5
PR4  = 0xffffffff; // Load 32-bit period value (lsw) 4 et (msw) 5
//IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
//IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
//IEC0bits.T3IE = 0; // disnable Timer3 interrupt

//  T4CONbits.TON = 1; // Start 32-bit Timer      


// pinMode(18, OUTPUT); digitalWrite(18, LOW); //yj ****
}



void loop() //   +0++a++N+  +0++a++N+  
{

  if(is_running )  
  {
    while(!( IC4CON & 0x0008)){} ;                                         // while(!(OBCI.isDataAvailable())){}   // wait for DRDY pin2.D9..PGED3/VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/PMD7/RA0

								 OBCI.ere |=0x100 ;     OBCI.push();         // reads A/D and push data , time stamp, ere, accel, aux...

    // Bin_format = 1 after hard reset, 0 after 'b' command
	if(Bin_format ) { writeBdataToSdCard();   if( IC4CON & 0x0008 ) { OBCI.ere |=0x400 ; 	OBCI.push(); } ;  } 
	else
	  while ( OBCI.read_ix != OBCI.write_ix )
	  {
         OBCI.updateChannelData();          								 // get from the FIFO ADS data , time stamp, ere, accel, aux...    
		 if( IC4CON & 0x0008 ) { OBCI.ere |=0x200 ; 	OBCI.push(); } ;     // Polling here is not neccessary but it might help ;     
        
		 if(SDfileOpen) 
			{
				writeDataToSDcard ( SDsampleCounter++ );    
				//if(SDsampleCounter == 0 && writeToSDonly ) { Serial0.print("SD BLOCKS written: "); Serial0.println(blockCounter);  }     // verbosity
			}
		 if( IC4CON & 0x0008 ) { OBCI.ere |=0x400 ; 	OBCI.push(); } ;     // reads A/D and push data , time stamp, ere, accel, aux...
		 
		//if(!writeToSDonly&&OBCI.sampleCounter++==0)  Serial0.println(".");
		  if(!writeToSDonly)  OBCI.sendChannelData();                         //  sample rate forced to 250 by 'b' command
		 
         if ( SDsampleCounter == 0 )   { digitalWrite(LED, HIGH);  }  			
	     if ( SDsampleCounter == 3 )   { digitalWrite(LED,  LOW);  }  			
      }		 
      
  }
   eventSerial();
   if(!SDfileOpen && SessionSize){ SDfileOpen = setupSDcard( /* & SessionSize */ ); if(!SDfileOpen) SessionSize = 0 ; } // open a new file if the  session is not finished
}







// ecrire ces donnees dans le footer sur SD
// utiliser la cache 512 pour stocker les moyennes et le buf circ d'entier, ainsi que les commentaires

enum burger_case {	
					waiting_burger_plus_start , 
					waiting_burger_patty      , 
					waiting_burger_plus_end 	
};
int burger_state = 	waiting_burger_plus_start ;
			
enum command_case {	
					waiting_command_beginning		,
					waiting_ChannelSettings_value	, 
					waiting_LeadOffSettings_value	,
					waiting_freq_value				,
					waiting_size_value				,
					waiting_base_value  			,
					waiting_duration_value			,
					waiting_mode_value				,
					waiting_comment
};

int command_state = waiting_command_beginning ;
int comment_length = 0 ;
char comment[32];
char testChar;
/*
                    k l     o 
a b c d e f g h i j     m n   p q r s t u v w x y z
A B C D E F G H I J K L   N 0   Q R S T U   W X Y Z    
0123456789

 %+ . !@#$%^&* -=[]?  <>  

*/

void eventSerial(){

  while(Serial0.available())  
  {    
  
    char inChar = (char)Serial0.read();
	
	switch ( burger_state )
	{
	  case waiting_burger_plus_start :  if(inChar=='+')   burger_state = waiting_burger_patty      ; break ; // commandTimer = millis(); 
	  case waiting_burger_patty      :  if(inChar=='+') { burger_state = waiting_burger_plus_start ; break ; } else { testChar = inChar; burger_state = waiting_burger_plus_end ; break;}
	  case waiting_burger_plus_end   :                    burger_state = waiting_burger_plus_start ;    if(inChar!='+')  break ; 

        Serial0.print(testChar) ;
        
        switch (command_state)  // command_state is only used for commands which length > 1 char
	    {
	  
			case waiting_ChannelSettings_value :   loadChannelSettings(testChar)     ;  break ;   // command_state = waiting_command_beginning ;  done in the function loadChannelSetting
				
				
			case waiting_LeadOffSettings_value :  loadLeadOffSettings(testChar)     ;  break ;    // commande_state = waiting_command_beginning ;  done in the function called
				
				
			case waiting_comment 		: if(testChar == '>') { comment[ 31&comment_length] = 0 ; Serial0.println(comment_length);
											//for ( int i =0;i<comment_length;i++) Serial0.println(comment[i]);
											  Serial0.println(comment); 
											command_state = waiting_command_beginning ;  break ; 
										  }
										  else comment[ 31&comment_length++] = testChar ; break;
							 
			case waiting_freq_value 	: if('1' <= testChar && testChar <='4') 
											{ Sample_Rate_SD_Only = 0b110-(testChar-'1'); 
											  if(EEPROM.read(2)!=Sample_Rate_SD_Only) EEPROM.write(2 , (uint8_t)Sample_Rate_SD_Only); 
											Serial0.println(250<<(0b110-Sample_Rate_SD_Only));  
											} // put eeprom   1:250   2:500   3:1000   4:2000
										  command_state = waiting_command_beginning ;  break ;
								  
			case waiting_size_value 	: // Serial0.print(testChar) ;
										  if(testChar == '>') // write eeprom  
										   { EEPROM.write(3 , (uint8_t) max_block_count); 
										     max_block_count = max_block_count ? max_block_count *= 10000 : 512 ;        // 2048 == 1M/512
										     Serial0.print(" : ") ;Serial0.print(max_block_count) ; Serial0.print(" Blocks or ") ; Serial0.print(max_block_count/2048) ; Serial0.println(" Mega Bytes") ; 
											 command_state = waiting_command_beginning ;  break ; 
										   } 
								          if( (testChar < '0') || ('9' < testChar )  ) {   break ; }            //Serial0.print("(erreur)") ;
										  max_block_count = max_block_count*10 + testChar-'0' ;   break ;       // Serial0.println(max_block_count) ;
										  
			case waiting_base_value		:   if(  '0'<= testChar  && testChar <= '9' )   
										  {union { uint16_t i ; char c[2] ; } filenumber ; filenumber.i = 1000*(testChar-'0'); EEPROM.write( 0 , filenumber.c[0] );EEPROM.write( 1 , filenumber.c[1] );}
										  command_state = waiting_command_beginning ; break ;
								  
			case waiting_duration_value 	: if(testChar == '>') { SessionSize = (SessionTime*60*(250<<(0b110-Sample_Rate_SD_Only)))/512 ; 	SessionSize *= (Bin_format ? 32 : 80) ;
																Serial0.print(SessionTime/60); Serial0.print("h"); Serial0.print((SessionTime%60)/10); Serial0.println(SessionTime%10); delay(10);
																Serial0.print(SessionSize); Serial0.println(" Blocks"); delay(10);
																command_state = waiting_command_beginning ;  																			break ; } 
										  if(testChar == 'h') {SessionTime *= 60 ; 																							break ; }
										  if(!(('0' <= testChar) && (testChar <= '9')) ) { Serial0.println(" Error, do it again"); delay(10); command_state = waiting_command_beginning ;  	break ; }
								          else { SessionTime += ((SessionTime%60)*9) + (testChar-'0') ; 															break ; } 

	  
			case  waiting_command_beginning   : 
	  
					switch (testChar){
					
					 case '<' : command_state = waiting_comment     ;  comment_length  = 0 ;   break ; // Comment
					 case '.' : command_state = waiting_freq_value  ;                          break ; // Sample Rate 1,2,3,4 for 250 500 1000 2000
					 case 'm' : command_state = waiting_size_value  ;  max_block_count = 0 ;   break ; // Maximal size of the file
					 case 'n' : command_state = waiting_base_value  ;  						   break ; // Attend un char de 0 a 9 -> BNumero du premier fichier 1000, 2000, 3000, ...
					 case 'h' : command_state = waiting_duration_value ;  SessionTime     = 0 ;   break ; // Duration off the recording session
				   //case 'k' : command_state = waiting_mode_value ;   data_mode       = 0 ;   break ; // To do : Accelerometer = 1 bit 0  |  Auxiliary data = 2 bit 1  |  test = 4 bit 2
					 case 'B' :                                        Bin_format     ^= 1 ;   break ; // Toggle Bin_format , use (start from scratch 'v') to check it's state
					                                                                                   
						//TURN CHANNELS ON/OFF COMMANDS
					 case '1': case'2': case'3': case'4': case'5': case'6': case'7': case'8': changeChannelState_maintainRunningState(inChar-'1'+1,DEACTIVATE); break;

					 case '!':        changeChannelState_maintainRunningState(1,ACTIVATE); break;
					 case '@':        changeChannelState_maintainRunningState(2,ACTIVATE); break;
					 case '#':        changeChannelState_maintainRunningState(3,ACTIVATE); break;
					 case '$':        changeChannelState_maintainRunningState(4,ACTIVATE); break;
					 case '%':        changeChannelState_maintainRunningState(5,ACTIVATE); break;
					 case '^':        changeChannelState_maintainRunningState(6,ACTIVATE); break;
					 case '&':        changeChannelState_maintainRunningState(7,ACTIVATE); break;
					 case '*':        changeChannelState_maintainRunningState(8,ACTIVATE); break;
					  
					 case 'q':        changeChannelState_maintainRunningState( 9,DEACTIVATE); break;
					 case 'w':        changeChannelState_maintainRunningState(10,DEACTIVATE); break;
					 case 'e':        changeChannelState_maintainRunningState(11,DEACTIVATE); break;
					 case 'r':        changeChannelState_maintainRunningState(12,DEACTIVATE); break;
					 case 't':        changeChannelState_maintainRunningState(13,DEACTIVATE); break;
					 case 'y':        changeChannelState_maintainRunningState(14,DEACTIVATE); break;
					 case 'u':        changeChannelState_maintainRunningState(15,DEACTIVATE); break;
					 case 'i':        changeChannelState_maintainRunningState(16,DEACTIVATE); break;
					  
					 case 'Q':        changeChannelState_maintainRunningState( 9,ACTIVATE); break;
					 case 'W':        changeChannelState_maintainRunningState(10,ACTIVATE); break;
					 case 'E':        changeChannelState_maintainRunningState(11,ACTIVATE); break;
					 case 'R':        changeChannelState_maintainRunningState(12,ACTIVATE); break;
					 case 'T':        changeChannelState_maintainRunningState(13,ACTIVATE); break;
					 case 'Y':        changeChannelState_maintainRunningState(14,ACTIVATE); break;
					 case 'U':        changeChannelState_maintainRunningState(15,ACTIVATE); break;
					 case 'I':        changeChannelState_maintainRunningState(16,ACTIVATE); break;

						// TEST SIGNAL CONTROL COMMANDS
					 case '0':        activateAllChannelsToTestCondition(ADSINPUT_SHORTED,ADSTESTSIG_NOCHANGE,ADSTESTSIG_NOCHANGE); break;
					 case '-':        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_SLOW); break;
					 case '=':        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_1X,ADSTESTSIG_PULSE_FAST); break;
					 case 'p':        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_DCSIG); break;
					 case '[':        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_SLOW); break;
					 case ']':        activateAllChannelsToTestCondition(ADSINPUT_TESTSIG,ADSTESTSIG_AMP_2X,ADSTESTSIG_PULSE_FAST); break;

					  
								// SD CARD COMMANDS // En format Binaire : 900000 block de 512 octes = 8 heures a 500Hz ou 460M octes
								//  #define max_block_count 2^20
								//  #define max_block_count 900000
								//  #define max_block_count 512

						//5min     15min    30min    1hr      2hr      4hr      12hr     24hr    512blocks
					 case 'A': case'S': case'F': case'G': case'H': case'J': case'K': case'L': case 'a':    
									// use limit to determine file size - session size
								  switch(testChar){
									  // case 'h':    SessionSize =         50	; break;
									  case 'a':    SessionSize =        512	; break;
									  case 'A':    SessionSize =  	  1100 	; break;	// #define BLOCK_5MIN    11000
									  case 'S':    SessionSize = 	  33000	; break;	// #define BLOCK_15MIN   33000
									  case 'F':    SessionSize = 	  66000	; break;	// #define BLOCK_30MIN   66000
									  case 'G':    SessionSize =   	 131000	; break;	// #define BLOCK_1HR    131000
									  case 'H':    SessionSize =  	 261000	; break;	// #define BLOCK_2HR    261000
									  case 'J':    SessionSize = 	 521000	; break;	// #define BLOCK_4HR    521000 
									  case 'K':    SessionSize =  	1561000	; break;	// #define BLOCK_12HR  1561000
									  case 'L':    SessionSize =  	3122000	; break;	// #define BLOCK_24HR  3122000
									  default : Serial0.println("invalid BLOCK count")	; SessionSize =  	3122000	; break;	//
								  }
							 break;       //  SDfileOpen = setupSDcard(fileSize);

							 
					 case 'j':         if(SDfileOpen){ SDfileOpen = closeSDfile(); } SessionSize =  0	;       break;// close the file and the session
					  

						// CHANNEL SETTING COMMANDS
					 case 'x':  if(!is_running) {Serial0.println("ready to accept new channel settings");}   			// expect 6 parameters
								 channelSettingsCounter = 0;        
								 command_state = waiting_ChannelSettings_value ; break;
					 case 'X':  if(!is_running) {Serial0.println("updating channel settings");}											// latch channel settings
								 writeChannelSettings_maintainRunningState(currentChannelToSet); break;
					 case 'd':  if(!is_running) {Serial0.println("updating channel settings to default");} 			// reset all channel settings to default
								 setChannelsToDefaultSetting(); break;
					 case 'D':  sendDefaultChannelSettings(); break; // report the default settings

						// LEAD OFF IMPEDANCE DETECTION COMMANDS
					 case 'z':  if(!is_running) {Serial0.println("ready to accept new impedance detect settings");}// expect 2 parameters
								 leadOffSettingsCounter = 0;         
								 command_state = waiting_LeadOffSettings_value ;        break; // reset counter
					 case 'Z':  if(!is_running) {Serial0.println("updating impedance detect settings");} // latch impedance parameters
								 changeChannelLeadOffDetect_maintainRunningState(currentChannelToSet);        break;

						// DAISY MODULE COMMANDS
					 case 'c':  if(OBCI.daisyPresent){ OBCI.removeDaisy(); }        outputType = OUTPUT_8_CHAN;        break;// use 8 channel mode
					 case 'C':  // use 16 channel mode
						if(OBCI.daisyPresent == false){OBCI.attachDaisy();}
						if(OBCI.daisyPresent){  Serial0.print("16"); outputType = OUTPUT_16_CHAN;        }
						else                 {  Serial0.print("8") ; outputType = OUTPUT_8_CHAN ;        }
						sendEOT();
						break;

						// STREAM DATA AND FILTER COMMANDS
					 case 'N':        
            if(SDfileOpen) stampSD(ACTIVATE);                     // time stamp the start time
            else { Serial0.println("Fichier non ouvert - recommencer..."); break;}
						writeToSDonly = true; OBCI.Sample_Rate = Sample_Rate_SD_Only ;
						if(OBCI.useAccel){OBCI.enable_accel(RATE_25HZ);}      // fire up the accelerometer if you want it
						startRunning(outputType);                             // turn on the fire hose
						
								TMR4 = 0x00;       // Clear 32-bit Timer       (lsw) 4 et (msw) 5
								PR4  = 0xffffffff; // Load 32-bit period value (lsw) 4 et (msw) 5
								T4CONbits.TON = 1;                                    // Start 32-bit Timer      yj ****
								//		 OBCI.read_ix  = 0 ;
								//		 OBCI.write_ix = 0 ;
						break;
						
					 case 'b':  // stream data
						writeToSDonly = false; Bin_format = false ; OBCI.Sample_Rate = 0b110 ;
						if(SDfileOpen)     stampSD(ACTIVATE);                     // time stamp the start time
						if(OBCI.useAccel) {OBCI.enable_accel(RATE_25HZ);}         // fire up the accelerometer if you want it
						startRunning(outputType);                                 // turn on the fire hose
						break;
						
					 case 's':  // stop streaming data
						if(SDfileOpen) stampSD(DEACTIVATE);       // time stamp the stop time
						if(OBCI.useAccel){OBCI.disable_accel();}  // shut down the accelerometer if you're using it
						stopRunning();
						break;
						
					 case 'f':         useFilters = true;         break;
					 case 'g':         useFilters = false;         break;

						//  INITIALIZE AND VERIFY
					 case 'v':         startFromScratch();  break;				// initialize ADS and read device IDs
						 
						//  QUERY THE ADS AND ACCEL REGITSTERS
					 case '?':        printRegisters();      break;             // print the ADS and accelerometer register values
					   
					 default:   break;
					}  // end switch testChar ; debut de commande 
    
        }  // end   switch command_state 
    }  // end    swith burger protocol 
 }  // end     while(Serial0.available())
}  // end      eventSerial


  
  
void sendEOT(){
  Serial0.println("$$$");  // shake hands with the controlling program
}

void loadChannelSettings(char c){

  if(channelSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannelToSet = getChannelNumber(c); // we just got the channel to load settings into (shift number down for array usage)
    channelSettingsCounter++;
    if(!is_running) {
      Serial0.print("load setting ");
      Serial0.print("for channel ");
      Serial0.println(currentChannelToSet+1,DEC);
    }
    return;
  }
//  setting bytes are in order: POWER_DOWN, GAIN_SET, INPUT_TYPE_SET, BIAS_SET, SRB2_SET, SRB1_SET
  if(!is_running) {
    Serial0.print(channelSettingsCounter-1);
    Serial0.print(" with "); Serial0.println(c);
  }
  c -= '0';
  if(channelSettingsCounter-1 == GAIN_SET){ c <<= 4; }
  OBCI.channelSettings[currentChannelToSet][channelSettingsCounter-1] = c;
  channelSettingsCounter++;
  if(channelSettingsCounter == 7){  // 1 currentChannelToSet, plus 6 channelSetting parameters
    if(!is_running) Serial0.print("done receiving settings for channel ");Serial0.println(currentChannelToSet+1,DEC);
    command_state = waiting_command_beginning ; // getChannelSettings = false;
  }
}

void writeChannelSettings_maintainRunningState(char chan){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  stopRunning();                   //must stop running to change channel settings

  OBCI.writeChannelSettings(chan+1);    // change the channel settings on ADS

  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

void setChannelsToDefaultSetting(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  stopRunning();  //must stop running to change channel settings

  OBCI.setChannelsToDefault();   // default channel settings

  if (is_running_when_called == true) {
    startRunning(cur_outputType);  //restart, if it was running before
  }
}

void loadLeadOffSettings(char c){
   if(leadOffSettingsCounter == 0){  // if it's the first byte in this channel's array, this byte is the channel number to set
    currentChannelToSet = getChannelNumber(c); // we just got the channel to load settings into (shift number down for array usage)
    if(!is_running) Serial0.print("changing LeadOff settings for channel "); Serial0.println(currentChannelToSet+1,DEC);
    leadOffSettingsCounter++;
    return;
  }
//  setting bytes are in order: PCHAN, NCHAN
  if(!is_running) {
    Serial0.print("load setting "); Serial0.print(leadOffSettingsCounter-1);
    Serial0.print(" with "); Serial0.println(c);
  }
  c -= '0';
  OBCI.leadOffSettings[currentChannelToSet][leadOffSettingsCounter-1] = c;
  leadOffSettingsCounter++;
  if(leadOffSettingsCounter == 3){  // 1 currentChannelToSet, plus 2 leadOff setting parameters
    if(!is_running) Serial0.print("done receiving leadOff settings for channel ");Serial0.println(currentChannelToSet+1,DEC);
    command_state = waiting_command_beginning ; //  getLeadOffSettings = false; // release the serial COM
  }
}

char getChannelNumber(char n){
  if( '0' < n && n < '9'){    n -= '1';  }
  else switch(n){
    case 'Q':      n = 0x08; break;
    case 'W':      n = 0x09; break;
    case 'E':      n = 0x0A; break;
    case 'R':      n = 0x0B; break;
    case 'T':      n = 0x0C; break;
    case 'Y':      n = 0x0D; break;
    case 'U':      n = 0x0E; break;
    case 'I':      n = 0x0F; break;
    default: break;
  }
  return n;
}

void changeChannelState_maintainRunningState(byte chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  //must stop running to change channel settings
  stopRunning();
  if (start == 1) {
    OBCI.activateChannel(chan);
  } else if (start == 0){
    OBCI.deactivateChannel(chan);
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

void activateAllChannelsToTestCondition(byte testInputCode, byte amplitudeCode, byte freqCode)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;
  //must stop running to change channel settings
  stopRunning(); delay(10);

  //set the test signal to the desired state
  OBCI.configureInternalTestSignal(amplitudeCode,freqCode);
  //change input type settings for all channels
  OBCI.changeInputType(testInputCode);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

int changeChannelLeadOffDetect_maintainRunningState(char chan)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  //must stop running to change channel settings
  stopRunning();

  OBCI.changeChannelLeadOffDetect(chan);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

void sendDefaultChannelSettings(){
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  OBCI.reportDefaultChannelSettings();
  sendEOT();
  delay(10);

  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}

boolean stopRunning(void) {
  if(is_running){
    OBCI.stopStreaming();  // stop the data acquisition, turn off accelerometer
    is_running = false;
    }
    return is_running;
  }

boolean startRunning(int OUT_TYPE) {
  if(!is_running){
    outputType = OUT_TYPE;
    OBCI.startStreaming();  // start the data acquisition, turn on accelerometer
    is_running = true;
  }
    return is_running;
}

void printRegisters(){

  if(!is_running){
    // print the ADS and LIS3DH registers
    OBCI.printAllRegisters();
    sendEOT();
    delay(20);
  }

}

void startFromScratch(){
  if(!is_running){
    OBCI.initialize();     // initializes accelerometer and on-board ADS and on-daisy ADS if present
    delay(500);
    Serial0.print  ("OpenBCI Version 2016/01/18    Bin_format = "      )   ;   Serial0.println( Bin_format                        );   delay(10);
    Serial0.print  ("SD Only Sample Rate : "           )   ;   Serial0.println( 250<<(0b110-Sample_Rate_SD_Only)  );  delay(10);
    Serial0.print  ("Taille Max des Fichiers : "       )   ;   Serial0.println( max_block_count                   );  delay(10);
    OBCI.configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);
    Serial0.print("On Board ADS1299 Device ID: 0x"); Serial0.println(OBCI.ADS_getDeviceID(ON_BOARD),HEX);
    if(OBCI.daisyPresent){  // library will set this in initialize() if daisy present and functional
      Serial0.print("On Daisy ADS1299 Device ID: 0x"); Serial0.println(OBCI.ADS_getDeviceID(ON_DAISY),HEX);
    }
    Serial0.print("LIS3DH Device ID: 0x"); Serial0.println(OBCI.LIS3DH_getDeviceID(),HEX);
    sendEOT();
  }
}



// end


