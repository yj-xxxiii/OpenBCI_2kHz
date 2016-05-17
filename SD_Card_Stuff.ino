

#define OVER_DIM 20 // make room for up to 20 write-time overruns

const char hexchar[] = "0123456789ABCDEF"; 

uint32_t BLOCK_COUNT = 1;
SdFile 		openfile				;  	// want to put this before setup...
Sd2Card 	card(&OBCI.spi,SD_SS)	;	// SPI needs to be init'd before here
SdVolume 	volume					;
SdFile 		root					;
uint8_t* pCache;      // array that points to the block buffer on SD card
uint32_t MICROS_PER_BLOCK = 5000; // block write longer than this will get flaged
uint32_t bgnBlock, endBlock; // file extent bookends
int byteCounter = 0;    // used to hold position in cache
//int blockCounter;       // count up to BLOCK_COUNT with this
boolean openvol;
boolean removefile ;
boolean cardInit = false;
boolean fileIsOpen = false;

struct {
  uint32_t block;   // holds block number that over-ran
  uint32_t micro;  // holds the length of this of over-run
} over[OVER_DIM];

uint32_t overruns;      // count the number of overruns
uint32_t maxWriteTime;  // keep track of longest write time
uint32_t minWriteTime;  // and shortest write time
uint32_t t;        // used to measure total file write time

byte fileTens, fileOnes;  // enumerate succesive files on card and store number in EEPROM
char  currentFileName[] = "FI_0000.TXT"; // file name will enumerate in  0000 - 9999
                                  //   12345678901234567890
prog_char elapsedTime[] PROGMEM = {  "\n%Total time mS:\n"    };  // 17
prog_char     minTime[] PROGMEM = {   "%min Write time uS:\n" };  // 20
prog_char     maxTime[] PROGMEM = {   "%max Write time uS:\n" };  // 20
prog_char     overNum[] PROGMEM = {   "%Over:\n"              };  //  7
prog_char   blockTime[] PROGMEM = {   "%block, uS\n"          };  // 11    74 chars + 2 32(16) + 2 16(8) = 98 + (n 32x2) up to 24 overruns...
prog_char   stopStamp[] PROGMEM = {   "%STOP AT\n"            };  // used to stamp SD record when stopped by PC
prog_char  startStamp[] PROGMEM = {   "%START AT\n"           };  // used to stamp SD record when started by PC





boolean setupSDcard(void){

  if(!cardInit){
       if(!card.init(SPI_FULL_SPEED, SD_SS)) {      Serial0.println("initialization failed. Things to check:");      Serial0.println("* is a card is inserted?"); return fileIsOpen/*=False*/;}     //   
       else                                  {      Serial0.println("Wiring is correct and a card is present.");      cardInit = true;      }
	   // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
       if (!volume.init(card)) {      Serial0.println("Could not find FAT16/FAT32 partition. Make sure you've formatted the card");  return fileIsOpen/*=False*/; }
   }

  if ( SessionSize > max_block_count )  BLOCK_COUNT = max_block_count ;   else BLOCK_COUNT = SessionSize ; 
       SessionSize -= BLOCK_COUNT ;
  
  incrementFileCounter();
  openvol    = root.openRoot(volume)                 ;  { Serial0.print("openroot ret: "); Serial0.println(openvol   ); delay(10); } // if ( !openvol    )
  removefile = openfile.remove(root, currentFileName);  { Serial0.print("remove   ret: "); Serial0.println(removefile); delay(10); } // if ( !removefile )if the file is over-writing, let it!

  if (!openfile.createContiguous(root, currentFileName, BLOCK_COUNT*512UL)) {      Serial0.println("createfdContiguous fail") ; cardInit = false; }
  //else                                                                    {      Serial0.println("got contiguous file...")  ; delay(1);         }
  
  // get the location of the file's blocks
  if (!openfile.contiguousRange(&bgnBlock, &endBlock)) {      Serial0.println("get contiguousRange fail"); cardInit = false; }
  //else                                               {      Serial0.println("got file range...")       ; delay(1);         }
  
  // grab the Cache
  pCache = (uint8_t*)volume.cacheClear();
  
  // tell card to setup for multiple block write with pre-erase
  if (!card.erase(bgnBlock, endBlock))  { Serial0.println("erase block fail"); cardInit = false;   }
  //else                                { Serial0.println( "erased..." );      delay(1);           }
  
  if (!card.writeStart(bgnBlock, BLOCK_COUNT))  { Serial0.println("writeStart fail"); cardInit = false; }
  else                                          { fileIsOpen = true;                    delay(1) ;      }
  
  OBCI.csHigh(SD_SS);  // release the spi
  
  // initialize write-time overrun error counter and min/max write time benchmarks
  overruns     =     0;
  maxWriteTime =     0;
  minWriteTime = 65000;
  byteCounter  =     0;  // counter from 0 - 512
  blockCounter =     0;  // counter from 0 - BLOCK_COUNT;
  
 // send corresponding file name to controlling program
 if(fileIsOpen == true){ Serial0.print("Corresponding SD file ");  Serial0.println(currentFileName); sendEOT();  }
 
 		 OBCI.read_ix  = 0 ;
		 OBCI.write_ix = 0 ;
   while(IC4CON & 0x0008) { int i  = IC4BUF ; } ;	
 return fileIsOpen;
}




boolean closeSDfile(){
  if(fileIsOpen){
    OBCI.csLow(SD_SS);   // take spi
      card.writeStop();
      Serial0.print("Ret sur close()" ) ; Serial0.println(openfile.close()); delay(10);;
    OBCI.csHigh(SD_SS);  // release the spi
    fileIsOpen = false;
    if(!is_running){        // verbosity. this also gets insterted as footer in openFileBLOCK_COUNT
      Serial0.print ("Blocks a ecrire : ")  ;                               Serial0.println(BLOCK_COUNT ) ;  delay(10);
      Serial0.print ("Blocks   ecrits : ")  ;                               Serial0.println(blockCounter) ;  delay(10);
      Serial0.print ("Total Elapsed Time: "); Serial0.print(t)           ;  Serial0.println("mS")         ;  delay(10);
      Serial0.print ("Max write time: ")    ; Serial0.print(maxWriteTime);  Serial0.println(" uS")        ;  delay(10);
      Serial0.print ("Min write time: ")    ; Serial0.print(minWriteTime);  Serial0.println(" uS")        ;  delay(10);
      Serial0.print ("Overruns: ")          ; Serial0.print(overruns)    ;  Serial0.println()             ;  delay(10);
      if (overruns) {
        uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
        Serial0.println("fileBlock,micros");
        for (uint8_t i = 0; i < n; i++) {
          Serial0.print(over[i].block); Serial0.print(','); Serial0.println(over[i].micro);
        }
      }
    }
  }else{
    Serial0.println("no open file to close");
  }
  digitalWrite(LED,HIGH);
  delay(100); // cool down
  return fileIsOpen;
}


void writeDataToSDcard(byte sampleNumber){
  boolean addComma = true ;
  addAuxToSD       = true ;    // yj to test
	
  convertToHex(sampleNumber, 1, addComma);														// convert 8 bit sampleCounter into HEX
    
//for (int currentChannel = 0; currentChannel < 6; currentChannel++)							// 6 channels to not to write to much during my tests (else keep 8 of coarse!!!)
  for (int currentChannel = 0; currentChannel < 8; currentChannel++)							// convert 24 bit channelData into HEX
  {							
   addComma = OBCI.daisyPresent || (currentChannel != 7) || addAuxToSD || addAccelToSD ;
   convertToHex(OBCI.boardChannelDataInt[currentChannel], 5, addComma);
  }
  
  if(OBCI.daisyPresent){
    for (int currentChannel = 0; currentChannel < 8; currentChannel++){
	  addComma =  (currentChannel != 7) || addAuxToSD || addAccelToSD ;
      convertToHex(OBCI.daisyChannelDataInt[currentChannel], 5, addComma);
    }
  }
  
  convertToHex (                 OBCI.b_time_stamp        , 3,  true    );  						// for yj test
  convertToHex (                 OBCI.b_ere               , 3,  true    );   						// for yj test  OBCI.ere = 0 ;// comma = (addAuxToSD || addAccelToSD)
  convertToHex ( circ_buf_mask & OBCI.b_write_ix          , 3,  true    );   
  convertToHex ( circ_buf_mask & OBCI.b_read_ix           , 3,  true    );  
  convertToHex ( circ_buf_mask & OBCI.  write_ix          , 3,  true    );   
  convertToHex ( circ_buf_mask & OBCI.  read_ix           , 3,  false   );  

/*  
 if(addAuxToSD == true)	{																// convert auxData into HEX
  for(int currentChannel = 0; currentChannel <  3; currentChannel++){
	convertToHex(OBCI.auxData[currentChannel], 3, addComma = (currentChannel != 2) ); 	//  if(currentChannel == 1) addComma = false;
  }     addAuxToSD = false; 															// reset addAuxToSD
 }  // end of aux data log
 
 else if(addAccelToSD == true) {  														// if we have accelerometer data to log
  for (int currentChannel = 0; currentChannel < 3; currentChannel++)	{				// convert 16 bit accelerometer data into HEX
    convertToHex(OBCI.axisData[currentChannel], 3, addComma = (currentChannel != 2));	// if(currentChannel == 1) addComma = false;
  }     addAccelToSD = false;  															// reset addAccel
 }	// end of accelerometer data log
 */
 
}



/* 
//    CONVERT RAW BYTE DATA TO HEX FOR SD STORAGE
void convertToHex(long rawData, int numNibbles, boolean useComma){

  for (int currentNibble = numNibbles; currentNibble >= 0; currentNibble--){
    pCache[byteCounter++] = hexchar [ (rawData >> currentNibble*4) & 0x0F ] ;  
    if(byteCounter == 512) writeCache(); 
  }
  
  if(useComma == true)  pCache[byteCounter++] = ',';    else    pCache[byteCounter++] = '\n';  
  if(byteCounter == 512)  writeCache();  
}// end of byteToHex converter

 */

//    CONVERT RAW BYTE DATA TO HEX FOR SD STORAGE
void convertToHex(long rawData, int numNibbles, boolean useComma){

  for (int currentNibble = numNibbles; currentNibble >= 0; currentNibble--){
    byte nibble = (rawData >> currentNibble*4) & 0x0F;
    if (nibble > 9){
      nibble += 55;  // convert to ASCII A-F
    }
    else{
      nibble += 48;  // convert to ASCII 0-9
    }
    pCache[byteCounter] = nibble;
    byteCounter++;
    if(byteCounter == 512){
      writeCache();
    }
  }
  if(useComma == true){
    pCache[byteCounter] = ',';
  }else{
    pCache[byteCounter] = '\n';
  }
  byteCounter++;
  if(byteCounter == 512){
    writeCache();
  }
}// end of byteToHex converter


/* 
void writeBdataToSdCard(void){
	while( ! ((OBCI.write_ix & 0xfff0) == OBCI.read_ix ) ) 
	{ 
		memcpy( pCache , &OBCI.circ_buf[(circ_buf_mask & OBCI.read_ix)*32] , 512 ) ;
		writeCache();
		OBCI.read_ix +=  16 ;
		
		 if ( (OBCI.read_ix & 0xff00) == 0  )   { digitalWrite(LED, HIGH);  }  			 // yj ****     digitalWrite(18, HIGH); 
	     if ( (OBCI.read_ix & 0xff00) == 16 )   { digitalWrite(LED,  LOW);  }  			 // yj ****     digitalWrite(18, LOW );
	}
}
 */
 
 int Led_state =0 ;
 void writeBdataToSdCard(void){
 
	while( ! ((OBCI.write_ix & 0xfff0) == OBCI.read_ix ) ) 
	{ 
//		uint8_t* SpCache = pCache ;
		pCache = &OBCI.circ_buf[(circ_buf_mask & OBCI.read_ix)*32] ;
		writeCache();
//		pCache = SpCache ;
		OBCI.read_ix +=  16 ;
		
		if ( (OBCI.read_ix & 0x00ff) == 0x80) { digitalWrite(LED, Led_state^=1 ); }  
	}
//  digitalWrite(LED, OBCI.read_ix & 0x0100 ); 
//	if ( OBCI.read_ix & 0x0080 ) { digitalWrite(LED, Led_state^=1 ); }  
}



void writeCache(){
    if(blockCounter > BLOCK_COUNT) return;
    uint32_t tw = micros();  // start block write timer
	
    OBCI.csLow(SD_SS);  // take spi
       if(!card.writeData (   pCache,   &OBCI  )) {Serial0.println("block write fail");}   // write the block  //     *********   yj  ********
    OBCI.csHigh(SD_SS);  // release spi
	
	// autant mettre ici, apres "release spi",  le controle du conv A/D car quand la SD est prette l'ecriture des 512 octe dure quand meme 200uS...
	// le faire non bloquant avec Input Capture.
			          //
	  if( IC4CON & 0x0008 ) { OBCI.ere |=0x20 ; 	OBCI.push(); } ;     // reads A/D and push data , time stamp, ere, accel, aux...

	
    tw = micros() - tw;      // stop block write timer
    if (tw > maxWriteTime) maxWriteTime = tw;  // check for max write time
    if (tw < minWriteTime) minWriteTime = tw;  // check for min write time
    if (tw > MICROS_PER_BLOCK) {      // check for overrun
      if (overruns < OVER_DIM) {
        over[overruns].block = blockCounter;
        over[overruns].micro = tw;
      }
      overruns++;
    }
    byteCounter = 0; // reset 512 byte counter for next block
	
    blockCounter++;    // increment BLOCK counter
	// if (  Bin_format && (blockCounter == BLOCK_COUNT  ) ) { t = millis() - t ;   stopRunning()  ;  OBCI.disable_accel()  ;                  }
    // if ( !Bin_format && (blockCounter == BLOCK_COUNT-1) ) { t = millis() - t ;   stopRunning()  ;  OBCI.disable_accel()  ;  writeFooter();  }
    if ( blockCounter == (BLOCK_COUNT-1) ) { t = millis() - t ;   if(SessionSize==0) {stopRunning()  ;  OBCI.disable_accel()  ;}  writeFooter();  }
    if ( blockCounter ==  BLOCK_COUNT    ) {  SDfileOpen = closeSDfile()   ;  BLOCK_COUNT = 0 ;  SDcardWriteComplete = true ;             }  
	// we did it!
}

void incrementFileCounter(){  // EEPROM.put(eeAddress, customVar);  EEPROM.get(eeAddress, customVar);
int i,j ;
union { uint16_t i ; char c[2] ; } filenumber ;

  
  filenumber.c[0] = EEPROM.read(0); filenumber.c[1] = EEPROM.read(1);  				// if it's the first time writing to EEPROM,  file number == 0xffff
  i =  (filenumber.i+1)%1000; j = filenumber.i/1000 ; filenumber.i = j*1000+i ;	//filenumber.i++; 
  EEPROM.write( 0 , filenumber.c[0] );EEPROM.write( 1 , filenumber.c[1] );
  
  currentFileName[ 3] = hexchar [  (filenumber.i/1000)%10   ] ;
  currentFileName[ 4] = hexchar [  (filenumber.i/ 100)%10   ] ;
  currentFileName[ 5] = hexchar [  (filenumber.i/  10)%10   ] ;
  currentFileName[ 6] = hexchar [   filenumber.i      %10   ] ;
//               [ 7] =               '.'                    ;
  currentFileName[ 8] = Bin_format? 'b':'t'                  ;
  currentFileName[ 9] = Bin_format? 'i':'x'                  ;
  currentFileName[10] = Bin_format? 'n':'t'                  ;

}




void stampSD(boolean state){    if (Bin_format) return ;
  unsigned long time = millis();
  if(state){
    for(int i=0; i<10; i++){
      pCache[byteCounter] = pgm_read_byte_near(startStamp+i);
      byteCounter++;
      if(byteCounter == 512){        writeCache();      }
    }
  }
  else{
    for(int i=0; i<9; i++){
      pCache[byteCounter] = pgm_read_byte_near(stopStamp+i);
      byteCounter++;
      if(byteCounter == 512){        writeCache();      }
    }
  }
  convertToHex(time, 7, false);
}

void writeFooter(){    

//if (Bin_format) { blockCounter++; return ;};
// byteCounter reseted by the last writeCache call

                                         for(int i=0; i<17; i++) {    pCache[byteCounter] = pgm_read_byte_near(elapsedTime+i);    byteCounter++;  }
  convertToHex(   t        , 7, false);  for(int i=0; i<20; i++) {    pCache[byteCounter] = pgm_read_byte_near(  minTime  +i);    byteCounter++;  }
  convertToHex(minWriteTime, 7, false);  for(int i=0; i<20; i++) {    pCache[byteCounter] = pgm_read_byte_near(  maxTime  +i);    byteCounter++;  }
  convertToHex(maxWriteTime, 7, false);  for(int i=0; i< 7; i++) {    pCache[byteCounter] = pgm_read_byte_near(  overNum  +i);    byteCounter++;  }
  convertToHex(overruns,     7, false);  for(int i=0; i<11; i++) {    pCache[byteCounter] = pgm_read_byte_near(blockTime  +i);    byteCounter++;  }
  
  if (overruns) {
    uint8_t n = overruns > OVER_DIM ? OVER_DIM : overruns;
    for (uint8_t i = 0; i < n; i++) {
      convertToHex(over[i].block, 7, true);
      convertToHex(over[i].micro, 7, false);
    }
  }
  
  for(int i=byteCounter; i<512; i++)  { pCache[i] = '.' ; }   writeCache() ;  // NULL
}

