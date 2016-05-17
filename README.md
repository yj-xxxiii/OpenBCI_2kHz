# OpenBCI_32-write-to-DS-only
OpenBCI 32 code mod to enable writing to the SD card only up to 2kHz plus some minor adds...

Does all the things that the OpenBCI_32 code does, but it has added features. To access these features, you must send commands which activates the board . You can run this from any serial terminal (OpenBCI doesn not have this functionality yet) and send the individual character commands.
  
      This version, called OpenBCI_32_SD_Only has added features to optionally :

      - Log data to the SD card only. 
        To do this, send an 'N' over the serial port.
       
      - Toggle between hex or binary format for the SD data. 
        To do this, send a single 'B' over the serial port.
                 extention of the created files will be then 'bin' or 'hex'.
                 bin format yields 32 bytes by sample ,  hex format is estimated to 80 bytes...
                 binary raw data from the A/D are 8 24bits signed Big Endian,  
                 auxiliary data from the Pic32 are 4 16 bits Litlle endian...
                 to get auxiliary data (or anything else) in the 4 16 words you need to customize this firmware.
       
      - Set the sample rate to 250, 500, 1000, 2000Hz. 
        To do this, send an '.' followed by a char in '1' to '4' corresponding to 250..2000.
        ex : '.2' sets to 500Hz

      - Define the duration of the record session. 
        To do this, send an 'hxxhxx>' .
        ex : 'h1h>' 	  sets to 1 hour
             'h2h30>'	  sets to 2 hours 30 minutes.
             'hh5>'     sets to          5 minutes.
             'hh99h'	  sets to 1 hour  39 minutes.
             'hh125>'   sets to 2 hours  5 minutes.
             'hh1234>'  sets to 2 hours 34 minutes!!
             'hh12345>' sets to 7 hours 45 minutes!!! The code is minimalist, so stay inside ' xxhxx '
         this command echoes 
             - the duration formated as 'xxhxx'
             - the corresponding closest number of blocks according to the current sample rate and the recording format (hex/bin)
 
     - Define the maximum size of the file created on the SD card only. 
        To do this, send an 'mn>' , n = number of 10000 Blocks; a block = 512 chars ; '>' to end the command.
        ex : 'm100>' sets the maximum size to 100 x 10000 x 512 Chars equals 512M bytes
             'm0>'   sets the size to   512 Blocks.
             'm1>'   sets the size to 10000 Blocks.
          If the recording session duration corresponds to more blocks then new files are incremently created until the end of the session.
          Take in account that during file Creating/opening which duration is 1 to 2 seconds, acquired data are lost...
 
      - Define the starting number nnnn of the SD file name which format is FI_nnnn . 
        To do this, send an 'n' , followed by a char in '0' to '9' corresponding to 0000..9000.
        ex : 'n0' sets the first file name to FI_0000.bin/hex
             'n1' sets the first file name to FI_1000.bin/hex
          The idea was to not have identical file name when working with several stations
          On reflexion it can be different and improved...

      - Last but not least, overruns caused by busy states of the SD interface have been overcome using pooling of the A/D converter and a circular buffer.
        When AD convertion is completed reading of a hard counter of the Pic32 constitutes a "time stamp" stored in the first 16 bits word of the auxiliary field.
        The clock period that increments this  hard counter is 256x25 nano seconds.  
        At a sample rate of 2kHz , the difference between two successive reading is equal to around 78 that corresponds to 0.4992 micro seconds.
        This way one can verify if overruns occured or not.


Tutorial for getting started from scratch uploading to OpenBCI 32bit found [**here**](http://docs.openbci.com/tutorials/02-Upload_Code_to_OpenBCI_Board#upload-code-to-openbci-board-32bit-upload-how-to)
