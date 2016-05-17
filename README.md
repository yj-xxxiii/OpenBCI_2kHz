# OpenBCI_32-write-to-DS-only
OpenBCI 32 code mod to enable writing to the SD card only

Does all the things that the OpenBCI_32 code does, but it has added feature to write to the SD card only, and not stream to the computer. To access this feature, you must send one of the [SD card Commands](http://docs.openbci.com/software/01-OpenBCI_SDK#openbci-sdk-command-set-sd-card-commands), and then send a 'N', which activates the board to write data to the SD. You can run this from any serial terminal (OpenBCI doesn not have this funcitonality yet) and send the individual character commands.
  
Super new. It has been tested, and it works.  Tutorial coming soon.

Tutorial for getting started from scratch uploading to OpenBCI 32bit found [**here**](http://docs.openbci.com/tutorials/02-Upload_Code_to_OpenBCI_Board#upload-code-to-openbci-board-32bit-upload-how-to)
