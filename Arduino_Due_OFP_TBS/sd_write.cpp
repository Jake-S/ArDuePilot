/*
//#include <SPI.h>
//#include <SdFat.h>
//#include <String.h>

//SD Card
char SD_file_name[25];
SdFat sd;
ofstream SDFile;
int buf_frame_counter = 0;
// In init:


  //SD Card
  if(sd_logging)
  {
  sd.begin(SD_CARD_SPI_PIN, SPI_FULL_SPEED);
  int log_num = 1;
  while(1){
       String stringOne = "log_";
      String stringTwo = stringOne + log_num;
      String stringThree = stringTwo + ".txt";
      stringThree.toCharArray(SD_file_name, 25);
      if(sd.exists(SD_file_name)==1)
          log_num++;
      else
          break;
   }
  SDFile.open(SD_file_name, O_CREAT | O_WRITE | O_APPEND);
  }
     
     
     // In run loop:
     
  // SD Card data Logging
  if(sd_logging)
  {
  SDFile << "12345678" << "\n";
  if(frame == 7)
  {
    buf_frame_counter++;
    if(buf_frame_counter>=4)
    {
    SDFile << flush;
    buf_frame_counter = 1;
    }
  }
  }
  
     
     */
