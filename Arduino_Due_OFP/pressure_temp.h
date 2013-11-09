
#define BMP085_ADDRESS 0x77  // I2C address of BMP085

void bmp085Calibration(short int *ac1_ptc,short int *ac2_ptc,short int *ac3_ptc,unsigned int *ac4_ptc,unsigned int *ac5_ptc,unsigned int *ac6_ptc,short int *b1_ptc,short int *b2_ptc,short int *mb_ptc,short int *mc_ptc,short int *md_ptc);

int bmp085ReadInt(unsigned char address);
char bmp085Read(unsigned char address);

short bmp085GetTemperature(unsigned int ut, unsigned int ac5_ptc,unsigned int ac6_ptc, short int mc_ptc,short int md_ptc, long *b5);
long bmp085GetPressure(unsigned long up,short int ac1,short int ac2,short int ac3,unsigned int ac4,short int b1,short int b2,long b5,const unsigned char OSS);

void bmp085RequestUT();
unsigned int bmp085ReadUT();
void bmp085RequestUP(const unsigned char OSS);
unsigned long bmp085ReadUP(const unsigned char OSS);


