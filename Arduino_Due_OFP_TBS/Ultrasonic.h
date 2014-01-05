#include <stddef.h>
#include <Arduino.h>

typedef struct bufferCtl
{
  float *pBegin;
  float *pIndex;
  size_t length;
  bool filled;
} 
BufCtl;

class Ultrasonic
{
public:
  Ultrasonic(int tp, int ep);
  long timing();
  float convert(long microsec, int metric);
  void setDivisor(float value, int metric);
  static const int IN = 0;
  static const int CM = 1;
  bool sampleCreate(size_t size, ...);
  void sampleClear();
  float unbiasedStdDev(float value, size_t bufNum);
private:
  int _trigPin;
  int _echoPin;
  float _cmDivisor;
  float _inDivisor;
  size_t _numBufs;
  BufCtl *_pBuffers;
  void _sampleUpdate(BufCtl *buf, float msec);
  void _freeBuffers();
};


