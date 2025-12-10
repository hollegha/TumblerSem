#pragma once

#include "Tp1Ord.h"

const int N_LS_CHAN = 6;
const float CAL_AMPL = 2000.0;

const int DISP_RAW =  1;
const int CALIBRATE = 2;
const int CAL_VALS  = 3;
const int POS_VALS = 4;

class LineChannel{
  const float ALPHA = 0.07; // 0.07
  const float BETHA = 1 - ALPHA;
public:
  LineChannel()
  {
    
  }

  void readADC()
  {
    
  }
  
  void initCal()
  {
    
  }
  
  void calcCalibration()
  {
    // calcFilt(idx);
    // y must be filtered
    
  }
  
  void calcFilt() 
  {
    
  }
  
  float calVal()
  {
    
  }
public:
  int rawVal;
  float y;
public:
  int _min, _max;
  float _k;
public:
  Adc1 adc;
};


LineChannel lsAry[N_LS_CHAN];



// 9000 200
class LineSensor {
public:
  int pos, pos2, pos3;
  float posDiff;
  Tp1Ord tp;
public:
  LineSensor()
  {
    
  }

  float posNorm()
  {
    
  }

  float diffNorm()
  {
    
  }

  void initADC()
  {
    /* lsAry[0].adc._ch = ADC_CHANNEL_0;
    lsAry[1].adc._ch = ADC_CHANNEL_3;
    lsAry[2].adc._ch = ADC_CHANNEL_6;
    lsAry[3].adc._ch = ADC_CHANNEL_7;
    lsAry[4].adc._ch = ADC_CHANNEL_4;
    lsAry[5].adc._ch = ADC_CHANNEL_5; */
    lsAry[0].adc._ch = ADC_CHANNEL_0;
    lsAry[1].adc._ch = ADC_CHANNEL_3;
    lsAry[2].adc._ch = ADC_CHANNEL_4;
    lsAry[3].adc._ch = ADC_CHANNEL_5;
    lsAry[4].adc._ch = ADC_CHANNEL_6;
    lsAry[5].adc._ch = ADC_CHANNEL_7;
    for (int i = 0; i < N_LS_CHAN; i++) {
      lsAry[i].adc.Init();
    }
  }

  void setDefaultCalib()
  {
    setCal(0, 15, 2801);
    setCal(1, 2, 2149);
    setCal(2, 0, 2042);
    setCal(3, 0, 2318);
    setCal(4, 0, 2112);
    setCal(5, 7, 3113);
  }
  
  void readADC()
  {
    
  }

  void dispRawVals(SvProtocol3* ua)
  {
    for (int i = 0; i < N_LS_CHAN; i++)
      ua->WriteSvI16(i + 1, lsAry[i].rawVal);
  }

  void dispYVals(SvProtocol3* ua)
  {
    for (int i = 0; i < N_LS_CHAN; i++)
      ua->WriteSvI16(i + 1, lsAry[i].y);
  }

  void dispMinMax(SvProtocol3* ua)
  {
    ua->SvMessage("Min Max");
    for (int i = 0; i < N_LS_CHAN; i++)
      ua->SvPrintf("%d %d", lsAry[i]._min, lsAry[i]._max);
  }
  
  void setCal(int idx, int aMin, int aMax)
  {
    lsAry[idx]._min = aMin; lsAry[idx]._max = aMax;
    lsAry[idx]._k = CAL_AMPL / (float)(aMax - aMin);
  }

  void initCal()
  {
    for (int i = 0; i < N_LS_CHAN; i++)
      lsAry[i].initCal();
  }

  void calStep()
  { // y must be filtered
    for (int i = 0; i < N_LS_CHAN; i++)
      lsAry[i].calcCalibration();
  }

  void calcFilt()
  {
    for (int i = 0; i < N_LS_CHAN; i++)
      lsAry[i].calcFilt();
  }

  void calcCal()
  {
    for (int i = 0; i < N_LS_CHAN; i++)
      lsAry[i].calVal();
  }

  void calcPos();

  void calcPos2();
  void calcPos3();

  bool checkRange(int idxL, int idxR);

  bool isMidZero()
  {
    const float LOW_LEVEL = 50;
    if (lsAry[1].y < LOW_LEVEL && lsAry[2].y < LOW_LEVEL
      && lsAry[3].y < LOW_LEVEL && lsAry[4].y < LOW_LEVEL)
      return true;
    return false;
  }

  bool allZero()
  {
    const float LOW_LEVEL = 50;
    for (int i = 0; i < N_LS_CHAN; i++)
      if (lsAry[1].y > LOW_LEVEL)
        return false;
    return true;
  }
private:
  int z1, z2;
};


void LineSensor::calcPos()
{
  
}

const int RG_MIN = 20;
const int RG_MAX = 1800; // 1800 2000

void LineSensor::calcPos3()
{
  
}

bool LineSensor::checkRange(int idxL, int idxR)
{
  
}


// links:0 rechts:5 links:- rechts:+
void LineSensor::calcPos2()
{
  
}


