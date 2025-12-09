
#pragma once
#include "LineSensor.h"
#include "ControlBasics.h"

LineSensor ls;
int dispMode = DISP_RAW;

SvProtocol3 ua0;

class LinePID : public PIDParam {
public:
  float out;
public:
  LinePID(float aKP, float aKD, float aKI, float aKS, const char* aName) :
    PIDParam(aKP, aKD, aKI, aKS, aName)
  {
  }
  void calcOneStep()
  {
    out = ls.posNorm() * KP + ls.diffNorm() * KD;
  }
};

LinePID lpd(0.7, 0.01, 0.0, 0.0, "LinePD");



void InitIO()
{
  Adc1::atten = ADC_ATTEN_DB_11;
  ls.initADC();
  ls.setDefaultCalib();
  lpd.on = 0;
}


void ControlParams(int cmd)
{
  if (cmd == 200) {
    int num = ua0.ReadI16();
    lpd.ReadCOM(&ua0);
    ua0.SvMessage("Set PID");
  }
  if (cmd == 201) {
    int num = ua0.ReadI16();
    ua0.LockOStream();
    ua0.WrB(200);
    lpd.WriteCOM(&ua0);
    ua0.Flush();
    ua0.UnlockOStream();
    ua0.SvMessage("Get PID");
  }
}


void Display()
{
  if (!ua0.acqON)
    return;
  if (dispMode == DISP_RAW) {
    ls.dispRawVals(&ua0);
  }
  else if (dispMode == CALIBRATE) {
    ls.dispYVals(&ua0);
  }
  else if (dispMode == CAL_VALS) {
    ls.dispYVals(&ua0);
    ua0.WriteSvI16(7, 1000 * ls.posNorm());
    ua0.WriteSvI16(8, 1000 * ls.diffNorm());
  }
  ua0.Flush();
}


void setDispMode(int mode)
{
  if (mode == DISP_RAW) {
    ua0.SvMessage("disp raw");
    dispMode = mode;
  }
  else if (mode == CALIBRATE) {
    ua0.SvMessage("calibarion...");
    ls.initCal(); // reihenfolge !!!
    dispMode = mode;
  }
  else if (mode == CAL_VALS) {
    ua0.SvMessage("cal vals");
    dispMode = mode;
  }
}
