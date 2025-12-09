
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "driver/gpio.h"
#include "LineSensor.h"

SvProtocol3 ua0;
// GpIoOut irLed(23);
LineSensor ls;
int dispMode = DISP_RAW;

void ExecLineSensor();
void setDispMode(int mode);

void InitIO()
{
  // irLed.Init(); irLed.Set(1);
  Adc1::atten = ADC_ATTEN_DB_11;
  ls.initADC();
  ls.setDefaultCalib();
}

void CommandLoop()
{
  int cmd;
  while (1) {
    cmd = ua0.GetCommand();
    if (cmd == 2) {
      // int onOff = ua0.ReadI16(); irLed.Set(onOff);
      // ua0.SvMessage2("IR", onOff);
    }
    else if (cmd == 3)
      setDispMode(ua0.ReadI16());
    else if (cmd == 4)
      ls.dispMinMax(&ua0);
    else if (cmd == 5) {
      ls.initCal();
      ua0.SvMessage("Init cal");
    }
    else if (cmd == 6) {
      float fg = ua0.ReadF();
      ls.tp.SetAlpha(fg);
      ua0.SvMessage("set Alpha");
    }
    else if (cmd == 50) {
      esp_restart();
    }
  }
}

extern "C" void Monitor(void* arg)
{
  while (1) {
    vTaskDelay(1);
    if (ua0.acqON) {
      ExecLineSensor();
      ua0.Flush();
    }
  }
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

void ExecLineSensor()
{
  ls.readADC();
  if (dispMode == DISP_RAW) {
    ls.dispRawVals(&ua0);
  }
  else if (dispMode == CALIBRATE) {
    ls.calcFilt();
    ls.calStep();
    ls.dispYVals(&ua0);
  }
  else if (dispMode == CAL_VALS) {
    ls.calcCal();
    ls.calcPos3(); 
    ls.dispYVals(&ua0);
    ua0.WriteSvI16(7, 1000*ls.posNorm());
    ua0.WriteSvI16(8, 1000*ls.diffNorm());
  }
}

extern "C" void app_main(void)
{
	printf("LineMonitor\n");
  InitRtEnvHL();
  InitIO();
  InitUart(UART_NUM_0, 500000);
  xTaskCreate(Monitor, "Monitor", 2048, NULL, 10, NULL); // 10=Prio
  CommandLoop();
}











