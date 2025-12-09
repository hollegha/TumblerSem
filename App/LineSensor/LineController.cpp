
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "driver/gpio.h"
#include "LineSensor.h"
#include "LineController.h"
#include "MotorSetup.h"

void ExecLineSensor();


void CommandLoop()
{
  int cmd;
  while (1) {
    cmd = ua0.GetCommand();
    if (cmd == 3)
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
    else if (cmd == 7) {
      lpd.on = ua0.ReadI16();
      ua0.SvMessage2("RGL ", lpd.on);
    }
    else if (cmd == 11) {
      int val = ua0.ReadI16();
      stdby.Set(val);
      ua0.SvMessage2("stdby", val);
    }
    else if (cmd == 200 || cmd == 201)
      ControlParams(cmd);
    else if (cmd == 50) {
      esp_restart();
    }
  }
}


extern "C" void Monitor(void* arg)
{
  while (1) 
  {
    vTaskDelay(1);
    ExecLineSensor();
    Display();
  }
}

void ExecLineSensor()
{
  ls.readADC();
  if (dispMode == CALIBRATE) {
    ls.calcFilt();
    ls.calStep();
  }
  else if (dispMode == CAL_VALS) {
    ls.calcCal();
    ls.calcPos3();
  }
  if (lpd.on) {
    lpd.calcOneStep();
    motL.setPow2(lpd.out); motR.setPow2(-lpd.out);
  }
  else {
    motL.setPow2(0); motR.setPow2(0);
  }
}


extern "C" void app_main(void)
{
	printf("LineController_3\n");
  InitRtEnvHL();
  InitMotors(); InitIO();
  // InitUart(UART_NUM_0, 500000);
  InitSoftAp("sepp", 8);
  xTaskCreate(Monitor, "Monitor", 2048, NULL, 10, NULL); // 10=Prio
  CommandLoop();
}











