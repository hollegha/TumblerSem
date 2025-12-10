
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "driver/gpio.h"
#include "LineSensor.h"
#include "LineController.h"
#include "MotorSetup.h"

void ExecLineSensor();
void DriveUntilLost();

float forew = 0.0;
int driveCmd = 0;


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
    else if (cmd == 8) {
      float pow = ua0.ReadF();
      motL.setPow2(pow); motR.setPow2(-pow);
      ua0.SvMessage("rotate");
    }
    else if (cmd == 10) {
      forew = ua0.ReadF();
      if (forew == 0) {
        lpd.on = 0;
        motL.setPow2(0); motL.setPow2(0);
      }
      ua0.SvMessage("forew");
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
    motL.setPow2(forew + lpd.out); motR.setPow2(forew - lpd.out );
  }
}


extern "C" void DriveControl(void* arg)
{
  while (1) {
    vTaskDelay(10);
    if (driveCmd == 1) {
      driveCmd = 0;
      ua0.SvMessage("Start");
      DriveUntilLost();
      ua0.SvMessage("Stop");
    }
    if (driveCmd == -1) 
      driveCmd = 0;
  }
}

void DriveUntilLost()
{
  lpd.on = 1;
  while (1) {
    vTaskDelay(1);
    if (driveCmd == -1)
      goto StopAll;
    if (ls.allZero()) {
      vTaskDelay(5);
      if (ls.allZero())
        goto StopAll;
    }
  }
StopAll:
  driveCmd = 0;
  lpd.on = 0;
  motL.setPow2(0); motR.setPow2(0);
}


extern "C" void app_main(void)
{
	printf("LineFollow\n");
  InitRtEnvHL();
  InitMotors(); 
  InitIO();
  // InitUart(UART_NUM_0, 500000);
  InitSoftAp("sepp", 8);
  xTaskCreate(Monitor, "Monitor", 2048, NULL, 10, NULL); // 10=Prio
  // xTaskCreate(DriveControl, "DriveControl", 2048, NULL, 8, NULL);
  CommandLoop();
}











