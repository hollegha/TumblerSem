/* Einfache Regelung mit dem Beschleunigungssensor (Accelerometer) MPU6050:
Stabilisierung der "Roll-Achse" (y-Achse)

Die x-Achse liegt quer zur Fahrtrichtung und wir erhalten den Wert 0, wenn
der Roboter gerade steht.
Neigt/kippt man ihn mit dem linken Rad nach oben, dann erhalten wir einen positiven Wert
mit mpu.getAccelX() und umgekehrt einen negativen.
Wenn der Roboter in einer Röhre (oder Halfpipe) fährt, dann wird dieses Programm
dafür sorgen, dass der Roboter nicht die Wände hochfährt, denn mit einem
positiven Ausschlag auf der x-Achse wird der linke Motor etwas schneller als der
rechte drehen:
motL.setPow2(SPEED+(float)accX/1000.0) => achte auf das "+" bei motL und das "-" bei motR
-> Kurve nach rechts -> Roboter fährt wieder "im Tal"

Die Division durch 1000 kann ggf. erhöht werden, da 90° Neigung ca. dem Wert 2000
entspricht (default-Konfiguration MPU) -> bei 45° Neigung würden wir 1.0f addieren,
bei 31,5° sind es 0.7f (und damit bereits die Vollaussteuerung der PWM)
Über den Umweg der Motoren und der Röhrengeometrie wird also die y-Achse ausgeregelt.
*/

#include "RoboLib7.h"
#include "EspMotor.h"
#include "MotorSetup.h" // Init-Werte für die Motoren (und Encoder) -> ggf. anpassen/auskommentieren
#include "MPU_Esp.h"   // Lib für den Sensor

#define SPEED    0.3f // Grundgeschwindigkeit (~30% der max. Geschwindigkeit von 1,2m/s = 40cm/s)
#define USER_BTN 2    // GPIO 2 ist der USER-Button auf der neuen Platine

GpIoIn btn(USER_BTN);
MPU6050 mpu;

extern "C" void monitor_task(void* arg) {  // parallel laufender Task zur Ausgabe alle 0,4s
    while(1) {
        printf("%d\n", mpu.getAccelX());
        MyDelay(400);
    }
}

extern "C" void app_main(void) {
    InitIO();
    I2cInit(); // I2C Treiber initialisieren
    printf("MPU-Sensoradresse: 0x%X\n", mpu.testConnection());  // hier sollte 0x68 ausgegeben werden
    mpu.Init(); // Init für den Beschleunigungssenor

    xTaskCreate(monitor_task, "Monitor", 2048, NULL, 10, NULL); // Prio 10 (geht von 1~20)

    do {
        MyDelay(100);
    } while (btn.get() == 0); // wir warten auf den Tastendruck von USR_BTN

    while(1) {
        int accX = mpu.getAccelX(); // Wert zwischenspeichern, damit identisch gesteuert wird
        motL.setPow2(SPEED+(float)accX/1000.0); motR.setPow2(SPEED-(float)accX/1000.0);
        MyDelay(100);  // 100ms = 10 Hz
    }
}
