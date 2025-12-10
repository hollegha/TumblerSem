#include "RoboLib7.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "math.h"

const int PWA = 4, AI1 = 16, PWB = 18, BI1 = 23, ST_BY = 17;
const int M1A = 27, M1B = 26, M2A = 19,  M2B = 25;

#define USR_BTN     2
#define PWM_FORWARD    0.3  // Standard-"Geschwindigkeit" (PWM-Wert)
#define T_FORWARD_MS 1000 // 1 Sekunde geradeaus fahren
#define PWM_TURN  0.15  // Differenz-"Geschwindigkeit" für Drehung

Motor motL(PWA, AI1, -1); // -1==Tumbler Motor
Motor motR(PWB, BI1, -1);
GpIoOut stdby(ST_BY);
GpIoIn btn(USR_BTN);
Encoder encL(M1A, M1B, &motL), encR(M2A, M2B, &motR); // 2-Chan Encoder

MPU6050 mpu;

void InitIO();

// --- KALIBRIERUNGSKONSTANTEN ---
#define CALIBRATION_DURATION_S 5.0f    // Dauer der Kalibrierung
#define SAMPLE_RATE_HZ         100.0f   // Frequenz der Messungen während der Kalibrierung
#define SAMPLE_INTERVAL_MS     (1000 / SAMPLE_RATE_HZ) // Intervall in Millisekunden
#define NUM_CALIBRATION_SAMPLES (CALIBRATION_DURATION_S * SAMPLE_RATE_HZ) // Gesamtzahl der Messungen

// --- GYRO BIAS/OFFSET SPEICHER ---
static int16_t gyro_offset_z = 0; 

// --- GYRO SKALIERUNGSKONSTANTE ---
#define GYRO_RANGE_DPS 2000.0f           // Gewählte Range: 2000 Grad/s
#define GYRO_SCALE_FACTOR (32768.0f / GYRO_RANGE_DPS) // Skalierungsfaktor (z.B. 16.4)

// --- MESS- UND INTEGRATIONSKONSTANTE ---
#define MEASUREMENT_INTERVAL_S (10.0f / 1000.0f) // 10 ms in Sekunden

// --- ZUSTANDSGRÖSSE WINKEL ---
static float theta_gyro = 0.0f; // Akkumulierter Winkel in °

void calibrate_gyro_z() {
    long long sum_gz = 0;
    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        sum_gz += mpu.getGyroZ();
        MyDelay(SAMPLE_INTERVAL_MS);
    }
    gyro_offset_z = (int16_t)(sum_gz / NUM_CALIBRATION_SAMPLES); // Berechne den Mittelwert (Bias bzw. Offset)
    printf("Kalibrierung abgeschlossen. Samples: %d\n", (int)NUM_CALIBRATION_SAMPLES);
    printf("Gyro Z Offset (Bias) berechnet: %d\n", gyro_offset_z);
}

static void gyro_task(void* arg)
{
    int16_t raw_gz = mpu.getGyroZ();
    int16_t corrected_gz_counts = raw_gz - gyro_offset_z; // Korrigierten Wert berechnen (Offset abziehen)
    float gz_dps = (float)corrected_gz_counts / GYRO_SCALE_FACTOR; // Umrechnung Roh-Counts in Grad/s (DPS)
    
    // Integration: d(Theta) = Omega * dt
    float d_theta = gz_dps * MEASUREMENT_INTERVAL_S;
    theta_gyro += d_theta; // Aufintegrieren des Winkels
    
    // Normalisieren des Winkels auf den Bereich +/- 180 Grad
    if (theta_gyro > 180.0f) theta_gyro -= 360.0f; // Korrektur: 360 abziehen
    if (theta_gyro < -180.0f) theta_gyro += 360.0f; // Korrektur: 360 addieren
    
    //printf("DPS: %.2f | d_theta: %.4f deg\n", gz_dps, d_theta); // d_theta ist in Grad
    // printf("Korrigierter Z-Count: %d | Akkumulierter Theta: %.4f deg\n", corrected_gz_counts, theta_gyro); 
}

extern "C" void app_main(void)
{
  InitIO();
  I2cInit(); printf("MPU-Adresse: 0x%X\n", mpu.testConnection()); 
  mpu.Init();
  mpu.setBW(2); // Filterung: 92Hz, 3.9ms Delay (fs=1kHz statt 8kHz) - BW=3: 41Hz, 5.9ms fs=1kHz
  printf("Init OK - Warte auf Tastendruck für Kalibrierung\n");
  
  do {
    MyDelay(100);
  } while (btn.get() == 0);
  printf("Starte Gyro Z-Achsen Kalibrierung für %.1f Sekunden...\n", CALIBRATION_DURATION_S);
  printf("Nicht wackeln!\n");
  MyDelay(500);
  calibrate_gyro_z();
  theta_gyro = 0.0f;
  printf("Kalibration abgeschlossen\n");

  const esp_timer_create_args_t periodic_timer_args = {
            .callback = &gyro_task,
            .name = "gyro_task_timer"
  };

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    
  uint64_t interval_us = (uint64_t)(MEASUREMENT_INTERVAL_S * 1000000); // Integrationsintervall dt
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, interval_us));  

  while (1) {
    for (int i = 0; i < 4; i++) 
    {
      motL.setPow2(PWM_FORWARD); motR.setPow2(PWM_FORWARD);
      MyDelay(T_FORWARD_MS);
      motL.setPow2(0.0f); motR.setPow2(0.0f);
      MyDelay(200);  // um das Ende der Geradeausfahrt sauber erkennen zu können

      if (i == 3) { break; }

      theta_gyro = 0.0f;
      motL.setPow2(-1.0f*PWM_TURN); motR.setPow2(PWM_TURN); // Drehung
      while (abs(theta_gyro) < 90.0f) {
        MyDelay(50);
        // printf("%.2f\n", theta_gyro);
      }
      motL.setPow2(0.0f); motR.setPow2(0.0f);
      printf("90 Grad erreicht (%.2f). Stoppe Drehung.\n", theta_gyro);
    }
    
    do {  // Restart?
      MyDelay(100);
    } while (btn.get() == 0);
  }
}

void InitIO()
{
  stdby.Init(); stdby.Set(1);
  // PWM-Init in dieser Reihenfolge:
  motL.inv = true;
  Motor::InitTimer(); motR.Init(); motL.Init(); Motor::StartTimer();
  motR.setPow2(0.0); motL.setPow2(0.0); // Beide Motoren sicherheitshalber auf 0
  encL.Init(); encR.Init();
  encR.inv = true; // Die .inv Flags kann man immer und überall setzen
}