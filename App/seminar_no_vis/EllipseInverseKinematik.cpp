#include "RoboLib7.h"
#include "EspMotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

const int PWA = 4, AI1 = 16, PWB = 18, BI1 = 23, ST_BY = 17;
const int M1A = 27, M1B = 26, M2A = 19,  M2B = 25;

Motor motL(PWA, AI1, -1); // -1==Tumbler Motor
Motor motR(PWB, BI1, -1);
GpIoOut stdby(ST_BY);
Encoder encL(M1A, M1B, &motL), encR(M2A, M2B, &motR); // 2-Chan Encoder
GpIoIn btn(2);

// --- KINEMATISCHE KONSTANTEN ---
#define WHEEL_BASE          17.5f  // cm - ausprobieren, indem man Roboter am Stand fahren lässt (v_command = 0.0)
#define ELLIPSE_A           50.0f  // cm
#define ELLIPSE_B           25.0f  // cm
#define TICKS_PER_CM        17.143f // 360 Ticks / 21 cm Radumfang
#define TIMESTEP_DT         0.05f  // 50 ms -> muss auch beim Timer in main so gesetzt sein!!!
#define CYCLE_TIME          20.0f  // Ein Umlauf in 20 Sekunden
#define PHI_DOT             M_PI/10.0f // 2*pi/T

// --- REGELUNGSKONSTANTEN ---
#define KP_LINEAR           3.0f   // P-Verstärkung für Lineargeschwindigkeit
#define KP_ANGULAR          1.5f   // P-Verstärkung für Winkelgeschwindigkeit
#define MAX_POWER           1.0f   // Max. PWM-Wert
#define MAX_SPEED           126.0f  // 126.0 Maximale Lineargeschw. in cm/s (360RPM=6*21cm/s)

// --- ZUSTANDSGRÖSSEN ---
static float global_time_t = 0.0f; 

// Aktuelle Pose des Roboters wird in update_actual_pose() berechnet - hier die Startwerte (Scheitelpunkt 1)
static float x_actual = ELLIPSE_A;
static float y_actual = 0.0f;
static float theta_actual = M_PI/2.0; // Orientierung 90° (rad)

// Zählerstände des letzten Timesteps
static int32_t encR_last = 0;
static int32_t encL_last = 0;

// global, statt nur in der Controller-Funktion - weil ich so neugierig bin...
static float error_x, error_y;  // Abweichungen der Koordinaten von der berechneten Position auf der Ellipse
static float error_lag, v_ref, alpha;  // error_lag > 0: Tumbler ist zu langsam
// v_ref: Sollgeschwindigkeit in cm/s, alpha: Winkelfehler (rad)
static float theta_ref = M_PI/2.0;  // Init/Startwert ist auch im Scheitelpunkt 1

void InitIO()
{
  stdby.Init(); stdby.Set(1);
  // PWM-Init in dieser Reihenfolge:
  motR.inv = true;
  Motor::InitTimer(); motR.Init(); motL.Init(); Motor::StartTimer();
  motR.setPow2(0.0); motL.setPow2(0.0); // Beide Motoren sicherheitshalber auf 0
  encL.Init(); encR.Init();
  encR.inv = true; // Die .inv Flags kann man immer und überall setzen
}

void update_actual_pose() {
    float delta_ticks_R = (float)(encR.cnt - encR_last);
    float delta_ticks_L = (float)(encL.cnt - encL_last);

    float ds_R = delta_ticks_R / TICKS_PER_CM; // inkrementelle Bewegung (Ticks in cm umrechnen)
    float ds_L = delta_ticks_L / TICKS_PER_CM;
    
    float ds_center = (ds_R + ds_L) / 2.0f; // Bewegung des Mittelpunkts und Drehung
    float d_theta = (ds_R - ds_L) / WHEEL_BASE; // w = (vr - vl)/b

    // Pose aktualisieren (Euler-Integration, vereinfacht)      
    x_actual += ds_center * cosf(theta_actual);
    y_actual += ds_center * sinf(theta_actual);        
    theta_actual += d_theta;  // Winkel aufsummieren und ...
    if (theta_actual > M_PI) theta_actual -= 2.0f * M_PI; // ...auf +/- pi korrigieren
    if (theta_actual < -M_PI) theta_actual += 2.0f * M_PI; 

    encR_last = encR.cnt;     // Encoder-Werte für nächsten Schritt speichern
    encL_last = encL.cnt;
}

void run_controller(void* arg) {
    update_actual_pose();

    float phi = PHI_DOT * global_time_t;
    float x_ref = ELLIPSE_A * cosf(phi);
    float y_ref = ELLIPSE_B * sinf(phi);

    error_x = x_ref - x_actual; // Fehlervektor E
    error_y = y_ref - y_actual;

    // inverse Kinematik der Ellipse
    // Wir brauchen die Vorwärtsgeschwindigkeit (v_ref:Ableitung nach der Zeit)
    float dx_dt_ref = -ELLIPSE_A * sin(phi) * PHI_DOT;
    float dy_dt_ref = ELLIPSE_B * cos(phi) * PHI_DOT;
    v_ref = sqrtf(dx_dt_ref * dx_dt_ref + dy_dt_ref * dy_dt_ref);

    // Normierter Tangentialvektor u_T
    float u_Tx = dx_dt_ref / v_ref;
    float u_Ty = dy_dt_ref / v_ref;

    // tangentialer Fehler = Skalarprodukt von E und Einheitstangentialvektor (normierte Länge)
    // Lagenfehler (error_lag) mit Vorzeichen (Projektion des Abstandsvektors P_ref->P_actual auf u_T)
    // Positive Projektion -> P_ref liegt vor P_actual (Roboter ist zu langsam)
    // Negative Projektion -> P_ref liegt hinter P_actual (Roboter ist zu schnell)
    error_lag = error_x * u_Tx + error_y * u_Ty;

    float v_correction = KP_LINEAR * error_lag;  // Lineare Geschw.-Korrektur in Richtung des Ziels    

    theta_ref = atan2f(dy_dt_ref, dx_dt_ref); // Wir brauchen den Winkel der Tangente an die Ellipse=tan(Geschwindigkeitsvektor)
    alpha = theta_ref - theta_actual;  // Winkelfehler
    if (alpha > M_PI) alpha -= 2.0f * M_PI; // auf +/- pi korrigieren
    if (alpha < -M_PI) alpha += 2.0f * M_PI;    
    
    float omega_correction = KP_ANGULAR * alpha / TIMESTEP_DT; // Winkelgeschw.-Korrektur

    // Gewünschte Lineargeschwindigkeit in cm/s = Basisgeschwindigkeit der Bahn + Korrektur
    float v_command = v_ref*5.3 + v_correction;  // experimentell gefundener Faktor - sollte eigentlich 1 sein!!! (check i net)
     
    // inverse Kinematik für Radgeschwindigkeiten
    float v_R = (v_command + (WHEEL_BASE * omega_correction)) / 2.0; // siehe Kursunterlagen "Inverse Kinem."
    float v_L = (v_command - (WHEEL_BASE * omega_correction)) / 2.0;
    
    // Normalisierung, Limitierung und Ansteuerung der Motoren
    float power_R = fminf(fmaxf(v_R / MAX_SPEED, -MAX_POWER), MAX_POWER); // v im verhältnis zu MAX_SPEED
    float power_L = fminf(fmaxf(v_L / MAX_SPEED, -MAX_POWER), MAX_POWER);
    motR.setPow2(power_R);
    motL.setPow2(power_L);

    global_time_t += TIMESTEP_DT; // Zeit aktualisieren
    if (global_time_t>CYCLE_TIME) {
      global_time_t = 0.0; // Reset nach einem vollen Umlauf
      x_actual = ELLIPSE_A;
      y_actual = 0.0f;
      theta_actual = M_PI/2.0; // Orientierung 90° (rad)
      encL.cnt = encR.cnt = 0;
      encL_last = encR_last = 0;
    }
}

extern "C" void app_main(void)
{
  InitIO();
  printf("init done\n");
  encL.cnt = encR.cnt = 0; // Hier erfolgt das Nullen der Encoder

  do {
    MyDelay(100);
  } while (btn.get() == 0);
  
  const esp_timer_create_args_t timer_args = {
        .callback = &run_controller, // Timer ruft jetzt den Regler auf
        .name = "kinematics_controller"
  };
  esp_timer_handle_t timer_handle;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    
  // Timer starten (50 ms = 50000 µs)
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 50000));
    
  printf("Positionsregelung gestartet.\n");
  while(1) {
    printf("t:%.2f X:%.2f Y:%.2f T:%.2f\n", global_time_t, x_actual, y_actual, theta_actual);
    printf("   v_ref:%.2f        t:%.2f\n", v_ref, theta_ref);
    printf("err_x: %.2f err_y: %.2f\n", error_x, error_y);
    printf("e_d: %.2f e_o: %.2f\n", error_lag, alpha);
    printf("-\n");
    //printf("L:%li R:%li\n", encL.cnt, encR.cnt);
    MyDelay(500);
  }
}