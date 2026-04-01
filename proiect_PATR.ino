#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <timers.h>
#include <Servo.h>

// Pini
#define PIN_TRIG_S1 8
#define PIN_ECHO_S1 7
#define PIN_TRIG_S2 10
#define PIN_ECHO_S2 6
#define PIN_SERVO   9

// Parametri
const float PRAG_CM = 5.0f;
const float HYST_CM = 1.0f;          // histerezis (la >6cm)

// Cerințele de timp
const TickType_t TAU_MIN = pdMS_TO_TICKS(1000);  // 1s: garda de siguranță după S1
const TickType_t TAU_MAX = pdMS_TO_TICKS(5000);  // 5s: timp maxim de staționare la S2

const uint32_t TAU_OPEN_MS  = 30;             // timp deschidere liniară
const uint32_t TAU_CLOSE_MS = 30;             // timp închidere liniară

// Unghiuri
const int ANGLE_OPEN  = 90;    // Deschis (Utilitate)
const int ANGLE_CLOSE = 180;   // Închis (Siguranță)

// Evenimente
typedef enum {
  EVT_S1_PRESENT,        // S1 < PRAG_CM
  EVT_S2_PRESENT,        // S2 < PRAG_CM
  EVT_S2_CLEAR,          // S2 > (PRAG_CM + HYST_CM)
  EVT_TAU_MIN_EXPIRED,   // garda de 1s s-a încheiat
  EVT_TAU_MAX_EXPIRED,   // staționare prelungită la S2
  EVT_OPEN_DONE,         // confirmare Servo deschis
  EVT_CLOSE_DONE         // confirmare Servo închis
} Event_t;

// Comenzi servo (pentru mișcare liniară)
typedef struct {
  int targetAngle;
  uint32_t durationMs;
} ServoCmd_t;

// Obiectele utilizate
static QueueHandle_t qEvents;
static QueueHandle_t qServo;

static TimerHandle_t tTauMax;
static TimerHandle_t tTauMin;
static TimerHandle_t tOpenDone;
static TimerHandle_t tCloseDone;

static Servo barrier;

// Stări
static volatile bool s2Present = false;           // mașina este în zona S2
static volatile bool tauMinActive = false;        // suntem în garda obligatorie de TAU_MIN
static volatile bool ignoreUntilClearAfterTauMax = false; // pentru recuperare după TAU_MAX

// Citire distanță senzor ultrasonic
static float readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin, LOW);
  // blocare necesara
  unsigned long dur = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) / 2.0f;
}

// Callback-uri pentru timere (toate Callback-urile trimit evenimente în coada qEvents, în mod asincron)
static void cbTauMax(TimerHandle_t) {
  Event_t ev = EVT_TAU_MAX_EXPIRED;
  xQueueSend(qEvents, &ev, 0);
}
static void cbTauMin(TimerHandle_t) {
  Event_t ev = EVT_TAU_MIN_EXPIRED;
  xQueueSend(qEvents, &ev, 0);
}
static void cbOpenDone(TimerHandle_t) {
  Event_t ev = EVT_OPEN_DONE;
  xQueueSend(qEvents, &ev, 0);
}
static void cbCloseDone(TimerHandle_t) {
  Event_t ev = EVT_CLOSE_DONE;
  xQueueSend(qEvents, &ev, 0);
}

// Task-ul servomotorului (mișcare liniară)
static void taskServo(void*) {
  barrier.attach(PIN_SERVO);
  barrier.write(ANGLE_OPEN); // inițial deschis

  ServoCmd_t cmd;
  while (1) {
    if (xQueueReceive(qServo, &cmd, portMAX_DELAY) == pdPASS) {
      int start = barrier.read();
      int target = cmd.targetAngle;

      uint32_t T = cmd.durationMs;
      if (T < 5) T = 5;

      const uint32_t stepMs = 1;
      uint32_t steps = T / stepMs;
      if (steps < 5) steps = 5;

      // mișcarea liniară
      for (uint32_t i = 1; i <= steps; i++) {
        int ang = start + (int)((long)(target - start) * (long)i / (long)steps);
        barrier.write(ang);
        vTaskDelay(pdMS_TO_TICKS(stepMs));
      }
      barrier.write(target);
    }
  }
}

// Helpers pentru funcții
static void requestClose() {
  ServoCmd_t c{ANGLE_CLOSE, TAU_CLOSE_MS};
  xQueueSend(qServo, &c, 0);
  xTimerStop(tOpenDone, 0);
  xTimerStop(tCloseDone, 0);
  xTimerStart(tCloseDone, 0);
}

static void requestOpen() {
  ServoCmd_t c{ANGLE_OPEN, TAU_OPEN_MS};
  xQueueSend(qServo, &c, 0);
  xTimerStop(tOpenDone, 0);
  xTimerStop(tCloseDone, 0);
  xTimerStart(tOpenDone, 0);
}

// Task de monitorizare a senzorilor
static void taskSensors(void*) {
  pinMode(PIN_TRIG_S1, OUTPUT);
  pinMode(PIN_ECHO_S1, INPUT);
  pinMode(PIN_TRIG_S2, OUTPUT);
  pinMode(PIN_ECHO_S2, INPUT);

  bool s1Prev = false;
  bool s2PrevPresent = false;

  while (1) {
    float d1 = readDistanceCm(PIN_TRIG_S1, PIN_ECHO_S1);
    float d2 = readDistanceCm(PIN_TRIG_S2, PIN_ECHO_S2);

    bool s1 = (d1 > 0 && d1 < PRAG_CM);

    bool s2PresentNow = (d2 > 0 && d2 < PRAG_CM);
    bool s2ClearNow   = (d2 < 0) || (d2 > (PRAG_CM + HYST_CM)); // S2 cu histerezis

    // S1: eveniment pe front pozitiv (apropierea unui tren)
    if (s1 && !s1Prev) {
      Event_t ev = EVT_S1_PRESENT;
      xQueueSend(qEvents, &ev, 0);
    }
    s1Prev = s1;

    // S2: prezent pe front pozitiv
    if (s2PresentNow && !s2PrevPresent) {
      Event_t ev = EVT_S2_PRESENT;
      xQueueSend(qEvents, &ev, 0);
      s2PrevPresent = true;
    }

    // S2: "clear" pe front pozitiv
    if (s2ClearNow && s2PrevPresent) {
      Event_t ev = EVT_S2_CLEAR;
      xQueueSend(qEvents, &ev, 0);
      s2PrevPresent = false;
    }

    // afișarea distanțelor
    Serial.print("S1="); Serial.print(d1, 1); Serial.print("cm  ");
    Serial.print("S2="); Serial.print(d2, 1); Serial.println("cm");
    
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}

// Task-ul de control
static void taskControl(void*) {
  Event_t ev;

  Serial.println("INIT: Bariera OPEN.");
  Serial.println("Reguli: S1 -> inchide + tine inchis TAU_MIN=1s indiferent. Dupa TAU_MIN: daca S2 prezent, tine inchis pana pleaca sau TAU_MAX=5s.");

  while (1) {
    if (xQueueReceive(qEvents, &ev, portMAX_DELAY) == pdPASS) {

      switch (ev) {

        // S1: închiderea de siguranță, garda tau_min
        case EVT_S1_PRESENT:
          Serial.println("S1 < 5cm -> INCHID bariera si respect TAU_MIN=1s.");
          requestClose();

          tauMinActive = true;
          xTimerStop(tTauMin, 0);
          xTimerChangePeriod(tTauMin, TAU_MIN, 0);
          xTimerStart(tTauMin, 0);
          break;

        // S2: tren în zona de control
        case EVT_S2_PRESENT:
          if (ignoreUntilClearAfterTauMax) {
            Serial.println("IGNOR: TAU_MAX a expirat, astept S2 CLEAR.");
            break;
          }

          s2Present = true;
          Serial.println("S2 < 5cm -> masina in zona: bariera trebuie inchisa, pornesc TAU_MAX=5s.");
          requestClose();

          // pornește TAU_MAX doar dacă nu este deja activat de expirarea TAU_MIN
          if (!tauMinActive) { 
            xTimerStop(tTauMax, 0);
            xTimerChangePeriod(tTauMax, TAU_MAX, 0);
            xTimerStart(tTauMax, 0);
          }
          break;

        // S2: trenul a plecat
        case EVT_S2_CLEAR:
          Serial.println("S2 > 5cm -> masina a plecat.");
          s2Present = false;
          ignoreUntilClearAfterTauMax = false;
          xTimerStop(tTauMax, 0);

          if (!tauMinActive) {
            Serial.println("TAU_MIN nu e activ -> RIDIC bariera.");
            requestOpen();
          } else {
            Serial.println("TAU_MIN activ -> bariera ramane inchisa pana expira TAU_MIN.");
          }
          break;

        // expirarea TAU_MIN
        case EVT_TAU_MIN_EXPIRED:
          tauMinActive = false;
          Serial.println("TAU_MIN expirat -> verific S2.");

          if (s2Present) {
            Serial.println("S2 prezent -> raman inchis si pornesc TAU_MAX.");
            if (xTimerIsTimerActive(tTauMax) == pdFALSE) {
              xTimerChangePeriod(tTauMax, TAU_MAX, 0);
              xTimerStart(tTauMax, 0);
            }
          } else {
            Serial.println("S2 nu e prezent -> RIDIC bariera.");
            requestOpen();
          }
          break;

        // expirarea TAU_MAX: forțarea plecării trenului
        case EVT_TAU_MAX_EXPIRED:
          if (s2Present) {
            Serial.println("Masina a stat TAU_MAX=5s la S2 -> RIDIC bariera.");
            requestOpen();

            // setare flag pentru a ignora reinchiderea imediată
            ignoreUntilClearAfterTauMax = true;
            s2Present = false; 
          }
          break;

        // confirmare
        case EVT_CLOSE_DONE:
          Serial.println("--- Barieră CLOSED complet (30ms).");
          break;

        case EVT_OPEN_DONE:
          Serial.println("--- Barieră OPEN complet (30ms).");
          break;

        default:
          break;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  // creare cozi
  qEvents = xQueueCreate(12, sizeof(Event_t));
  qServo  = xQueueCreate(5, sizeof(ServoCmd_t));

  // creare timere
  tTauMin    = xTimerCreate("tMin", TAU_MIN, pdFALSE, NULL, cbTauMin);
  tTauMax    = xTimerCreate("tMax", TAU_MAX, pdFALSE, NULL, cbTauMax);
  tOpenDone  = xTimerCreate("tOpen", pdMS_TO_TICKS(TAU_OPEN_MS), pdFALSE, NULL, cbOpenDone);
  tCloseDone = xTimerCreate("tClose", pdMS_TO_TICKS(TAU_CLOSE_MS), pdFALSE, NULL, cbCloseDone);

  // creare task-uri (prioritate: CTRL > SERVO > SENS)
  xTaskCreate(taskServo,   "SERVO",  256, NULL, 2, NULL);
  xTaskCreate(taskSensors, "SENS",   384, NULL, 1, NULL);
  xTaskCreate(taskControl, "CTRL",   512, NULL, 3, NULL);

  vTaskStartScheduler();
}

void loop() {}