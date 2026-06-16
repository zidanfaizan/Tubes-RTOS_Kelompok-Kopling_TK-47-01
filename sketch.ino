#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- Pin Definitions ---
// Utara (Lane A)
#define N_RED     25
#define N_YEL     26
#define N_GRN     27

// Timur (Lane B)
#define E_RED     19
#define E_YEL     18
#define E_GRN     5

// Selatan (Lane C)
#define S_RED     17
#define S_YEL     16
#define S_GRN     4

// Barat (Lane D)
#define W_RED     32
#define W_YEL     33
#define W_GRN     15

// --- Pin Input ---
#define BTN_PEDESTRIAN 13
#define BTN_EMRG_N     12
#define BTN_EMRG_E     21
#define BTN_EMRG_S     22
#define BTN_EMRG_W     23
#define BTN_VEHICLE    14

// --- FreeRTOS Handles ---
QueueHandle_t vehicleQueue;
SemaphoreHandle_t emergencySemaphore;
SemaphoreHandle_t pedSemaphore;
SemaphoreHandle_t serialMutex;

// --- Global Variables ---
volatile bool isEmergency = false;
volatile int emergencyDirection = -1; // 0: N, 1: E, 2: S, 3: W
volatile bool pedRequest = false;
volatile bool jamRequest = false;

// Array pin untuk mempermudah perulangan
const int redPins[4] = {N_RED, E_RED, S_RED, W_RED};
const int yelPins[4] = {N_YEL, E_YEL, S_YEL, W_YEL};
const int grnPins[4] = {N_GRN, E_GRN, S_GRN, W_GRN};
const char* laneNames[4] = {"Utara", "Timur", "Selatan", "Barat"};

void setAllRed() {
    for(int i=0; i<4; i++) {
        digitalWrite(redPins[i], HIGH);
        digitalWrite(yelPins[i], LOW);
        digitalWrite(grnPins[i], LOW);
    }
}

void setLaneGreen(int lane) {
    for(int i=0; i<4; i++) {
        if (i == lane) {
            digitalWrite(redPins[i], LOW);
            digitalWrite(yelPins[i], LOW);
            digitalWrite(grnPins[i], HIGH);
        } else {
            digitalWrite(redPins[i], HIGH);
            digitalWrite(yelPins[i], LOW);
            digitalWrite(grnPins[i], LOW);
        }
    }
}

void setLaneYellow(int lane) {
    for(int i=0; i<4; i++) {
        if (i == lane) {
            digitalWrite(redPins[i], LOW);
            digitalWrite(yelPins[i], HIGH);
            digitalWrite(grnPins[i], LOW);
        } else {
            digitalWrite(redPins[i], HIGH);
            digitalWrite(yelPins[i], LOW);
            digitalWrite(grnPins[i], LOW);
        }
    }
}

// --- Implementasi ISR ---
volatile unsigned long lastEmrgTime = 0;
const unsigned long DEBOUNCE_MS = 500; // 500ms anti-bounce

void IRAM_ATTR emrgISR_N() { if(millis() - lastEmrgTime > DEBOUNCE_MS) { lastEmrgTime = millis(); emergencyDirection = 0; BaseType_t xHP = pdFALSE; xSemaphoreGiveFromISR(emergencySemaphore, &xHP); if(xHP) portYIELD_FROM_ISR(); } }
void IRAM_ATTR emrgISR_E() { if(millis() - lastEmrgTime > DEBOUNCE_MS) { lastEmrgTime = millis(); emergencyDirection = 1; BaseType_t xHP = pdFALSE; xSemaphoreGiveFromISR(emergencySemaphore, &xHP); if(xHP) portYIELD_FROM_ISR(); } }
void IRAM_ATTR emrgISR_S() { if(millis() - lastEmrgTime > DEBOUNCE_MS) { lastEmrgTime = millis(); emergencyDirection = 2; BaseType_t xHP = pdFALSE; xSemaphoreGiveFromISR(emergencySemaphore, &xHP); if(xHP) portYIELD_FROM_ISR(); } }
void IRAM_ATTR emrgISR_W() { if(millis() - lastEmrgTime > DEBOUNCE_MS) { lastEmrgTime = millis(); emergencyDirection = 3; BaseType_t xHP = pdFALSE; xSemaphoreGiveFromISR(emergencySemaphore, &xHP); if(xHP) portYIELD_FROM_ISR(); } }

volatile unsigned long lastPedTime = 0;
void IRAM_ATTR pedestrianISR() {
    if(millis() - lastPedTime > DEBOUNCE_MS) {
        lastPedTime = millis();
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(pedSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
    }
}

// --- Implementasi TrafficLightTask ---
void TrafficLightTask(void *pvParameters) {
    int currentLane = 0; // 0: Utara, 1: Timur, 2: Selatan, 3: Barat
    
    while (1) {
        if (isEmergency) {
            // Biarkan EmergencyTask yang mengontrol lampu
            vTaskDelay(100 / portTICK_PERIOD_MS);
        } else {
            // Cek Queue dari VehicleTask
            int trafficStatus = 0;
            if (xQueueReceive(vehicleQueue, &trafficStatus, 0) == pdPASS) {
                if(trafficStatus == 1) {
                    jamRequest = true; // Ada kemacetan
                }
            }

            // Durasi dinamis (Jika macet, durasi hijau jadi 6 detik, jika tidak 3 detik)
            int greenDuration = jamRequest ? 60 : 30; // 100ms multiplier
            if(jamRequest) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                printf("[Traffic Task] Sensor aktif: Waktu Hijau diperpanjang untuk jalur %s!\n", laneNames[currentLane]);
                xSemaphoreGive(serialMutex);
                jamRequest = false; // Reset setelah digunakan
            }

            // Lampu Hijau Jalur Aktif
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            printf("[Traffic Task] GREEN - Lane %s\n", laneNames[currentLane]);
            xSemaphoreGive(serialMutex);
            setLaneGreen(currentLane);
            
            for (int i=0; i<greenDuration; i++) {
                if(isEmergency) break; // Emergency potong seketika
                // Pedestrian request tidak memotong hijau, ia menunggu kuning
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            if(isEmergency) continue;
            
            // Lampu Kuning Jalur Aktif
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            printf("[Traffic Task] YELLOW - Lane %s\n", laneNames[currentLane]);
            xSemaphoreGive(serialMutex);
            setLaneYellow(currentLane);
            
            for (int i=0; i<10; i++) {
                if(isEmergency) break;
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            if(isEmergency) continue;

            // Jika ada pejalan kaki, sisipkan fase pejalan kaki
            if (pedRequest) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                printf("[Traffic Task] SCRAMBLE CROSSING - Semua Merah untuk Pejalan Kaki\n");
                xSemaphoreGive(serialMutex);
                setAllRed();
                vTaskDelay(5000 / portTICK_PERIOD_MS); // 5 Detik fase pejalan kaki
                pedRequest = false;
            }
            
            // Lanjut ke jalur berikutnya
            currentLane = (currentLane + 1) % 4;
        }
    }
}

// --- Implementasi VehicleTask ---
void VehicleTask(void *pvParameters) {
    int jamSignal = 1;
    bool btnLastState = HIGH;
    
    while(1) {
        bool btnState = digitalRead(BTN_VEHICLE);
        // Deteksi penekanan tombol sensor kemacetan
        if (btnState == LOW && btnLastState == HIGH) {
            xSemaphoreTake(serialMutex, portMAX_DELAY);
            printf("[Vehicle Task] Sensor Kemacetan Ditekan!\n");
            xSemaphoreGive(serialMutex);

            xQueueSend(vehicleQueue, &jamSignal, portMAX_DELAY);
        }
        btnLastState = btnState;
        vTaskDelay(50 / portTICK_PERIOD_MS); // Debounce
    }
}

// --- Implementasi PedestrianTask ---
void PedestrianTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(pedSemaphore, portMAX_DELAY) == pdTRUE) {
            if (!isEmergency && !pedRequest) {
                pedRequest = true;
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                printf("[Pedestrian Task] Permintaan Menyeberang Diterima. Menunggu siklus hijau selesai...\n");
                xSemaphoreGive(serialMutex);
            }
        }
    }
}

// --- Implementasi EmergencyTask ---
void EmergencyTask(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(emergencySemaphore, portMAX_DELAY) == pdTRUE) {
            isEmergency = true;
            pedRequest = false; // Batalkan pejalan kaki jika ada darurat
            
            // Simpan direction ke variabel lokal agar aman
            int dir = emergencyDirection; 
            
            // Validasi arah, jika tidak valid (-1) karena bounce, hiraukan
            if (dir >= 0 && dir <= 3) {
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                printf("[ISR] AMBULANS DETECTED DARI JALUR %s!\n", laneNames[dir]);
                printf("[Emergency Task] Flushing Phase Activated! Menghijaukan jalur %s dan memerahkan yang lain.\n", laneNames[dir]);
                xSemaphoreGive(serialMutex);
                
                setLaneGreen(dir);
                vTaskDelay(8000 / portTICK_PERIOD_MS); // Tahan merah semua selama 8 detik
                
                xSemaphoreTake(serialMutex, portMAX_DELAY);
                printf("[Emergency Task] Darurat Selesai, kembali ke normal.\n");
                xSemaphoreGive(serialMutex);
            }
            
            isEmergency = false;
            emergencyDirection = -1;
        }
    }
}

// --- Implementasi LoggingTask ---
void LoggingTask(void *pvParameters) {
    UBaseType_t stackLeft; 
    while (1) {
        xSemaphoreTake(serialMutex, portMAX_DELAY);
        stackLeft = uxTaskGetStackHighWaterMark(NULL);
        // printf("[Logging Task] System Running, Memori Bebas: %lu words\n", (unsigned long)stackLeft);
        xSemaphoreGive(serialMutex);
        
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Log lebih jarang agar tidak spam
    }
}

// --- Implementasi Main Program ---
void setup() {
    Serial.begin(115200);

    for(int i=0; i<4; i++) {
        pinMode(redPins[i], OUTPUT);
        pinMode(yelPins[i], OUTPUT);
        pinMode(grnPins[i], OUTPUT);
    }
    
    setAllRed();
    
    vehicleQueue = xQueueCreate(5, sizeof(int));
    emergencySemaphore = xSemaphoreCreateBinary();
    pedSemaphore = xSemaphoreCreateBinary();
    serialMutex = xSemaphoreCreateMutex();

    pinMode(BTN_EMRG_N, INPUT_PULLUP);
    pinMode(BTN_EMRG_E, INPUT_PULLUP);
    pinMode(BTN_EMRG_S, INPUT_PULLUP);
    pinMode(BTN_EMRG_W, INPUT_PULLUP);
    pinMode(BTN_PEDESTRIAN, INPUT_PULLUP);
    pinMode(BTN_VEHICLE, INPUT_PULLUP); // Tombol ini tidak pakai ISR, tapi polling

    attachInterrupt(digitalPinToInterrupt(BTN_EMRG_N), emrgISR_N, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_EMRG_E), emrgISR_E, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_EMRG_S), emrgISR_S, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_EMRG_W), emrgISR_W, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_PEDESTRIAN), pedestrianISR, FALLING);

    xTaskCreate(TrafficLightTask, "Traffic Task", 2048, NULL, 3, NULL);
    xTaskCreate(VehicleTask, "Vehicle Task", 2048, NULL, 2, NULL);
    xTaskCreate(PedestrianTask, "Pedestrian Task", 2048, NULL, 2, NULL);
    xTaskCreate(EmergencyTask, "Emergency Task", 2048, NULL, 4, NULL);
    xTaskCreate(LoggingTask, "Logging Task", 2048, NULL, 1, NULL);
}

void loop() {
    // Memberikan waktu jeda pada core utama agar simulator Wokwi tidak lag/hang
    delay(10);
}
