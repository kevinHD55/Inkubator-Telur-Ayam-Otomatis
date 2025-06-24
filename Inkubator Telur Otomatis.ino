#include <DHT.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <AS5600.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h> // WARNING: Library ini mungkin tidak sepenuhnya kompatibel dengan ESP32, namun seringkali berfungsi.

// --- PIN DEFINITIONS ---
#define DHTPIN 4
#define DHTTYPE DHT22
#define HEATER_PIN 19
#define FAN_PIN 18
#define FAN_PIN_2 26
#define MISTMAKER_PIN 12
#define SERVO_PIN 14
#define LIMIT_SWITCH_PIN 27
#define TRIAC_PIN 13

#define HEATER_PWM_CHANNEL 10
#define PWM_FREQUENCY 510
#define PWM_RESOLUTION 8
#define MAX_PWM_DUTY ((1 << PWM_RESOLUTION) - 1)

#define I2C_SDA_PIN_0 21
#define I2C_SCL_PIN_0 22

#define I2C_SDA_PIN_1 32
#define I2C_SCL_PIN_1 33


// --- EEPROM ADDRESSES ---
#define EEPROM_ADDR_DATE 0             // 3 bytes for Day, Month, Year
#define EEPROM_ADDR_INCUBATION_STARTED 3 // 1 byte for boolean status (0 or 1)
#define EEPROM_ADDR_CENTER_RAW 4       // 2 bytes for int (AS5600 center raw value)
#define EEPROM_ADDR_LAST_DAY 6         // 1 byte for last saved currentDay (max 21)


#define EEPROM_ADDR_TEMP_HISTORY        (EEPROM_ADDR_LAST_DAY + 1) // Mulai setelah lastSavedDay
#define EEPROM_ADDR_HUM_HISTORY         (EEPROM_ADDR_TEMP_HISTORY + (sizeof(float) * 21))
#define EEPROM_ADDR_RACK_ANGLE_HISTORY  (EEPROM_ADDR_HUM_HISTORY + (sizeof(float) * 21))
#define EEPROM_ADDR_VENT_ANGLE_HISTORY  (EEPROM_ADDR_RACK_ANGLE_HISTORY + (sizeof(int) * 21))


// Tambahkan definisi EEPROM_SIZE ini di bagian atas file Anda
const int EEPROM_SIZE = EEPROM_ADDR_VENT_ANGLE_HISTORY + (sizeof(int) * 21);

// Total EEPROM size needed: Alamat tertinggi (6) + ukuran variabel (1 byte) = 7 byte.
// EEPROM.begin() akan disesuaikan secara otomatis saat kompilasi berdasarkan alamat tertinggi + 1.

// --- GLOBAL OBJECTS ---
DHT dht(DHTPIN, DHTTYPE);
Servo myServo;
AS5600 encoder(&Wire1); // AS5600 menggunakan Wire1 (I2C Bus 1)
RTC_DS3231 rtc;         // RTC menggunakan Wire (I2C Bus 0)
AsyncWebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD menggunakan Wire (I2C Bus 0)

// --- WIFI CREDENTIALS ---
const char* ssid = "MINDIL";
const char* password = "mewah12345";

// --- GLOBAL VARIABLES ---
float suhu = NAN;
float kelembapan = NAN;
bool heaterState = false;
bool mistState = false;
bool isAutoMode = false;
bool isHeaterOn = false;        // Status heater ON/OFF
bool isMistMakerOn = false;     // Status mist maker ON/OFF
bool isIncubationStarted = false; // Variabel global untuk melacak status inkubasi
bool isRackMotorMoving = false;
int currentDay = 1;               // Hari inkubasi saat ini
int lastSavedDay = 1;             // Hari inkubasi terakhir yang disimpan di EEPROM
bool isWiFiConnected = false;
bool webServerStarted = false;

bool manualRackRotationActive = false; // Status apakah motor sedang berputar karena perintah manual
unsigned long lastManualRotateMillis = 0; // Waktu terakhir putar manual dimulai
const unsigned long MANUAL_ROTATE_DURATION = 7000; // Durasi putar manual dalam ms (sesuaikan dengan RotateByTimer)


int currentServoPos = 40;
int servoAngleByDay = 40;

unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL = 10000; // 10 detik

int targetPwmDuty = MAX_PWM_DUTY;
int currentPwmDuty = MAX_PWM_DUTY;
const int PWM_STEP = 10; // Langkah perubahan PWM
unsigned long lastPwmUpdateTime = 0;
const unsigned long PWM_UPDATE_INTERVAL = 235; // Update PWM setiap 20ms untuk transisi halus

DateTime tanggalMulai;
int overrideDay = 0; // Digunakan untuk override hari inkubasi secara manual

unsigned long lastStatusPrint = 0;
const unsigned long STATUS_PRINT_INTERVAL = 2000; // Print status ke Serial setiap 2 detik

const int NUM_DHT_READINGS = 10; // Jumlah pembacaan untuk filter DHT
float dhtTempReadings[NUM_DHT_READINGS];
float dhtHumReadings[NUM_DHT_READINGS];
int dhtReadingIndex = 0;
bool dhtReadingsInitialized = false;

//int rawDiffReadings[NUM_ANGLE_READINGS];
int angleReadingIndex = 0;
bool angleReadingsInitialized = false;

unsigned long lastWiFiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_INTERVAL = 30000; // Coba reconnect WiFi setiap 30 detik

unsigned long lastLcdUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 1000; // Update LCD setiap 5 detik

// --- NEW GLOBAL ARRAYS FOR DAILY HISTORY ---
float dailyTemperatures[21]; // Array untuk menyimpan suhu harian
float dailyHumidities[21];   // Array untuk menyimpan kelembaban harian

int dailyVentAngles[21];     // Array untuk menyimpan sudut ventilasi harian

// --- FUNCTION DECLARATIONS ---
void matikanSemua();
void tampilkanStatus();
bool bolehGerakKe(int targetAngle);
void connectToWiFi();



float readFilteredTemperature();
float readFilteredHumidity();

void simpanStatusInkubasi(bool started);
bool bacaStatusInkubasi();
void simpanTanggalMulai(DateTime dt);
DateTime bacaTanggalMulai();
void simpanHariTerakhir(int day);
int bacaHariTerakhir();
void hitungHariInkubasi(); // Logika di dalamnya akan disesuaikan
void setTanggalMulaiManualHariKe(int hariKe);
void scanI2CBus(TwoWire *wireObj, const char* busName, int sdaPin, int sclPin);

// NEW FUNCTION DECLARATIONS
void saveDailyHistory(int dayIndex, float temp, float hum,  int ventAngle);
void loadDailyHistory();
void clearDailyHistory(); // Fungsi untuk mereset data historis (saat inkubasi berhenti)


//UNTUK RAK TELUR
#define HALL_DO_PIN 23
#define TRIAC_PIN 13
bool motorNyala = false;
unsigned long lastStartTime = 0;
#define ROT_NONE 0
#define ROT_ZERO 1
#define ROT_SKEW 2
static volatile int rotateState = ROT_NONE;
static volatile int rotateStateTarget = ROT_NONE;
int sudutZero = 0;
float sudutZeroDerajat = 0;
float sudutSekarang = 0;
float sudutSekarangDerajat = 0;
bool rotateFindZero = false;
uint32_t tRotate = 0;
uint32_t jedaPutarRak = 14400000;//5000;//14400000; //14400000 = 4 jam

bool putarRakByTimer = true; //kalau false coba pakai sensor sudut. kalau true, dibatasi sama sudut dan timer
uint32_t tStopPutar = 0;

uint32_t lastTickUpdate = 0;

bool detachServoFlag = false;
uint32_t detachServoTick = 0;
uint32_t skipTick = 0;

// --- FUNCTION IMPLEMENTATIONS ---

void scanI2CBus(TwoWire *wireObj, const char* busName, int sdaPin, int sclPin) {
  byte error, address;
  int nDevices = 0;
  Serial.printf("\n--- Scanning I2C Bus %s (SDA=%d, SCL=%d) ---\n", busName, sdaPin, sclPin);
  Serial.println("    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");

  for (int i = 0; i < 128; i += 16) {
    Serial.printf("%02x ", i);
    for (address = 0; address < 16; address++) {
      if (i + address < 8) { // Alamat 0-7 dicadangkan, tidak perlu scan
        Serial.print("   ");
        continue;
      }
      wireObj->beginTransmission(i + address);
      error = wireObj->endTransmission();

      if (error == 0) {
        Serial.printf("%02x ", i + address);
        nDevices++;
      } else if (error == 4) {
        Serial.print("?? "); // Unknown error
      } else {
        Serial.print("-- "); // No device found
      }
    }
    Serial.println();
  }

  if (nDevices == 0) {
    Serial.printf("\nNo I2C devices found on Bus %s.\n", busName);
  } else {
    Serial.printf("\nDone scanning I2C Bus %s. Found %d device(s).\n", busName, nDevices);
  }
  Serial.println("------------------------------------------");
}

void simpanTanggalMulai(DateTime dt) {
  EEPROM.write(EEPROM_ADDR_DATE, dt.day());
  EEPROM.write(EEPROM_ADDR_DATE + 1, dt.month());
  EEPROM.write(EEPROM_ADDR_DATE + 2, dt.year() - 2000);
  EEPROM.commit();
  Serial.printf("Tanggal mulai inkubasi disimpan: %02d-%02d-%04d\n", dt.day(), dt.month(), dt.year());
}

DateTime bacaTanggalMulai() {
  int d = EEPROM.read(EEPROM_ADDR_DATE);
  int m = EEPROM.read(EEPROM_ADDR_DATE + 1);
  int y = EEPROM.read(EEPROM_ADDR_DATE + 2) + 2000;
  // Cek validitas data dasar (contoh: tahun 2000-2099)
  if (d < 1 || d > 31 || m < 1 || m > 12 || y < 2000 || y > 2099) {
    Serial.println("Tanggal mulai di EEPROM tidak valid, menggunakan tanggal default.");
    return DateTime(2025, 5, 20); // Contoh default tanggal
  }
  Serial.printf("Tanggal mulai dibaca dari EEPROM: %02d-%02d-%04d\n", d, m, y);
  return DateTime(y, m, d);
}

void simpanStatusInkubasi(bool started) {
  EEPROM.write(EEPROM_ADDR_INCUBATION_STARTED, started ? 1 : 0);
  EEPROM.commit();
  isIncubationStarted = started; // Pastikan variabel global juga terupdate
  Serial.printf("Status inkubasi disimpan: %s\n", started ? "DIMULAI" : "BELUM DIMULAI");
}

bool bacaStatusInkubasi() {
  byte status = EEPROM.read(EEPROM_ADDR_INCUBATION_STARTED);
  // Jika nilai yang dibaca tidak valid (misal, EEPROM belum pernah ditulis), anggap belum dimulai
  if (status != 0 && status != 1) {
    Serial.println("Status inkubasi di EEPROM tidak valid, dianggap BELUM DIMULAI.");
    return false;
  }
  isIncubationStarted = (status == 1); // Perbarui variabel global
  Serial.printf("Status inkubasi dibaca dari EEPROM: %s\n", isIncubationStarted ? "DIMULAI" : "BELUM DIMULAI");
  return isIncubationStarted;
}

void simpanHariTerakhir(int day) {
  if (day < 1) day = 1; // Pastikan hari tidak kurang dari 1
  if (day > 21) day = 21; // Pastikan hari tidak lebih dari 21
  EEPROM.write(EEPROM_ADDR_LAST_DAY, day);
  EEPROM.commit();
  lastSavedDay = day; // Update variabel global
  Serial.printf("Hari terakhir inkubasi disimpan ke EEPROM: %d\n", day);
}

int bacaHariTerakhir() {
  int day = EEPROM.read(EEPROM_ADDR_LAST_DAY);
  // Jika nilai tidak valid (misal, EEPROM belum pernah ditulis), anggap hari 1
  if (day < 1 || day > 21) {
    Serial.println("Hari terakhir di EEPROM tidak valid, dianggap hari ke-1.");
    return 1;
  }
  lastSavedDay = day; // Update variabel global
  Serial.printf("Hari terakhir dibaca dari EEPROM: %d\n", lastSavedDay);
  return lastSavedDay;
}

void hitungHariInkubasi() {
  // Fungsi ini sekarang lebih fokus pada perhitungan real-time untuk tujuan display/debug jika diperlukan,
  // atau untuk set tanggalMulai saat start baru.
  // currentDay aktual untuk logika utama akan diatur oleh lastSavedDay atau setTanggalMulaiManualHariKe.
  if (overrideDay > 0) {
    // Jika ada override, currentDay diset langsung di setTanggalMulaiManualHariKe
  } else if (!isIncubationStarted) {
    // Hanya hitung dari RTC jika inkubasi belum dimulai.
    // Ini untuk setting awal tanggalMulai atau saat pertama kali siklus dijalankan.
    DateTime now = rtc.now();
    TimeSpan durasi = now - tanggalMulai;
    currentDay = durasi.days() + 1;
    if (currentDay < 1) currentDay = 1;
    if (currentDay > 21) currentDay = 21;
  }
  // Jika isIncubationStarted TRUE dan tidak ada overrideDay, currentDay akan mempertahankan
  // nilai dari lastSavedDay yang dimuat di setup() atau yang terakhir disimpan secara eksplisit.
  // Logika untuk maju hari otomatis ada di loop().
}

void setTanggalMulaiManualHariKe(int hariKe) {
  DateTime now = rtc.now();
  if (hariKe < 1) hariKe = 1;
  if (hariKe > 21) hariKe = 21;
  // Sesuaikan tanggalMulai agar hari saat ini menjadi hariKe
  DateTime newStart = now - TimeSpan(hariKe - 1, 0, 0, 0);
  tanggalMulai = newStart;
  simpanTanggalMulai(tanggalMulai);

  overrideDay = 0; // Membatalkan override hari
  currentDay = hariKe; // Set currentDay langsung ke hari yang diminta
  simpanHariTerakhir(currentDay); // <--- SIMPAN HARI KE EEPROM
  isAutoMode = true; // Langsung setel ke mode otomatis
  simpanStatusInkubasi(true); // <--- TANDAI INKUBASI SUDAH DIMULAI
  tampilkanStatus();
  Serial.printf("Tanggal mulai diset ke %02d-%02d-%04d agar hari sekarang adalah hari ke-%d, mode AUTO aktif.\n",
                tanggalMulai.day(), tanggalMulai.month(), tanggalMulai.year(), hariKe);
}

float readFilteredTemperature() {
  float rawTemp = dht.readTemperature();
  if (isnan(rawTemp)) {
    if (dhtReadingsInitialized || dhtReadingIndex > 0) {
      float sum = 0;
      int count = 0;
      if (dhtReadingsInitialized) {
        for (int i = 0; i < NUM_DHT_READINGS; i++) {
          sum += dhtTempReadings[i];
        }
        count = NUM_DHT_READINGS;
      } else {
        for (int i = 0; i < dhtReadingIndex; i++) {
          sum += dhtTempReadings[i];
        }
        count = dhtReadingIndex;
      }
      if (count > 0) return sum / count;
    }
    return suhu; // Mengembalikan nilai suhu terakhir yang valid jika pembacaan gagal
  }

  dhtTempReadings[dhtReadingIndex] = rawTemp;

  dhtReadingIndex = (dhtReadingIndex + 1) % NUM_DHT_READINGS;

  if (dhtReadingIndex == 0) {
    dhtReadingsInitialized = true;
  }

  float sum = 0;
  int count = 0;
  if (dhtReadingsInitialized) {
    for (int i = 0; i < NUM_DHT_READINGS; i++) {
      sum += dhtTempReadings[i];
    }
    count = NUM_DHT_READINGS;
  } else {
    for (int i = 0; i < dhtReadingIndex; i++) {
      sum += dhtTempReadings[i];
    }
    count = dhtReadingIndex;
  }

  if (count == 0) return rawTemp; // Seharusnya tidak terjadi jika rawTemp valid
  return sum / count;
}

float readFilteredHumidity() {
  float rawHum = dht.readHumidity();
  if (isnan(rawHum)) {
    if (dhtReadingsInitialized || dhtReadingIndex > 0) {
      float sum = 0;
      int count = 0;
      if (dhtReadingsInitialized) {
        for (int i = 0; i < NUM_DHT_READINGS; i++) {
          sum += dhtHumReadings[i];
        }
        count = NUM_DHT_READINGS;
      } else {
        for (int i = 0; i < dhtReadingIndex; i++) {
          sum += dhtHumReadings[i];
        }
        count = dhtReadingIndex;
      }
      if (count > 0) return sum / count;
    }
    return kelembapan; // Mengembalikan nilai kelembapan terakhir yang valid jika pembacaan gagal
  }

  dhtHumReadings[dhtReadingIndex] = rawHum;

  // dhtReadingIndex sudah diupdate di readFilteredTemperature, jadi ini akan digunakan untuk keduanya
  // dhtReadingIndex = (dhtReadingIndex + 1) % NUM_DHT_READINGS; // Tidak perlu di sini lagi, karena sudah di readFilteredTemperature

  float sum = 0;
  int count = 0;
  if (dhtReadingsInitialized) {
    for (int i = 0; i < NUM_DHT_READINGS; i++) {
      sum += dhtHumReadings[i];
    }
    count = NUM_DHT_READINGS;
  } else {
    for (int i = 0; i < dhtReadingIndex; i++) {
      sum += dhtHumReadings[i];
    }
    count = dhtReadingIndex;
  }

  if (count == 0) return rawHum; // Seharusnya tidak terjadi jika rawHum valid
  return sum / count;
}

void bacaSensor() {
  suhu = readFilteredTemperature();
  kelembapan = readFilteredHumidity();
  Serial.printf("Pembacaan DHT (Filtered): Suhu=%.2f, Kelembapan=%.2f\n", suhu, kelembapan);
}

void updateHeaterPWM() {
  unsigned long now = millis();
  if (now - lastPwmUpdateTime >= PWM_UPDATE_INTERVAL) {
    if (suhu < 37.3){
      targetPwmDuty += PWM_STEP;
    }else if (suhu > 37.7){
      targetPwmDuty -= 1;
    }
    if (targetPwmDuty<0) targetPwmDuty = 0;
    if (targetPwmDuty>MAX_PWM_DUTY) targetPwmDuty = MAX_PWM_DUTY;
    if (isAutoMode){
       currentPwmDuty = targetPwmDuty;
    }else{
      currentPwmDuty = 0;
    }
    ledcWrite(HEATER_PWM_CHANNEL, currentPwmDuty);
    lastPwmUpdateTime = now;
  }
}

void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!isWiFiConnected) {
      isWiFiConnected = true;
      Serial.print("WiFi Terhubung! Alamat IP: ");
      Serial.println(WiFi.localIP());
      lcd.clear();
      lcd.print("WiFi Connected!");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      //delay(2000);
      //lcd.clear();
    }
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (isWiFiConnected) {
      isWiFiConnected = false;
      Serial.println("WiFi Terputus. Mencoba menyambung kembali...");
      lcd.clear();
      lcd.print("WiFi Disconnected");
      lcd.setCursor(0, 1);
      lcd.print("Retrying...");
    }

    // Hindari mencoba menyambung terlalu sering jika sudah dalam proses
    if (millis() - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL || lastWiFiReconnectAttempt == 0) {
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      Serial.println("Mencoba menyambungkan WiFi...");
      lastWiFiReconnectAttempt = millis();
    }
  }
}

// --- NEW FUNCTION IMPLEMENTATIONS FOR DAILY HISTORY ---

// Menyimpan satu titik data harian ke EEPROM
void saveDailyHistory(int dayIndex, float temp, float hum,  int ventAngle) {
  if (dayIndex >= 0 && dayIndex < 21) {
    // Simpan ke array global (cache)
    dailyTemperatures[dayIndex] = temp;
    dailyHumidities[dayIndex] = hum;
    dailyVentAngles[dayIndex] = ventAngle;

    // Simpan ke EEPROM
    EEPROM.put(EEPROM_ADDR_TEMP_HISTORY + (dayIndex * sizeof(float)), temp);
    EEPROM.put(EEPROM_ADDR_HUM_HISTORY + (dayIndex * sizeof(float)), hum);
    EEPROM.put(EEPROM_ADDR_VENT_ANGLE_HISTORY + (dayIndex * sizeof(int)), ventAngle);
    EEPROM.commit();
    Serial.printf("Data historis Hari %d disimpan: Suhu=%.2f, RH=%.2f, Rak=%d, Vent=%d\n",
                  dayIndex + 1, temp, hum,  ventAngle);
  } else {
    Serial.println("Indeks hari untuk penyimpanan histori tidak valid.");
  }
}

// Memuat semua data historis dari EEPROM ke array global
void loadDailyHistory() {
  Serial.println("Memuat data historis harian dari EEPROM...");
  for (int i = 0; i < 21; i++) {
    EEPROM.get(EEPROM_ADDR_TEMP_HISTORY + (i * sizeof(float)), dailyTemperatures[i]);
    EEPROM.get(EEPROM_ADDR_HUM_HISTORY + (i * sizeof(float)), dailyHumidities[i]);
    EEPROM.get(EEPROM_ADDR_VENT_ANGLE_HISTORY + (i * sizeof(int)), dailyVentAngles[i]);
    // Validasi data yang dibaca (misal, jika EEPROM kosong awalnya)
    if (isnan(dailyTemperatures[i]) || dailyTemperatures[i] < 0 || dailyTemperatures[i] > 100) dailyTemperatures[i] = 0.0;
    if (isnan(dailyHumidities[i]) || dailyHumidities[i] < 0 || dailyHumidities[i] > 100) dailyHumidities[i] = 0.0;
    if (dailyVentAngles[i] < 0 || dailyVentAngles[i] > 180) dailyVentAngles[i] = 0;
  }
  Serial.println("Data historis harian selesai dimuat.");
}

// Mengosongkan semua data historis di EEPROM dan RAM
void clearDailyHistory() {
  Serial.println("Mengosongkan data historis harian...");
  for (int i = 0; i < 21; i++) {
    dailyTemperatures[i] = 0.0;
    dailyHumidities[i] = 0.0;
    dailyVentAngles[i] = 0;
    // Opsional: Langsung tulis 0 ke EEPROM untuk clear total
    EEPROM.put(EEPROM_ADDR_TEMP_HISTORY + (i * sizeof(float)), 0.0f);
    EEPROM.put(EEPROM_ADDR_HUM_HISTORY + (i * sizeof(float)), 0.0f);
    EEPROM.put(EEPROM_ADDR_VENT_ANGLE_HISTORY + (i * sizeof(int)), 0);
  }
  EEPROM.commit(); // Pastikan perubahan disimpan
  Serial.println("Data historis harian berhasil dikosongkan.");
}

void setup() {
  delay(2000);
  Serial.begin(115200);
  // Ukuran EEPROM.begin() harus mencakup alamat tertinggi yang digunakan + ukuran variabelnya
  //EEPROM.begin(EEPROM_ADDR_LAST_DAY + 1); // Cukup 1 byte untuk hari (1-21)
  EEPROM.begin(EEPROM_SIZE);
  loadDailyHistory();

  if (!SPIFFS.begin(true)) {
    Serial.println("Gagal mount SPIFFS!");
    while (true) delay(1000);
  }
  Serial.println("SPIFFS mounted successfully.");

  Wire.begin(I2C_SDA_PIN_0, I2C_SCL_PIN_0);
  Serial.printf("I2C Bus 0 (RTC & LCD) diinisialisasi: SDA=%d, SCL=%d\n", I2C_SDA_PIN_0, I2C_SCL_PIN_0);

  lcd.init();
  lcd.backlight();
  lcd.print("Inkubator Nyala");
  lcd.setCursor(0, 1);
  lcd.print("Loading...");
  delay(2000);

  Wire1.begin(I2C_SDA_PIN_1, I2C_SCL_PIN_1);
  Serial.printf("I2C Bus 1 (AS5600) diinisialisasi: SDA=%d, SCL=%d\n", I2C_SDA_PIN_1, I2C_SCL_PIN_1);

  angleReadingsInitialized = true;
  angleReadingIndex = 0;

  dht.begin();
  Serial.println("DHT22 sensor diinisialisasi.");

  ledcSetup(HEATER_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, HEATER_PWM_CHANNEL);
  ledcWrite(HEATER_PWM_CHANNEL, 0);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(FAN_PIN_2, OUTPUT);
  pinMode(MISTMAKER_PIN, OUTPUT);
  pinMode(TRIAC_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  digitalWrite(FAN_PIN, LOW);
  digitalWrite(FAN_PIN_2, LOW);
  digitalWrite(MISTMAKER_PIN, LOW);
  digitalWrite(TRIAC_PIN, LOW);

  myServo.attach(SERVO_PIN);
  myServo.write(currentServoPos);

  // --- PEMBACAAN DAN LOGIKA INISIALISASI RTC & EEPROM ---
  if (!rtc.begin(&Wire)) {
    Serial.println("RTC tidak terdeteksi atau gagal diinisialisasi!");
    Serial.println("Periksa koneksi RTC (SDA=21, SCL=22) atau batere.");
    // Jika RTC tidak ada, kita *tidak akan* mereset hari inkubasi ke 1 secara paksa,
    // melainkan mengandalkan `lastSavedDay` jika inkubasi sudah dimulai.
    // `tanggalMulai` akan diset ke waktu kompilasi sebagai fallback minimal.
    tanggalMulai = DateTime(F(__DATE__), F(__TIME__));
    simpanTanggalMulai(tanggalMulai); // Simpan tanggal kompilasi ini ke EEPROM
    bacaStatusInkubasi(); // Baca status inkubasi dari EEPROM
    bacaHariTerakhir();   // Baca hari terakhir dari EEPROM
    // currentDay akan diatur berdasarkan lastSavedDay di bagian bawah setup()
    Serial.println("RTC tidak aktif, mengandalkan data EEPROM dan waktu kompilasi sebagai fallback.");
  } else {
    Serial.println("RTC berhasil inisialisasi.");
    bacaStatusInkubasi(); // Baca status inkubasi terlebih dahulu
    bacaHariTerakhir();   // Baca hari terakhir terlebih dahulu

    if (rtc.lostPower()) {
      Serial.println("RTC kehilangan daya, waktu perlu disetel ulang!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Setel RTC ke waktu kompilasi
      Serial.println("RTC telah disetel ulang ke waktu kompilasi.");
      // PENTING: Kita TIDAK mereset isIncubationStarted atau currentDay di sini
      // agar bisa melanjutkan dari data EEPROM jika inkubasi sudah berjalan.
      // tanggalMulai akan disesuaikan di bawah jika 'now < tanggalMulai'
    }

    // Baca tanggalMulai dari EEPROM. Ini adalah tanggal yang kita simpan saat siklus dimulai.
    tanggalMulai = bacaTanggalMulai();
  }

  DateTime now = rtc.now();
  if (now < tanggalMulai) {
    Serial.println("Peringatan: Waktu RTC saat ini lebih awal dari tanggal mulai yang dibaca dari EEPROM.");
    Serial.println("Ini mungkin indikasi RTC disetel mundur atau masalah sinkronisasi.");
    // PENTING: Kita TIDAK mereset isIncubationStarted atau currentDay di sini.
    // Hanya sesuaikan tanggalMulai agar tidak ada error perhitungan durasi negatif.
    tanggalMulai = now;
    simpanTanggalMulai(tanggalMulai); // Simpan tanggalMulai yang baru (now)
    // currentDay akan diatur berdasarkan lastSavedDay di bagian bawah setup()
  }

  // Penentuan mode awal saat boot
  if (isIncubationStarted) {
    // Jika inkubasi sudah dimulai (dari EEPROM), paksa ke mode otomatis
    isAutoMode = true;
    // Kita telah membaca lastSavedDay sebelumnya. Gunakan nilai itu!
    currentDay = lastSavedDay;
    Serial.printf("Sistem dinyalakan, inkubasi sudah berjalan. Mode OTOMATIS aktif, melanjutkan dari Hari ke-%d.\n", currentDay);
  } else {
    // Jika inkubasi belum dimulai, default ke mode manual
    isAutoMode = false;
    currentDay = 1; // Hari ke-1 jika siklus belum dimulai
    simpanHariTerakhir(currentDay); // Pastikan hari ke-1 disimpan juga
    Serial.println("Sistem dinyalakan, inkubasi belum dimulai. Mode MANUAL aktif secara default.");
  }
  // --- AKHIR LOGIKA INISIALISASI RTC & EEPROM ---

  connectToWiFi(); // Coba koneksi WiFi

  // --- WEB SERVER HANDLERS ---
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("login1.html");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->redirect("/login1.html");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest * request) {
    bacaSensor();
    // hitungHariInkubasi(); // Tidak perlu di sini, currentDay sudah diatur oleh logika di loop/setup

    String json = "{";
    json += "\"temp\":" + String(suhu, 2) + ",";
    json += "\"hum\":" + String(kelembapan, 2) + ",";
    json += "\"day\":" + String(currentDay) + ",";
    json += "\"sudutVentilasi\":" + String(currentServoPos) + ",";

    String rackAngleString = "";

    if (posisiRakMiring()) {
      rackAngleString = "Miring";
    } else if (posisiRakTegak()) {
      rackAngleString = "Tegak";
    } else {
      rackAngleString = "Belum Inisiasi";
    }
    json += "\"sudutRak\":\"" + rackAngleString + "\",";

    int pwmDisplayValue = (currentPwmDuty*100) / MAX_PWM_DUTY;
    
    json += "\"pwmHeater\":" + String(pwmDisplayValue) + ",";
    json += "\"wifiStatus\":\"" + String(isWiFiConnected ? "CONNECTED" : "DISCONNECTED") + "\",";
    json += "\"isAutoMode\":" + String(isAutoMode ? "true" : "false") + ","; // Tambahkan status mode
    json += "\"isIncubationStarted\":" + String(isIncubationStarted ? "true" : "false"); // Tambahkan status siklus

    json += ",\"isHeaterOn\":" + String(isHeaterOn ? "true" : "false");
    json += ",\"isMistMakerOn\":" + String(isMistMakerOn ? "true" : "false");
    json += ",\"isRackMotorMoving\":" + String(motorNyala ? "true" : "false");

    json += ","; // Tambahkan koma sebelum menambahkan item baru
    json += "\"currentRtcTime\":" + String(rtc.now().unixtime() * 1000);
    json += ",\"startDate\":\""; // Tambahkan koma sebelum startDate
    if (isIncubationStarted) {
      char dateBuffer[11]; // YYYY-MM-DD\0
      dateBuffer[10] = 0;
      sprintf(dateBuffer, "%04d-%02d-%02d", tanggalMulai.year(), tanggalMulai.month(), tanggalMulai.day());
      json += String(dateBuffer);
    } else {
      json += ""; // Kosong jika inkubasi belum dimulai
    }
    json += "\"";
    json += "}";

    request->send(200, "application/json", json);
  });


  server.on("/setday", HTTP_POST,
  [](AsyncWebServerRequest * request) {
    request->send(200, "application/json", "{\"message\":\"OK\"}");
  },
  NULL,
  [](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    static String body;
    if (index == 0) body = "";
    for (size_t i = 0; i < len; i++) {
      body += (char)data[i];
    }
    if (index + len == total) {
      DynamicJsonDocument doc(256);
      auto error = deserializeJson(doc, body);
      if (!error) {
        int day = doc["day"];
        if (day >= 1 && day <= 21) {
          //set pwm ke max saat pertama kali aktif
          targetPwmDuty = MAX_PWM_DUTY;
          currentPwmDuty = MAX_PWM_DUTY;
          overrideDay = day; // overrideDay akan digunakan oleh setTanggalMulaiManualHariKe
          // Panggil fungsi setTanggalMulaiManualHariKe untuk mengatur hari dan menyimpannya
          setTanggalMulaiManualHariKe(day); // Ini akan meng-update currentDay dan menyimpannya ke EEPROM
          Serial.printf("Hari inkubasi di-set manual ke %d dari web, mode AUTO aktif\n", day);
        } else {
          Serial.println("Nilai hari inkubasi tidak valid dari web.");
        }
      } else {
        Serial.println("JSON invalid diterima di /setday dari web");
        Serial.println(error.c_str());
      }
    }
  });


  server.on("/stop", HTTP_POST, [](AsyncWebServerRequest * request) {
    isAutoMode = false;
    overrideDay = 0;
    currentDay = 1; // Reset hari
    tanggalMulai = rtc.now(); // Reset tanggal mulai ke sekarang
    simpanTanggalMulai(tanggalMulai);
    simpanStatusInkubasi(false);
    isAutoMode = false;
    Serial.println("Sistem diatur ke Mode MANUAL."); // Debug
    simpanHariTerakhir(1);
    clearDailyHistory(); // <--- PENTING: Kosongkan histori saat stop!
    matikanSemua();
    Serial.println("🛑 Inkubasi dihentikan manual dari dashboard");
    request->send(200, "text/plain", "Inkubasi dihentikan");
  });

  server.on("/toggleRack", HTTP_POST, [](AsyncWebServerRequest * request) {
    // Izinkan putaran rak manual HANYA jika inkubasi TIDAK berjalan
    if (!isIncubationStarted) { // <--- KONDISI BARU: Cek isIncubationStarted
      if (posisiRakMiring()) {
        RotateZero();
        request->send(200, "application/json", "{\"message\":\"Memulai putaran rak ke tegak.\"}");
      } else if (posisiRakTegak()) {
        RotateAgain();
        request->send(200, "application/json", "{\"message\":\"Memulai putaran rak ke miring.\"}");
      } else {
        RotateZero();
        request->send(200, "application/json", "{\"message\":\"Memulai putaran rak ke tegak.\"}");
      }
    } else { // Jika inkubasi sedang berjalan
      Serial.println("🚫 Gerakan rak manual ditolak: Inkubasi sedang berjalan.");
      request->send(403, "application/json", "{\"message\":\"Rak tidak dapat diputar secara manual saat inkubasi berjalan.\"}");
    }
  });

  server.on("/setmode/auto", HTTP_POST, [](AsyncWebServerRequest * request) {
    isAutoMode = true;
    Serial.println("Mode: OTOMATIS aktif (via web).");
    request->send(200, "text/plain", "Mode OTOMATIS aktif.");
  });

  server.on("/setmode/manual", HTTP_POST, [](AsyncWebServerRequest * request) {
    if (!isIncubationStarted) { // Hanya izinkan manual jika inkubasi belum dimulai
      isAutoMode = false;

      Serial.println("Mode: MANUAL aktif (via web).");
      request->send(200, "text/plain", "Mode MANUAL aktif.");
    } else {
      isAutoMode = true; // Paksa kembali ke otomatis jika sudah dimulai
      Serial.println("Inkubasi sudah dimulai. Mode MANUAL tidak diizinkan (via web).");
      request->send(403, "text/plain", "Mode MANUAL tidak diizinkan setelah inkubasi dimulai.");
    }
  });

  server.onNotFound([](AsyncWebServerRequest * request) {
    request->send(404, "text/plain", "File Not Found");
  });

  server.begin();
  webServerStarted = true;
  Serial.println("Web server dimulai.");

  //buat rak telur
  setupRakTelur();
  //delay(2000);

}

void loop() {
  unsigned long nowMillis = millis();

  //limit stack call
  if (nowMillis-skipTick>=50){
    skipTick=nowMillis;
  }else{
    return;
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("start ")) {
      int d, m, y;
      if (sscanf(input.c_str(), "start %d-%d-%d", &d, &m, &y) == 3) {
        tanggalMulai = DateTime(y, m, d);
        simpanTanggalMulai(tanggalMulai);

        currentDay = 1;
        simpanHariTerakhir(currentDay);

        isAutoMode = true;
        overrideDay = 0;
        simpanStatusInkubasi(true);
        clearDailyHistory(); // <--- TAMBAHKAN BARIS INI
        Serial.printf("Inkubasi dimulai dengan tanggal: %02d-%02d-%04d, hari ke-%d.\n", d, m, y, currentDay);
      } else {
        Serial.println("Format salah! Gunakan: start DD-MM-YYYY");
      }
    } else if (input.startsWith("setday ")) {
      int h = input.substring(7).toInt();
      setTanggalMulaiManualHariKe(h); // Fungsi ini sudah memanggil simpanStatusInkubasi(true) dan simpanHariTerakhir(currentDay)
    } else if (input.equalsIgnoreCase("auto")) {
      isAutoMode = true;
      Serial.println("Mode: OTOMATIS aktif (termasuk motor).");
    } else if (input.equalsIgnoreCase("manual")) {
      if (!isIncubationStarted) { // Hanya izinkan manual jika inkubasi belum dimulai
        isAutoMode = false;
        Serial.println("Mode: MANUAL aktif (termasuk motor).");

      } else {
        Serial.println("Inkubasi sudah dimulai. Mode MANUAL tidak diizinkan.");
        isAutoMode = true; // Paksa kembali ke otomatis jika sudah dimulai
      }
    } else if (input.equalsIgnoreCase("calibrate")) {
      Serial.println("Perintah kalibrasi AS5600 manual tidak didukung lagi.");
      Serial.println("Titik 0 derajat AS5600 otomatis disetel saat boot.");
    }
  }

  ////Serial.println("Debug point: 1");

  // --- LOGIKA UNTUK MAJU HARI SECARA OTOMATIS SAAT INKUBASI BERJALAN ---
  // Ini akan memastikan currentDay maju sesuai waktu kalender jika ESP tidak mati.
  // Jika ESP mati dan menyala kembali, currentDay akan melanjutkan dari lastSavedDay yang dimuat di setup.
  if (isIncubationStarted && overrideDay == 0) { // Hanya saat inkubasi otomatis dan tidak di-override
    DateTime now = rtc.now();
    TimeSpan durasi = now - tanggalMulai;
    int calculatedDay = durasi.days() + 1; // Hitung hari berdasarkan RTC dan tanggalMulai

    // Jika hari yang dihitung berbeda dari currentDay yang sedang berjalan, update dan simpan
    // Ini memastikan currentDay tidak melompat maju jika ada pemadaman singkat
    if (calculatedDay != currentDay && calculatedDay >= 1 && calculatedDay <= 21) {
      currentDay = calculatedDay;
      simpanHariTerakhir(currentDay); // <--- SIMPAN PERUBAHAN HARI OTOMATIS
      Serial.printf("Hari inkubasi otomatis berubah ke Hari ke-%d. (%s)\n", currentDay, now.timestamp().c_str());
    }
  }
  // Jika tidak di isIncubationStarted, atau ada overrideDay, currentDay akan diatur secara eksplisit.
  // Maka currentDay tidak akan maju otomatis disini.

  //Serial.println("Debug point: 2");

  if (nowMillis - lastDHTRead >= DHT_INTERVAL) {
    bacaSensor();
    lastDHTRead = nowMillis;
  }

  //Serial.println("Debug point: 3");

  if (isAutoMode) {
    //heaterState = suhu < 37.3;

    //if (heaterState) {
    //  targetPwmDuty = MAX_PWM_DUTY;
    //} else {
    //  if (suhu >= 37.7) {
    //    targetPwmDuty = 0;
    //  } else {
    //
    //  }
    //}
    
    updateHeaterPWM();
    
    isHeaterOn = (targetPwmDuty > 0);

    digitalWrite(FAN_PIN, isHeaterOn ? HIGH : LOW); // Fan 1 nyala saat heater nyala

    bool fan2State = false;
    if (suhu > 38.0 || kelembapan > 70 ) {
      fan2State = true;
    }
    else if (suhu < 37.6 && kelembapan < 68) {
      fan2State = false; // Set kipas mati
    }
    digitalWrite(FAN_PIN_2, fan2State ? HIGH : LOW);

    float targetRH = (currentDay <= 18) ? 55 : 68; // Target RH 55% untuk hari 1-18, 68% untuk hari 19-21
    float turnOnRH;
    if (targetRH == 55) {
      turnOnRH = 53; // Nyala di 53% jika target 55%
    } else { // targetRH == 68
      turnOnRH = 65; // Nyala di 65% jika target 68%
    }

    if (kelembapan < turnOnRH) {
      mistState = true;
    }
    else if (kelembapan >= targetRH) {
      mistState = false;
    }
    digitalWrite(MISTMAKER_PIN, mistState ? HIGH : LOW);
    isMistMakerOn = mistState;

    switch (currentDay) { // currentDay sudah diatur di setup atau diperbarui di loop
      case 1: case 2: case 3:
        servoAngleByDay = 40;
        break;
      case 4:
        servoAngleByDay = 60;
        break;
      case 5:
        servoAngleByDay = 85;
        break;
      case 6:
        servoAngleByDay = 110;
        break;
      case 7: case 8: case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17:
        servoAngleByDay = 135;
        break;
      case 18: case 19: case 20: case 21:
        servoAngleByDay = 40; // Kembali ke 40 derajat saat periode penetasan
        break;
      default:
        servoAngleByDay = 40; // Default untuk hari di luar range
        break;
    }

    //Serial.println("Debug point: 4");

    if (currentServoPos != servoAngleByDay) { // Cek apakah posisi target berbeda dari posisi saat ini
      if (bolehGerakKe(servoAngleByDay)) { // Cek apakah servo diizinkan untuk bergerak
        if (!myServo.attached()) { // Pastikan servo terhubung (attached)
          myServo.attach(SERVO_PIN);
          Serial.println("Servo di-ATTACH"); // Debugging: Konfirmasi attach
        }
        myServo.write(servoAngleByDay);
        //Serial.printf("✅ Servo ventilasi digerakkan ke %d°\n", servoAngleByDay); // Konfirmasi perintah gerakan
        Serial.print("✅ Servo ventilasi digerakkan ke "); // Konfirmasi perintah gerakan
        Serial.println(servoAngleByDay);
        delay(200);
        myServo.detach();
        Serial.println("Servo di-DETACH"); // Debugging: Konfirmasi detach
        currentServoPos = servoAngleByDay; // Perbarui variabel posisi servo saat ini
        detachServoFlag = true;
        detachServoTick = millis();
      } else {
        Serial.println("🚫 Gerakan servo ditolak karena limit switch atau sudut tidak valid.");
      }
    }else{
      if (detachServoFlag){
        if (millis()-detachServoTick>=500){
          detachServoFlag = false;
          myServo.detach();
          Serial.println("Servo di-DETACH"); // Debugging: Konfirmasi detach
        }
      }
    }
  } else { // Mode MANUAL
    targetPwmDuty = 0;
    updateHeaterPWM();
    isHeaterOn = false;
    digitalWrite(FAN_PIN, LOW);
    digitalWrite(FAN_PIN_2, LOW);
    digitalWrite(MISTMAKER_PIN, LOW);
    isMistMakerOn = false;
    myServo.write(40); // Setel servo ke posisi default di mode manual
    currentServoPos = 40;
  }

  //Serial.println("Debug point: 5");

  if (nowMillis - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
    //Serial.println("Debug point: 5.1");
    tampilkanStatus();
    //Serial.println("Debug point: 5.2");
    lastStatusPrint = nowMillis;
  }

  //Serial.println("Debug point: 5.10");

  if (nowMillis - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
    //if (Wire.endTransmission()){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.printf("S: %.1fC K:%.0f%%", suhu, kelembapan);
      lcd.setCursor(0, 1);
  
      if (isWiFiConnected) {
        lcd.printf("Hari: %d | WiFi OK", currentDay);
      } else {
        lcd.printf("Hari: %d | No WiFi", currentDay);
      }
  
      if (nowMillis - lastWiFiReconnectAttempt >= WIFI_RECONNECT_INTERVAL) {
        connectToWiFi();
        lastWiFiReconnectAttempt = nowMillis;
      }
    //}
      Wire.endTransmission();
    
    lastLcdUpdate = nowMillis;
  }

  //Serial.println("Debug point: 6");

  if (nowMillis - lastTickUpdate >= 1000){
    if (isIncubationStarted && overrideDay == 0) {
      DateTime now = rtc.now();
      TimeSpan durasi = now - tanggalMulai;
      int calculatedDay = durasi.days() + 1;
  
      if (calculatedDay != currentDay && calculatedDay >= 1 && calculatedDay <= 21) {
        saveDailyHistory(calculatedDay - 1, suhu, kelembapan,  currentServoPos);
  
        currentDay = calculatedDay;
        simpanHariTerakhir(currentDay); // <--- SIMPAN PERUBAHAN HARI OTOMATIS
        //Serial.printf("Hari inkubasi otomatis berubah ke Hari ke-%d. (%s)\n", currentDay, now.timestamp().c_str());
        String s = "Hari inkubasi otomatis berubah ke Hari ke-" + String(currentDay);
        Serial.println(s);
      }
    }
    lastTickUpdate = nowMillis;
  }

  //Serial.println("Debug point: 7");

  RotateRoutine();

  //Serial.println("Debug point: 8");
}

void matikanSemua() {
  targetPwmDuty = MAX_PWM_DUTY; //set to max again after off
  currentPwmDuty = 0;
  ledcWrite(HEATER_PWM_CHANNEL, 0);
  Serial.println("Heater dimatikan.");

  digitalWrite(FAN_PIN, LOW);
  digitalWrite(FAN_PIN_2, LOW);
  Serial.println("Fan dimatikan.");
  digitalWrite(MISTMAKER_PIN, LOW);
  Serial.println("Mistmaker dimatikan.");
  myServo.write(40); // Setel servo ke posisi default
  currentServoPos = 40;
  Serial.println("Servo ventilasi disetel ke 40°.");

  RotateZero();
  Serial.println("Rak telur tegak");

  Serial.println("Semua aktuator dimatikan.");
}

void tampilkanStatus() {
  //Serial.println("Debug point -> tampilkanStatus: 1");

  String rackAngleString = "";
  rackAngleString = "Belum inisiasi";
  if (posisiRakMiring()) rackAngleString = "Miring";
  if (posisiRakTegak()) rackAngleString = "Tegak";
  int pwmDisplayValue = (currentPwmDuty*100) / MAX_PWM_DUTY;

  //  Serial.printf("Suhu: %.2f°C, RH: %.2f%%, Hari inkubasi: %d, Sudut ventilasi: %d°, Heater PWM: %d, Fan: %s, Mistmaker: %s, Motor: %s | Mode Sistem: %s | Inkubasi: %s | WiFi: %s\n",
  //                suhu, kelembapan, currentDay, currentServoPos, pwmDisplayValue,
  //                heaterState ? "ON" : "OFF",
  //                mistState ? "ON" : "OFF",
  //                isAutoMode ? "OTOMATIS" : "MANUAL",
  //                isIncubationStarted ? "DIMULAI" : "BELUM DIMULAI", // Tambahkan status inkubasi
  //                isWiFiConnected ? "TERHUBUNG" : "TERPUTUS");
  Serial.print("Suhu: " + String(suhu, 2) + "°C, ");
  Serial.print("RH: " + String(kelembapan, 2) + "%, ");
  Serial.print("Hari inkubasi: " + String(currentDay) + ", ");
  Serial.print("Sudut ventilasi: " + String(currentServoPos) + ", ");
  Serial.print("Heater PWM: " + String(pwmDisplayValue) + ", ");
  Serial.print("posisi rak: " + rackAngleString + ", ");
  Serial.print("Motor Rak: " + String(motorNyala ? "ON" : "OFF") + ", ");

  Serial.println();
  //Serial.println("Debug point -> tampilkanStatus: 2");
}

bool bolehGerakKe(int targetAngle) {
  if (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
    Serial.println("Limit switch tertekan, gerakan servo dicegah.");
    return false;
  }
  // Batasi rentang gerak servo
  if (targetAngle < 40 || targetAngle > 135) {
    Serial.printf("Sudut target %d° di luar rentang yang diizinkan (40-135°).\n", targetAngle);
    return false;
  }
  return true;
}


//urusan rak telur
void setupRakTelur() {
  Serial.println("Melakukan Setup Rak Telur...");
  pinMode(HALL_DO_PIN, INPUT);
  pinMode(TRIAC_PIN, OUTPUT);
  //Wire.begin(32, 33);
  encoder.begin();
  lastStartTime = millis();

  if (isIncubationStarted) {
    if (currentDay >= 19) {
      RotateZero();
    } else {
      RotateAgain();
    }
  } else {
    RotateZero();
    Serial.println("Menegakkan rak telur.");
  }
  tRotate = millis();
  Serial.println("Setup Rak Telur OK.");
}

void SetMotor(bool state) {
  if (motorNyala != state) {
    motorNyala = state;
    digitalWrite(TRIAC_PIN, state ? HIGH : LOW);
    Serial.println("Motor: " + (String) (state ? "ON" : "OFF"));

  }
}

bool RotateZero() {
  //  int raw = encoder.rawAngle();
  //  uint8_t i2cError = Wire1.lastError();
  //  if (i2cError != 0) {
  //    SetMotor(false);
  //    rotateState = ROT_NONE;
  //    rotateState = rotateStateTarget;
  //    Serial.println("Gagal membaca sensor sudut!");
  //    return false;
  //  }

  if (rotateStateTarget != ROT_ZERO) {
    rotateStateTarget = ROT_ZERO;
    SetMotor(true);
  }

  //tunggu sampai kena hall sensor
  if (rotateStateTarget != rotateState) {
    bool hall =  !((bool)digitalRead(HALL_DO_PIN));
    if (hall) {
      rotateState = rotateStateTarget;
      SetMotor(false);
      sudutZero = encoder.rawAngle();
      sudutZeroDerajat = (sudutZero * 360) / 4096;
      sudutSekarang = 0;
      sudutSekarangDerajat = 0;
      encoder.setOffset(0);
      encoder.resetPosition(0);
      Serial.print("Sudut tengah: raw ");
      Serial.print(sudutZero);
      Serial.print(", derajat ");
      Serial.println(sudutZeroDerajat);
      //tRotate = millis();
      return true;
    }
  }

  return false;
}

bool RotateAgain() {
  if (rotateStateTarget == ROT_SKEW) {
    if (rotateState == ROT_SKEW) {
      rotateState = ROT_NONE;
      rotateStateTarget = ROT_NONE;
      if (putarRakByTimer) {
        RotateByTimer();
      } else {
        Rotate();
      }
      Serial.println("Memutar rak...");
      return true;
    }
  } else if ((rotateState == ROT_NONE && rotateStateTarget == ROT_NONE) || (rotateState == ROT_ZERO && rotateStateTarget == ROT_ZERO) ) {
    if (putarRakByTimer) {
      RotateByTimer();
    } else {
      Rotate();
    }
    Serial.println("Memutar rak...");
    return true;
  }
  Serial.println("Gagal memutar rak!");
  Serial.println("rotateState: " + String(rotateState));
  Serial.println("rotateStateTarget: " + String(rotateStateTarget));
  return false;
}

bool RotateByTimer() {
  if (rotateStateTarget != ROT_SKEW) {
    rotateStateTarget = ROT_SKEW;
    bool hall =  !((bool)digitalRead(HALL_DO_PIN));
    if (hall) {
      sudutZero = encoder.rawAngle();
      sudutZeroDerajat = (sudutZero * 360) / 4096;
      rotateFindZero = false;
      encoder.setOffset(0);
      encoder.resetPosition(0);
      Serial.print("Sudut tengah: raw ");
      Serial.print(sudutZero);
      Serial.print(", derajat ");
      Serial.println(sudutZeroDerajat);
    } else {
      rotateFindZero = true;
      Serial.println("Cari sudut tengah dulu...");
    }
    tStopPutar = millis();
    SetMotor(true);
  }

  bool hall = false;
  if (rotateStateTarget != rotateState) {
    if (rotateFindZero) {
      hall =  !((bool)digitalRead(HALL_DO_PIN));
      if (hall) {
        rotateFindZero = false;
        sudutZero = encoder.rawAngle();
        sudutZeroDerajat = (sudutZero * 360) / 4096;
        encoder.setOffset(0);
        encoder.resetPosition(0);
        Serial.print("Update Sudut tengah: raw ");
        Serial.print(sudutZero);
        Serial.print(", derajat ");
        Serial.println(sudutZeroDerajat);
        tStopPutar = millis();
      }
    } else {
      hall =  !((bool)digitalRead(HALL_DO_PIN));
      if (hall) {
        rotateFindZero = false;
        sudutZero = encoder.rawAngle();
        sudutZeroDerajat = (sudutZero * 360) / 4096;
        encoder.setOffset(0);
        encoder.resetPosition(0);
        tStopPutar = millis();
      }
      sudutSekarang = encoder.rawAngle();
      sudutSekarangDerajat = abs(((sudutSekarang * 360) / 4096) - sudutZeroDerajat);
      float angular = encoder.getAngularSpeed();
      Serial.print("Sudut: ");
      Serial.print(sudutSekarangDerajat);
      Serial.print(", angular speed ");
      Serial.println(angular);
      if ((sudutSekarangDerajat >= 45) || (millis() - tStopPutar >= 7000)) {
        rotateState = rotateStateTarget;
        SetMotor(false);
        tRotate = millis();
        Serial.println("Berhenti.");
        return true;
      }
    }
  }

  return false;
}

bool Rotate() {
  //  int raw = encoder.rawAngle();
  //  uint8_t i2cError = Wire1.lastError();
  //  if (i2cError != 0) {
  //    SetMotor(false);
  //    rotateState = ROT_NONE;
  //    rotateState = rotateStateTarget;
  //    Serial.println("Gagal membaca sensor sudut!");
  //    return false;
  //  }

  if (rotateStateTarget != ROT_SKEW) {
    rotateStateTarget = ROT_SKEW;
    bool hall =  !((bool)digitalRead(HALL_DO_PIN));
    if (hall) {
      sudutZero = encoder.rawAngle();
      sudutZeroDerajat = (sudutZero * 360) / 4096;
      rotateFindZero = false;
      encoder.setOffset(0);
      encoder.resetPosition(0);
      Serial.print("Sudut tengah: raw ");
      Serial.print(sudutZero);
      Serial.print(", derajat ");
      Serial.println(sudutZeroDerajat);
    } else {
      rotateFindZero = true;
      Serial.println("Cari sudut tengah dulu...");
    }
    SetMotor(true);
  }

  bool hall = false;
  if (rotateStateTarget != rotateState) {
    if (rotateFindZero) {
      hall =  !((bool)digitalRead(HALL_DO_PIN));
      if (hall) {
        rotateFindZero = false;
        sudutZero = encoder.rawAngle();
        sudutZeroDerajat = (sudutZero * 360) / 4096;
        encoder.setOffset(0);
        encoder.resetPosition(0);
        Serial.print("Update Sudut tengah: raw ");
        Serial.print(sudutZero);
        Serial.print(", derajat ");
        Serial.println(sudutZeroDerajat);
      }
    } else {
      hall =  !((bool)digitalRead(HALL_DO_PIN));
      if (hall) {
        rotateFindZero = false;
        sudutZero = encoder.rawAngle();
        sudutZeroDerajat = (sudutZero * 360) / 4096;
        encoder.setOffset(0);
        encoder.resetPosition(0);
      }
      sudutSekarang = encoder.rawAngle();
      sudutSekarangDerajat = abs(((sudutSekarang * 360) / 4096) - sudutZeroDerajat);
      float angular = encoder.getAngularSpeed();
      Serial.print("Sudut: ");
      Serial.print(sudutSekarangDerajat);
      Serial.print(", angular speed ");
      Serial.println(angular);
      if (sudutSekarangDerajat >= 45) {
        rotateState = rotateStateTarget;
        SetMotor(false);
        tRotate = millis();
        Serial.println("Berhenti.");
        return true;
      }
    }
  }

  return false;
}

void RotateRoutine() {
  if (isAutoMode && isIncubationStarted) {
    //if (true) {
    if (!motorNyala) {
      if (currentDay <= 19) {
        if (((millis() - tRotate) >= jedaPutarRak) || posisiRakTegak()) {
          //Serial.println("debug r3");
          tRotate = millis();
          RotateAgain();
          Serial.println("Memutar rak telur...");
        }
      } else {
        RotateZero();
      }
    }
  }

  if (rotateStateTarget != rotateState) {
    if (rotateStateTarget == ROT_ZERO) {
      RotateZero();
    } else if (rotateStateTarget == ROT_SKEW) {
      if (putarRakByTimer) {
        RotateByTimer();
      } else {
        Rotate();
      }
    }
  }
}

bool posisiRakMiring() {
  return (rotateState == ROT_SKEW);
}

bool posisiRakTegak() {
  return (rotateState == ROT_ZERO);
}
