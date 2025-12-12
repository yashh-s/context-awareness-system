/* 
 * INTEGRATED SYSTEM: Gesture Detection + Temperature Monitoring + Dual RFID Registration
 * - VL53L0X Lidar for hand gestures
 * - MLX90614 IR temperature sensor
 * - DHT11 humidity/temperature sensor
 * - Two MFRC522 RFID readers with registration/pairing
 */

#include <Wire.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <math.h>

// Sensors
#include "Adafruit_VL53L0X.h"
#include <Adafruit_MLX90614.h>
#include <DHT.h>
#include <MFRC522.h>

// ========================================
// PIN DEFINITIONS
// ========================================
// I2C: SDA=21, SCL=22
// SPI: SCK=18, MISO=19, MOSI=23
#define RST_PIN 5
#define SS_1 2      // RFID Reader 1
#define SS_2 15     // RFID Reader 2
#define DHTPIN 4

// ========================================
// SENSOR OBJECTS
// ========================================
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
DHT dht(DHTPIN, DHT11);
MFRC522 rfid1(SS_1, RST_PIN);
MFRC522 rfid2(SS_2, RST_PIN);
MFRC522::MIFARE_Key globalKey;

// ========================================
// COMMUNICATION SETUP
// ========================================
#define USE_SERIAL_USB true
Stream *commsSerial = nullptr;
const unsigned long BAUD = 115200;

// ========================================
// RFID DATABASE
// ========================================
const char *DB_PATH = "/devices.json";
const int SAFE_BLOCKS[] = {4,5,6,8,9,10};
const int SAFE_BLOCK_COUNT = sizeof(SAFE_BLOCKS)/sizeof(SAFE_BLOCKS[0]);
const int BLOCK_BYTES = 16;
#define NTAG_START_PAGE 4
#define NTAG_END_PAGE 39

String pairedUID = "";
String currentReader = ""; // Track which reader detected the tag

// ========================================
// TIMING (Non-blocking)
// ========================================
unsigned long lastTempTime = 0;
unsigned long lastGestureCheck = 0;
const unsigned long tempInterval = 1000;      // 1 second
const unsigned long gestureInterval = 50;     // 20 Hz

// ========================================
// GESTURE STATE MACHINE (EXACT ORIGINAL LOGIC)
// ========================================
enum State {
  IDLE,           // No valid hand detected (OOR or > 40cm).
  CHECKING_HOLD,  // Hand is stable, timing for Gesture 3 (Hold)
  CHECKING_MOVE,  // Hand is actively moving (Gestures 4 & 5)
  CHECKING_TAP,   // Hand is very close (<4cm), timing for Tap (1) or Mute (6)
  CHECKING_FLICK, // Hand just appeared, timing for Gesture 2 (Flick)
  CHECKING_MUTE,  // Hand is close (4-5cm), timing for Gesture 6 (Mute)
  COOLDOWN        // Waiting for 1 second after a gesture was detected
};
State current_state = IDLE;

// --- Gesture 3: Hold (Pause/Play) ---
const unsigned long HOLD_DURATION_MS = 1500; // 1.5 seconds
const float HOLD_TOLERANCE_CM = 3.0;         // Hand can move 3cm and still be "holding"

// --- Gestures 4 & 5: Move (Volume) ---
const float MOVE_MIN_DISTANCE_CM = 3.0;      // 3cm

// --- Gesture 1: Tap ---
const float TAP_THRESHOLD_CM = 4.0;          // 40mm
const unsigned long TAP_MAX_DURATION_MS = 500; // Must be in/out of zone in < 0.5s

// --- Gesture 2: Flick ---
const unsigned long FLICK_MAX_DURATION_MS = 300; // Must appear/disappear in < 0.3s

// --- Gesture 6: Mute ---
const float MUTE_THRESHOLD_CM = 5.0;         // 5cm
const unsigned long MUTE_DURATION_MS = 1500; // 1.5 seconds

// --- Max distance for any gesture ---
const float MAX_GESTURE_DISTANCE_CM = 40.0;  // 40cm

// --- Cooldown ---
const unsigned long GESTURE_COOLDOWN_MS = 1000; // 1 second
unsigned long last_gesture_time = 0; // When the last gesture was detected

// --- State machine tracking variables ---
float previous_valid_distance_cm = -1; // -1 indicates last reading was out-of-range
float hold_start_distance = 0;   // Where a hold was initiated
float move_start_distance = 0;   // Where a move gesture began
bool move_path_invalid = false;  // Flag if move ever entered the tap zone
unsigned long gesture_start_time = 0; // A generic timer for hold, tap, and flick

// ========================================
// FORWARD DECLARATIONS
// ========================================
void processGestures();
void processTemperature();
void processRFID(MFRC522 &rfid, const String &readerName);
void processIncomingComms();
void handleNewTag(MFRC522 &rfid, const String& uid, const String &readerName);
void gestureDetected();
void printSimpleGesture(int gesture_num);
void printMoveGesture(float total_distance_moved);
bool isUIDRegistered(const String& uid);
void incrementHits(const String& uid);
void storePairing(const String& uid, const String& id, const String& name);
String getUID(MFRC522 &rfid);
void ensureDbExists();
void printDevice(const String& uid);
bool tagStillPresent(MFRC522 &rfid);
void waitForTagRemovalWithPairing(MFRC522 &rfid, const String& uid, const String& typeName);
void performWriteForUIDAndType(MFRC522 &rfid, const String &uid, const String &typeName);
bool writeClassicBlocks(MFRC522 &rfid, const byte *buf, int len, const int blocks[], int count);
bool writeUltralightPages(MFRC522 &rfid, const byte *buf, int len);
String readBlockHexNoHalt(MFRC522 &rfid, int block);
void printNonEmptyClassicBlocks(MFRC522 &rfid);
void printUltralightDump(MFRC522 &rfid);
bool reselectCard(MFRC522 &rfid);
String extractKV(const String& body, const String& key);
String readAllSafeBlocksHex(MFRC522 &rfid);

// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(BAUD);
  delay(1200);
  
  if (USE_SERIAL_USB) {
    commsSerial = &Serial;
  } else {
    Serial2.begin(BAUD, SERIAL_8N1, 16, 17);
    commsSerial = &Serial2;
  }

  // Default RFID key
  for (byte i = 0; i < 6; i++) globalKey.keyByte[i] = 0xFF;

  // I2C Init
  Wire.begin(21, 22);
  
  // VL53L0X Lidar
  if (!lidar.begin()) {
    Serial.println("VL53L0X failed!");
    while (!lidar.begin()) delay(500);
  }
  Serial.println("VL53L0X ready!");

  // MLX90614
  if (!mlx.begin()) {
    Serial.println("MLX90614 not found!");
    while (1);
  }
  Serial.println("MLX90614 ready!");

  // DHT11
  dht.begin();
  Serial.println("DHT11 ready!");

  // SPI & RFID
  SPI.begin(18, 19, 23);
  rfid1.PCD_Init();
  rfid2.PCD_Init();
  Serial.println("RFID Readers ready!");

  // SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed!");
    while (1) delay(500);
  }
  ensureDbExists();

  Serial.println("\n=== INTEGRATED SYSTEM READY ===");
  Serial.println("Gesture + Temperature + Dual RFID");
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  unsigned long now = millis();
  
  // Process incoming serial commands
  processIncomingComms();
  
  // Temperature monitoring (every 1 second)
  if (now - lastTempTime >= tempInterval) {
    lastTempTime = now;
    processTemperature();
  }
  
  // Gesture detection (every 50ms)
  if (now - lastGestureCheck >= gestureInterval) {
    lastGestureCheck = now;
    processGestures();
  }
  
  // RFID detection (continuous, non-blocking)
  processRFID(rfid1, "RFID1");
  processRFID(rfid2, "RFID2");
}

// ========================================
// TEMPERATURE PROCESSING
// ========================================
void processTemperature() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float amb = mlx.readAmbientTempC();
  float obj = mlx.readObjectTempC();

  Serial.println("[TEMP]");
  Serial.print("DHT="); Serial.println(t);
  Serial.print("HUM="); Serial.println(h);
  Serial.print("AMB="); Serial.println(amb);
  Serial.print("OBJ="); Serial.println(obj);
  Serial.println("[/TEMP]");
}

// ========================================
// GESTURE PROCESSING (EXACT ORIGINAL LOGIC)
// ========================================
void processGestures() {
  unsigned long current_time = millis();

  // --- 0. Check for Cooldown ---
  if (current_state == COOLDOWN) {
    if (current_time - last_gesture_time > GESTURE_COOLDOWN_MS) {
      current_state = IDLE; // Cooldown finished, go to IDLE
      previous_valid_distance_cm = -1; // Force fresh start
    }
    delay(50);
    return;    // Skip all logic
  }

  // --- 1. Get Reading ---
  VL53L0X_RangingMeasurementData_t measure;
  lidar.rangingTest(&measure, false); // Take a distance reading
  float current_distance_cm = 0.0;
  bool is_valid_gesture_range = false;

  // --- 2. Classify Reading (No Gesture / Valid Gesture) ---
  if (measure.RangeStatus == 4) {   // It's Out of Range
    is_valid_gesture_range = false;
  }
  else {    // It's a valid reading
    current_distance_cm = measure.RangeMilliMeter / 10.0;
    if (current_distance_cm > MAX_GESTURE_DISTANCE_CM) {     // It's too far (> 40cm)
      is_valid_gesture_range = false;
    }
    else {    // It's in the gesture zone (0-40 cm)
      is_valid_gesture_range = true;
    }
  }

  // --- 3. Handle "No Gesture" state ---
  if (!is_valid_gesture_range) {
    
    // Check if this completes GESTURE 1 (Tap)
    if (current_state == CHECKING_TAP && (current_time - gesture_start_time < TAP_MAX_DURATION_MS)) {
      printSimpleGesture(1); 
    }
    // Check if this completes GESTURE 2 (Flick)
    else if (current_state == CHECKING_FLICK && (current_time - gesture_start_time < FLICK_MAX_DURATION_MS)) {
      printSimpleGesture(2); 
    }
    // Check if this completes GESTURES 4 or 5 (Move)
    else if (current_state == CHECKING_MOVE) {
      float total_distance_moved = previous_valid_distance_cm - move_start_distance;
      printMoveGesture(total_distance_moved); 
    }

    // Reset state machine to IDLE *only if a gesture wasn't just detected*
    if (current_state != COOLDOWN) {
       current_state = IDLE;
    }
    previous_valid_distance_cm = -1; // Mark distance as invalid
    return; // Skip the rest of the loop
  }

  // --- 4. Handle "Valid Gesture" state (0-40 cm) ---
  
  // Check for GESTURE 2 (Flick) Start
  if (previous_valid_distance_cm == -1 && current_distance_cm > MUTE_THRESHOLD_CM) {
    current_state = CHECKING_FLICK;
    gesture_start_time = current_time;
  }
  
  // Check for GESTURE 1 (Tap) Start
  else if (current_distance_cm < TAP_THRESHOLD_CM) { // < 4cm
    move_path_invalid = true; // A move is invalid if it ever enters the tap zone
    
    if (current_state != CHECKING_TAP) {
      // Check if a move was interrupted. If so, end it.
      if (current_state == CHECKING_MOVE) {
        float total_distance_moved = previous_valid_distance_cm - move_start_distance;
        printMoveGesture(total_distance_moved);
      }
      current_state = CHECKING_TAP;
      gesture_start_time = current_time; // Start timer for tap OR mute
    }
  }
  
  // Check for GESTURE 1 (Tap) End (within valid range)
  else if (current_state == CHECKING_TAP && current_distance_cm >= TAP_THRESHOLD_CM) {
    if (current_time - gesture_start_time < TAP_MAX_DURATION_MS) {
      printSimpleGesture(1); // Tap complete! 
    } else { // else: it was too slow, not a tap
      current_state = IDLE; // Reset to IDLE to re-evaluate
    }
  }

  // --- 5. Run Stateful Logic (Hold, Move, Mute, Timeouts) ---
  switch (current_state) {
    
    case IDLE: {
      // Hand just appeared or a gesture just finished.
      gesture_start_time = current_time;
      hold_start_distance = current_distance_cm;
      move_start_distance = current_distance_cm;
      move_path_invalid = false;
      
      if (current_distance_cm < MUTE_THRESHOLD_CM) {
        current_state = CHECKING_MUTE;
      } else {
        current_state = CHECKING_HOLD;
      }
      break;
    }

    case CHECKING_HOLD: {
      float hold_delta = abs(current_distance_cm - hold_start_distance);
      
      // Check for Mute zone first
      if (current_distance_cm < MUTE_THRESHOLD_CM) {
        current_state = CHECKING_MUTE;
        gesture_start_time = current_time; // Reset timer for mute
        hold_start_distance = current_distance_cm;
      }
      // Check if hold is broken
      else if (hold_delta > HOLD_TOLERANCE_CM) {
        current_state = CHECKING_MOVE;
      }
      // Check if GESTURE 3: Hold is complete
      else if (current_time - gesture_start_time > HOLD_DURATION_MS) {
        printSimpleGesture(3); 
      }
      break;
    }

    case CHECKING_MUTE: {
      float hold_delta = abs(current_distance_cm - hold_start_distance);
      
      // Check if hold is broken
      if (hold_delta > HOLD_TOLERANCE_CM) {
        current_state = CHECKING_MOVE;
      }
      // Check if GESTURE 6: Mute is complete
      else if (current_time - gesture_start_time > MUTE_DURATION_MS) {
        printSimpleGesture(6); 
      }
      break;
    }

    case CHECKING_MOVE: {
      float move_delta = abs(current_distance_cm - hold_start_distance);

      if (move_delta < HOLD_TOLERANCE_CM) {
        // Hand stopped moving. End the move gesture.
        float total_distance_moved = current_distance_cm - move_start_distance;
        printMoveGesture(total_distance_moved); 
      }
      // else: still moving, reset hold_start_distance
      hold_start_distance = current_distance_cm;
      break;
    }

    case CHECKING_TAP: {
      if (current_time - gesture_start_time > MUTE_DURATION_MS) {
        // Held too long, it's a Mute!
        printSimpleGesture(6); 
      }
      break;
    }

    case CHECKING_FLICK: {
      if (current_time - gesture_start_time > FLICK_MAX_DURATION_MS) {
        // Stayed in range for too long, not a "flick".
        current_state = IDLE;
      }
      break;
    }

    case COOLDOWN: { break; }
  }

  // --- 6. Save current distance for the next loop ---
  previous_valid_distance_cm = current_distance_cm;
}

void gestureDetected() {
  current_state = COOLDOWN;
  last_gesture_time = millis();
}

void printSimpleGesture(int gesture_num) {
  String name = "";
  if (gesture_num == 1) name = "Tap";
  else if (gesture_num == 2) name = "Flick";
  else if (gesture_num == 3) name = "Hold";
  else if (gesture_num == 6) name = "Mute";
  else name = "Unknown";

  Serial.print("[Gesture] Gesture_id = ");
  Serial.print(gesture_num);
  Serial.print("; Gesture_Name = ");
  Serial.print(name);
  Serial.print("; volume_change = +0");
  Serial.println("; [/Gesture]");
  
  gestureDetected();
}

void printMoveGesture(float total_distance_moved) {
  if (move_path_invalid) return;
  if (abs(total_distance_moved) < MOVE_MIN_DISTANCE_CM) return;

  int gesture_num;
  int sign;

  if (total_distance_moved < 0) {
    gesture_num = 5;
    sign = 1;
  } else {
    gesture_num = 4;
    sign = -1;
  }

  int volume_change = (int)(floor(abs(total_distance_moved)) * 3.0) * sign;
  if (volume_change > 100) volume_change = 100;
  if (volume_change < -100) volume_change = -100;

  String name = (gesture_num == 4) ? "Volume Down" : "Volume Up";

  Serial.print("[Gesture] Gesture_id = ");
  Serial.print(gesture_num);
  Serial.print("; Gesture_Name = ");
  Serial.print(name);
  Serial.print("; volume_change = ");
  if (volume_change >= 0) Serial.print("+");
  Serial.print(volume_change);
  Serial.println("; [/Gesture]");
  
  gestureDetected();
}

// ========================================
// RFID PROCESSING
// ========================================
// ========================================
// TAG TRACKING STATE
// ========================================
struct TagSession {
  bool active;
  String uid;
  String device_id;
  String device_name;
  String reader_name;
  String type_name;
  unsigned long detected_time;
  unsigned long last_presence_check;
  int consecutive_absent;
  bool was_registered;
  bool needs_pairing_write;
};

TagSession session1 = {false, "", "", "", "", "", 0, 0, 0, false, false};
TagSession session2 = {false, "", "", "", "", "", 0, 0, 0, false, false};

void processRFID(MFRC522 &rfid, const String &readerName) {
  // Get the correct session for this reader
  TagSession* session = (readerName == "RFID1") ? &session1 : &session2;
  
  // If session is active, check for tag removal
  if (session->active) {
    unsigned long now = millis();
    // Check every 200ms
    if (now - session->last_presence_check >= 200) {
      session->last_presence_check = now;
      
      if (!tagStillPresent(rfid)) {
        session->consecutive_absent++;
        if (session->consecutive_absent >= 3) {
          // Tag removed!
          Serial.println("\n---- TAG REMOVED from " + readerName + " ----");
          Serial.println(session->uid);
          
          if (session->device_id.length() > 0) {
            String removedPkt = "[removed]UID=" + session->uid + 
                               ";device_id=" + session->device_id + 
                               ";device_name=" + session->device_name + 
                               "[/removed]";
            if (commsSerial) {
              commsSerial->println(removedPkt);
              Serial.println("Sent removed packet to Pi:");
              //Serial.println(removedPkt);
            }
          }
          
          rfid.PICC_HaltA();
          rfid.PCD_StopCrypto1();
          
          // Clear session
          session->active = false;
          session->uid = "";
          session->device_id = "";
          session->device_name = "";
          session->consecutive_absent = 0;
          
          Serial.println("Session ended. Ready for next tag.\n");
        }
      } else {
        session->consecutive_absent = 0;
        
        // Check if we need to write pairing data
        if (session->needs_pairing_write && pairedUID.length() && pairedUID == session->uid) {
          Serial.println("\n!!! PAIRING DATA RECEIVED - Writing to tag !!!");
          pairedUID = "";
          performWriteForUIDAndType(rfid, session->uid, session->type_name);
          session->needs_pairing_write = false;
        }
      }
    }
    return; // Don't detect new tags while session is active
  }
  
  // Detect new card
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  String uid = getUID(rfid);
  Serial.println("\n---- NEW TAG DETECTED on " + readerName + " ----");
  Serial.println(uid);

  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  String typeName = rfid.PICC_GetTypeName(piccType);
  
  bool wasRegistered = isUIDRegistered(uid);

  // Initialize session
  session->active = true;
  session->uid = uid;
  session->reader_name = readerName;
  session->type_name = typeName;
  session->detected_time = millis();
  session->last_presence_check = millis();
  session->consecutive_absent = 0;
  session->was_registered = wasRegistered;
  session->needs_pairing_write = false;

  // Handle the tag
  handleNewTag(rfid, uid, readerName);

  // Get device info
  if (isUIDRegistered(uid)) {
    DynamicJsonDocument doc(4096);
    File f = SPIFFS.open(DB_PATH, "r");
    if (f && !deserializeJson(doc, f)) {
      JsonObject devs = doc["devices"];
      if (devs.containsKey(uid)) {
        JsonObject ent = devs[uid];
        session->device_id = ent["device_id"] | "";
        session->device_name = ent["device_name"] | "";
      }
      f.close();
    }
  }
  
  // If it's a new tag that just got registered, flag it for pairing write
  if (!wasRegistered && isUIDRegistered(uid)) {
    session->needs_pairing_write = true;
    Serial.println("📝 New tag registered - waiting for pairing data to write...");
  }
}

void handleNewTag(MFRC522 &rfid, const String& uid, const String &readerName) {
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  String typeName = rfid.PICC_GetTypeName(piccType);
  Serial.print("Card Type: "); Serial.println(typeName);

  if (isUIDRegistered(uid)) {
    Serial.println("✓ Tag already registered!");
    
    DynamicJsonDocument doc(4096);
    File f = SPIFFS.open(DB_PATH, "r");
    if (f) {
      if (!deserializeJson(doc, f)) {
        JsonObject devs = doc["devices"];
        if (devs.containsKey(uid)) {
          JsonObject ent = devs[uid];
          String device_id = ent["device_id"] | "";
          String device_name = ent["device_name"] | "";
          
          String detectedPkt = "[detected]UID=" + uid + ";device_id=" + device_id + ";device_name=" + device_name + "[/detected]";
          if (commsSerial) {
            commsSerial->println(detectedPkt);
            Serial.println("Sent detected packet to Pi:");
            Serial.println(detectedPkt);
          }
        }
      }
      f.close();
    }
    
    incrementHits(uid);
    Serial.println("\nStored entry:");
    printDevice(uid);
    Serial.println("\n🎮 Gesture control active. Tag monitoring in background.");
    return;
  }

  Serial.println("⚠ Unregistered tag detected!");
  String blocksCSV = readAllSafeBlocksHex(rfid);
  String pkt = "[register]UID=" + uid + ";blocks=" + blocksCSV + "[/register]";
  if (commsSerial) commsSerial->println(pkt);
  Serial.println("Sent register packet to Pi:");
  //Serial.println(pkt);
  Serial.println("📝 Waiting for pairing from Pi. Gestures/temp continue in background.");
}

// ========================================
// RFID HELPER FUNCTIONS
// ========================================
String getUID(MFRC522 &rfid) {
  String s = "";
  for (byte i=0; i<rfid.uid.size; i++){
    if (rfid.uid.uidByte[i] < 0x10) s += "0";
    s += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size-1) s += ":";
  }
  s.toUpperCase();
  return s;
}

bool tagStillPresent(MFRC522 &rfid) {
  byte atqa[2]; 
  byte atqaLen = sizeof(atqa);
  MFRC522::StatusCode st = rfid.PICC_WakeupA(atqa, &atqaLen);
  if (st == MFRC522::STATUS_OK) return true;
  
  atqaLen = sizeof(atqa);
  st = rfid.PICC_RequestA(atqa, &atqaLen);
  if (st == MFRC522::STATUS_OK) {
    st = rfid.PICC_Select(&(rfid.uid));
    if (st == MFRC522::STATUS_OK) return true;
  }
  
  return false;
}

bool reselectCard(MFRC522 &rfid) {
  rfid.PCD_StopCrypto1();
  delay(20);
  
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  MFRC522::StatusCode status = rfid.PICC_WakeupA(bufferATQA, &bufferSize);
  if (status != MFRC522::STATUS_OK) {
    bufferSize = sizeof(bufferATQA);
    status = rfid.PICC_RequestA(bufferATQA, &bufferSize);
    if (status != MFRC522::STATUS_OK) return false;
  }
  
  delay(10);
  status = rfid.PICC_Select(&rfid.uid);
  if (status != MFRC522::STATUS_OK) return false;
  delay(10);
  
  return true;
}

String readAllSafeBlocksHex(MFRC522 &rfid) {
  String out = "";
  for (int i=0;i<SAFE_BLOCK_COUNT;i++) {
    if (i) out += ",";
    out += readBlockHexNoHalt(rfid, SAFE_BLOCKS[i]);
  }
  return out;
}

String readBlockHexNoHalt(MFRC522 &rfid, int block) {
  byte buff[18];
  byte size = sizeof(buff);
  MFRC522::MIFARE_Key key;
  for (byte i=0;i<6;i++) key.keyByte[i] = 0xFF;

  byte trailer = (block/4)*4 + 3;
  
  MFRC522::StatusCode st;
  for (int attempt = 0; attempt < 2; attempt++) {
    st = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &key, &(rfid.uid));
    if (st == MFRC522::STATUS_OK) break;
    
    rfid.PCD_StopCrypto1();
    delay(50);
    
    byte atqa[2];
    byte atqaLen = sizeof(atqa);
    rfid.PICC_WakeupA(atqa, &atqaLen);
    delay(10);
    rfid.PICC_Select(&(rfid.uid));
    delay(30);
  }
  
  if (st != MFRC522::STATUS_OK) {
    rfid.PCD_StopCrypto1();
    return "NOAUTH";
  }

  st = rfid.MIFARE_Read(block, buff, &size);
  rfid.PCD_StopCrypto1();
  if (st != MFRC522::STATUS_OK) return "ERR";

  String hex = "";
  for (byte i=0;i<16;i++){
    char tmp[3];
    sprintf(tmp, "%02X", buff[i]);
    hex += tmp;
  }
  return hex;
}

bool writeClassicBlocks(MFRC522 &rfid, const byte *buf, int len, const int blocks[], int count) {
  int total = count * 16;
  static byte temp[128];
  if (total > (int)sizeof(temp)) return false;
  
  memset(temp, 0, total);
  for (int i=0; i<len && i<total; i++) temp[i] = buf[i];

  int currentSector = -1;
  bool authenticated = false;

  for (int i=0; i<count; i++) {
    byte chunk[16];
    for (int j=0; j<16; j++) chunk[j] = temp[i*16 + j];

    int currentBlock = blocks[i];
    int blockSector = currentBlock / 4;
    byte trailer = (blockSector * 4) + 3;
    
    if (blockSector != currentSector) {
      if (authenticated) {
        rfid.PCD_StopCrypto1();
        delay(100);
        
        bool reselected = false;
        for (int attempt = 0; attempt < 3; attempt++) {
          rfid.PICC_HaltA();
          delay(50);
          if (reselectCard(rfid)) {
            reselected = true;
            break;
          }
        }
        if (!reselected) return false;
        delay(80);
      }
      
      MFRC522::StatusCode st = rfid.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &globalKey, &(rfid.uid));
      
      if (st != MFRC522::STATUS_OK) {
        rfid.PCD_StopCrypto1();
        return false;
      }
      
      authenticated = true;
      currentSector = blockSector;
    }

    MFRC522::StatusCode st = rfid.MIFARE_Write(currentBlock, chunk, 16);
    if (st != MFRC522::STATUS_OK) {
      rfid.PCD_StopCrypto1();
      return false;
    }
    
    delay(30);
    yield(); // Allow other processes to run
  }
  
  if (authenticated) {
    rfid.PCD_StopCrypto1();
  }
  
  return true;
}

bool writeUltralightPages(MFRC522 &rfid, const byte *buf, int len) {
  int available = (NTAG_END_PAGE - NTAG_START_PAGE + 1) * 4;
  if (len > available) len = available;

  int page = NTAG_START_PAGE;
  int idx = 0;
  while (idx < len && page <= NTAG_END_PAGE) {
    byte pageBuf[4] = {0,0,0,0};
    for (int k=0; k<4 && idx < len; k++) {
      pageBuf[k] = buf[idx++];
    }
    MFRC522::StatusCode st = rfid.MIFARE_Ultralight_Write(page, pageBuf, 4);
    if (st != MFRC522::STATUS_OK) {
      Serial.printf("Ultralight write failed page %d\n", page);
      return false;
    }
    page++;
    delay(20);
    processIncomingComms();
    yield();
  }
  return true;
}

void performWriteForUIDAndType(MFRC522 &rfid, const String &uid, const String &typeName) {
  Serial.println("\n📝 STARTING TAG WRITE OPERATION");
  
  DynamicJsonDocument doc(4096);
  File f = SPIFFS.open(DB_PATH, "r");
  if (!f) { Serial.println("ERR reading DB"); return; }
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.println("ERR parse DB"); return; }
  JsonObject devs = doc["devices"];
  if (!devs.containsKey(uid)) { Serial.println("ERR no DB entry"); return; }
  JsonObject ent = devs[uid];
  String device_id = ent["device_id"] | "";
  String device_name = ent["device_name"] | "";

  String payload = device_id + ";" + device_name;
  Serial.printf("Writing: id=%s name=%s\n", device_id.c_str(), device_name.c_str());

  if (typeName.indexOf("Classic") >= 0) {
    Serial.println("Checking if data already matches...");
    
    String currentData = "";
    for (int i = 0; i < SAFE_BLOCK_COUNT; i++) {
      String hex = readBlockHexNoHalt(rfid, SAFE_BLOCKS[i]);
      if (hex != "NOAUTH" && hex != "ERR") {
        for (int j = 0; j < hex.length(); j += 2) {
          if (j + 1 < hex.length()) {
            String byteStr = hex.substring(j, j + 2);
            char c = (char)strtol(byteStr.c_str(), NULL, 16);
            if (c != 0) currentData += c;
          }
        }
      }
    }
    
    currentData.trim();
    if (currentData == payload) {
      Serial.println("✓ Data already matches! Skipping write.");
      return;
    }
  }

  int plen = payload.length();
  static byte writeBuf[256];
  memset(writeBuf, 0, sizeof(writeBuf));
  for (int i=0; i<plen && i < (int)sizeof(writeBuf); ++i) writeBuf[i] = (byte)payload[i];

  rfid.PCD_StopCrypto1();
  delay(50);
  
  if (!reselectCard(rfid)) {
    Serial.println("⚠ Card reselect before write failed");
  }
  delay(50);

  bool ok = false;
  if (typeName.indexOf("Ultralight") >= 0 || typeName.indexOf("NTAG") >= 0) {
    Serial.println("Using Ultralight/NTAG writes");
    ok = writeUltralightPages(rfid, writeBuf, plen);
  } else {
    Serial.println("Using MIFARE Classic writes");
    ok = writeClassicBlocks(rfid, writeBuf, plen, SAFE_BLOCKS, SAFE_BLOCK_COUNT);
    if (ok) {
      rfid.PCD_StopCrypto1();
      delay(150);
      reselectCard(rfid);
      delay(100);
    }
  }

  if (ok) Serial.println("✓ WRITE SUCCESSFUL");
  else Serial.println("❌ WRITE FAILED");
}

void printNonEmptyClassicBlocks(MFRC522 &rfid) {
  Serial.println("\n=== NON-EMPTY TAG BLOCKS ===");
  for (int i = 0; i < SAFE_BLOCK_COUNT; i++) {
    int block = SAFE_BLOCKS[i];
    String hex = readBlockHexNoHalt(rfid, block);
    if (hex == "ERR" || hex == "NOAUTH") {
      Serial.printf("Block %d : %s\n", block, hex.c_str());
      continue;
    }
    bool allZero = true;
    for (int k = 0; k < hex.length(); k++) {
      if (hex[k] != '0') { allZero = false; break; }
    }
    if (!allZero) {
      Serial.printf("Block %d : %s\n", block, hex.c_str());
    }
  }
  Serial.println("============================\n");
}

void printUltralightDump(MFRC522 &rfid) {
  Serial.println("\n=== ULTRALIGHT MEMORY DUMP ===");
  for (int page = NTAG_START_PAGE; page <= NTAG_END_PAGE; page += 4) {
    byte buff[18];
    byte size = sizeof(buff);
    MFRC522::StatusCode st = rfid.MIFARE_Read(page, buff, &size);
    if (st != MFRC522::STATUS_OK) continue;
    Serial.printf("P%02d: ", page);
    for (int i=0; i<16; i++) Serial.printf("%02X ", buff[i]);
    Serial.println();
    delay(10);
  }
  Serial.println("===========================\n");
}

void waitForTagRemovalWithPairing(MFRC522 &rfid, const String& uid, const String& typeName) {
  Serial.println("Waiting for tag removal (processing pairing if received)...");
  unsigned long start = millis();
  bool writePerformed = false;
  int consecutiveAbsent = 0;
  
  while (true) {
    processIncomingComms();
    
    if (!writePerformed && pairedUID.length() && pairedUID == uid) {
      Serial.println("\n!!! NEW PAIRING DATA RECEIVED !!!");
      pairedUID = "";
      
      if (tagStillPresent(rfid)) {
        performWriteForUIDAndType(rfid, uid, typeName);
        writePerformed = true;
        consecutiveAbsent = 0;
      } else {
        Serial.println("Tag removed, cannot write.");
        return;
      }
    }
    
    if (!tagStillPresent(rfid)) {
      consecutiveAbsent++;
      if (consecutiveAbsent >= 3) {
        Serial.println(writePerformed ? "✓ Write performed, tag removed." : "Tag removed.");
        break;
      }
      delay(80);
    } else {
      consecutiveAbsent = 0;
    }
    
    if (millis() - start > 120000) { 
      Serial.println("Wait timeout"); 
      break; 
    }
    
    delay(100);
    yield();
  }
}

// ========================================
// DATABASE FUNCTIONS
// ========================================
void ensureDbExists() {
  if (!SPIFFS.exists(DB_PATH)) {
    File f = SPIFFS.open(DB_PATH, "w");
    StaticJsonDocument<256> doc;
    doc.createNestedObject("devices");
    serializeJsonPretty(doc, f);
    f.close();
    Serial.println("Created devices.json");
  }
}

bool isUIDRegistered(const String& uid) {
  DynamicJsonDocument doc(4096);
  File f = SPIFFS.open(DB_PATH, "r");
  if (!f) return false;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  if (!doc.containsKey("devices")) return false;
  JsonObject devs = doc["devices"];
  return devs.containsKey(uid);
}

void incrementHits(const String& uid) {
  DynamicJsonDocument doc(4096);
  File f = SPIFFS.open(DB_PATH, "r");
  if (!f) return;
  if (deserializeJson(doc, f)) { f.close(); return; }
  f.close();
  if (!doc.containsKey("devices")) doc.createNestedObject("devices");
  JsonObject devs = doc["devices"];
  if (!devs.containsKey(uid)) {
    JsonObject e = devs.createNestedObject(uid);
    e["hits"] = 1;
  } else {
    devs[uid]["hits"] = devs[uid]["hits"].as<int>() + 1;
    devs[uid]["last_seen"] = millis();
  }
  f = SPIFFS.open(DB_PATH, "w");
  serializeJsonPretty(doc, f);
  f.close();
}

void storePairing(const String& uid, const String& id, const String& name) {
  DynamicJsonDocument doc(4096);
  File f = SPIFFS.open(DB_PATH, "r");
  if (!f) return;
  if (deserializeJson(doc, f)) { f.close(); return; }
  f.close();
  if (!doc.containsKey("devices")) doc.createNestedObject("devices");
  JsonObject devs = doc["devices"];
  JsonObject e = devs.createNestedObject(uid);
  e["device_id"] = id;
  e["device_name"] = name;
  e["hits"] = 1;
  e["last_seen"] = millis();
  f = SPIFFS.open(DB_PATH, "w");
  serializeJsonPretty(doc, f);
  f.close();
}

void printDevice(const String& uid) {
  DynamicJsonDocument doc(4096);
  File f = SPIFFS.open(DB_PATH, "r");
  if (!f) { Serial.println("Cannot open DB"); return; }
  if (deserializeJson(doc, f)) { f.close(); Serial.println("DB parse err"); return; }
  f.close();
  if (!doc.containsKey("devices")) { Serial.println("No devices key"); return; }
  JsonObject devs = doc["devices"];
  if (!devs.containsKey(uid)) { Serial.println("Device not found"); return; }
  String out; 
  serializeJsonPretty(devs[uid], out);
  Serial.println(out);
}

// ========================================
// SERIAL COMMUNICATION
// ========================================
void processIncomingComms() {
  if (!commsSerial) return;
  while (commsSerial->available()) {
    String line = commsSerial->readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    Serial.println("RX COMMS: " + line);
    if (line.startsWith("[paired]") && line.endsWith("[/paired]")) {
      String body = line.substring(8, line.length()-9);
      String uid = extractKV(body, "UID");
      String device_id = extractKV(body, "device_id");
      String device_name = extractKV(body, "device_name");
      if (uid.length() && device_id.length()) {
        storePairing(uid, device_id, device_name);
        Serial.println("Stored pairing: " + uid + " -> " + device_id);
        pairedUID = uid;
      }
    }
  }
}

String extractKV(const String& body, const String& key) {
  int idx = body.indexOf(key + "=");
  if (idx < 0) return "";
  int start = idx + key.length() + 1;
  int end = body.indexOf(';', start);
  if (end < 0) end = body.length();
  return body.substring(start, end);
}
