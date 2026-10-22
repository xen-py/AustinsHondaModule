
#include <SoftwareSerial.h>

// ============================================================
// CONFIGURATION
// ============================================================

#define OUTPUT_HUMAN 0
#define OUTPUT_CSV   1

// Default: readable output for Serial Monitor.
// Change to OUTPUT_CSV when running the Tkinter application.
#define OUTPUT_MODE OUTPUT_HUMAN

const unsigned long USB_BAUD = 115200;
const unsigned long ECU_BAUD = 38400;

const unsigned long POLL_INTERVAL_MS = 25;
const unsigned long DISPLAY_INTERVAL_MS = 500;
const unsigned long HANDSHAKE_RETRY_MS = 1000;
const unsigned long PACKET_TIMEOUT_MS = 35;

const byte ECU_WAKE = 0x10;
const byte ECU_PACKET_REQUEST = 0x20;
const byte ECU_CONNECTED = 0xCD;

const byte PACKET_SIZE = 52;

// Assumes pins 8 = Arduino RX, 9 = Arduino TX.
SoftwareSerial ecuSerial(8, 9);

// ============================================================
// PACKET AND CONNECTION STATE
// ============================================================

byte packet[PACKET_SIZE];

bool connected = false;
bool haveValidData = false;

unsigned long lastHandshakeAttempt = 0;
unsigned long lastPoll = 0;
unsigned long lastDisplay = 0;
unsigned long validPackets = 0;
unsigned long badChecksums = 0;
unsigned long packetTimeouts = 0;

// ============================================================
// DECODED SENSOR DATA
// ============================================================

float rpm = 0;
float speedKmh = 0;
float ectC = 0;
float iatC = 0;
float o2Voltage = 0;
float mapVoltage = 0;
float mapInHg = 0;
float tpsPercent = 0;
float batteryVoltage = 0;
float ignitionAdvance = 0;
float knockRetard = 0;
float injectorMs = 0;
float injectorDuty = 0;

bool vtec = false;

// ============================================================
// SERIAL HELPERS
// ============================================================

void clearEcuInput() {
  while (ecuSerial.available() > 0) {
    ecuSerial.read();
  }
}

bool waitForByte(byte expected, unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (ecuSerial.available() > 0) {
      if (ecuSerial.read() == expected) {
        return true;
      }
    }
  }

  return false;
}

bool connectToEcu() {
  clearEcuInput();

  ecuSerial.write(ECU_WAKE);

  if (waitForByte(ECU_CONNECTED, 500)) {
    connected = true;
    lastPoll = millis();
    return true;
  }

  connected = false;
  return false;
}

// ============================================================
// RECEIVE ONE COMPLETE 52-BYTE PACKET
// ============================================================

bool receivePacket() {
  unsigned long start = millis();
  byte count = 0;

  while (count < PACKET_SIZE &&
         millis() - start < PACKET_TIMEOUT_MS) {

    if (ecuSerial.available() > 0) {
      packet[count++] = ecuSerial.read();
    }
  }

  if (count != PACKET_SIZE) {
    packetTimeouts++;
    return false;
  }

  return true;
}

// ============================================================
// CHECKSUM: SUM OF BYTES 0..50, MODULO 256
// ============================================================

bool checksumValid() {
  uint16_t sum = 0;

  for (byte i = 0; i <= 50; i++) {
    sum += packet[i];
  }

  return (byte)(sum & 0xFF) == packet[51];
}

// ============================================================
// SENSOR CONVERSIONS
// ============================================================

float rawTemperatureF(byte raw) {
  float x = raw / 51.0f;

  return 0.1423f * pow(x, 6)
       - 2.4938f * pow(x, 5)
       + 17.837f * pow(x, 4)
       - 68.698f * pow(x, 3)
       + 154.69f * pow(x, 2)
       - 232.75f * x
       + 284.24f;
}

float fahrenheitToCelsius(float f) {
  return (f - 32.0f) * 5.0f / 9.0f;
}

void decodePacket() {
  byte ectRaw = packet[0];
  byte iatRaw = packet[1];
  byte o2Raw = packet[2];
  byte mapRaw = packet[4];
  byte tpsRaw = packet[5];

  byte rpmLow = packet[6];
  byte rpmHigh = packet[7];

  byte knockRaw = packet[10];

  byte vssRaw = packet[16];
  byte injectorLow = packet[17];
  byte injectorHigh = packet[18];
  byte ignitionRaw = packet[19];

  byte vtecRaw = packet[23];
  byte batteryRaw = packet[25];

  uint16_t rpmDivisor =
      ((uint16_t)rpmHigh << 8) | rpmLow;

  // A zero divisor means RPM cannot be calculated.
  rpm = (rpmDivisor == 0)
      ? 0
      : 1875000.0f / rpmDivisor;

  speedKmh = vssRaw;

  ectC = fahrenheitToCelsius(rawTemperatureF(ectRaw));
  iatC = fahrenheitToCelsius(rawTemperatureF(iatRaw));

  o2Voltage = o2Raw / 51.0f;
  mapVoltage = mapRaw / 51.0f;

  // Formula from the supplied Honda Tuning Suite example.
  mapInHg = -10.87f * ((mapRaw * 5.0f) / 256.0f)
            + 30.48f;

  tpsPercent = 0.472637f * tpsRaw - 11.46119f;
  tpsPercent = constrain(tpsPercent, 0.0f, 100.0f);

  batteryVoltage = (26.0f * batteryRaw) / 270.0f;

  ignitionAdvance = 0.25f * ignitionRaw - 6.0f;
  knockRetard = 0.25f * knockRaw;

  uint16_t injectorRaw =
      ((uint16_t)injectorHigh << 8) | injectorLow;

  injectorMs = injectorRaw * 3.2f / 1000.0f;
  injectorDuty = rpm * injectorMs / 1200.0f;

  vtec = (vtecRaw & 0x01) != 0;

  haveValidData = true;
  validPackets++;
}

// ============================================================
// OUTPUT: CSV FOR PYTHON OR HUMAN-READABLE DASHBOARD
// ============================================================

void printCsv() {
  Serial.print("DATA,");

  Serial.print(rpm, 0); Serial.print(',');
  Serial.print(speedKmh, 0); Serial.print(',');
  Serial.print(ectC, 1); Serial.print(',');
  Serial.print(iatC, 1); Serial.print(',');
  Serial.print(o2Voltage, 2); Serial.print(',');
  Serial.print(mapVoltage, 2); Serial.print(',');
  Serial.print(mapInHg, 2); Serial.print(',');
  Serial.print(tpsPercent, 1); Serial.print(',');
  Serial.print(batteryVoltage, 2); Serial.print(',');
  Serial.print(ignitionAdvance, 2); Serial.print(',');
  Serial.print(knockRetard, 2); Serial.print(',');
  Serial.print(injectorMs, 2); Serial.print(',');
  Serial.print(injectorDuty, 1); Serial.print(',');
  Serial.println(vtec ? 1 : 0);
}

void printHuman() {
  Serial.println();
  Serial.println(F("========== HONDA DASH =========="));

  Serial.print(F("RPM:       "));
  Serial.println(rpm, 0);

  Serial.print(F("Speed:     "));
  Serial.print(speedKmh, 0);
  Serial.println(F(" km/h"));

  Serial.print(F("Coolant:   "));
  Serial.print(ectC, 1);
  Serial.println(F(" C"));

  Serial.print(F("Intake:    "));
  Serial.print(iatC, 1);
  Serial.println(F(" C"));

  Serial.print(F("O2:        "));
  Serial.print(o2Voltage, 2);
  Serial.println(F(" V"));

  Serial.print(F("MAP:       "));
  Serial.print(mapVoltage, 2);
  Serial.print(F(" V / "));
  Serial.print(mapInHg, 2);
  Serial.println(F(" inHg*"));

  Serial.print(F("TPS:       "));
  Serial.print(tpsPercent, 1);
  Serial.println(F(" %"));

  Serial.print(F("Battery:   "));
  Serial.print(batteryVoltage, 2);
  Serial.println(F(" V"));

  Serial.print(F("Ignition:  "));
  Serial.print(ignitionAdvance, 2);
  Serial.println(F(" deg"));

  Serial.print(F("Knock:     "));
  Serial.print(knockRetard, 2);
  Serial.println(F(" deg"));

  Serial.print(F("Injector:  "));
  Serial.print(injectorMs, 2);
  Serial.print(F(" ms / "));
  Serial.print(injectorDuty, 1);
  Serial.println(F(" %"));

  Serial.print(F("VTEC:      "));
  Serial.println(vtec ? F("ON") : F("OFF"));

  Serial.print(F("Valid packets: "));
  Serial.println(validPackets);

  Serial.print(F("Bad checksums: "));
  Serial.println(badChecksums);

  Serial.print(F("Timeouts: "));
  Serial.println(packetTimeouts);

  Serial.println(F("*MAP formula from supplied sample"));
  Serial.println(F("==============================="));
}

// ============================================================
// ARDUINO SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);
  ecuSerial.begin(ECU_BAUD);

  delay(250);

  Serial.println(F("HONDA Dash starting..."));
}

void loop() {
  unsigned long now = millis();

  if (!connected) {
    if (now - lastHandshakeAttempt >= HANDSHAKE_RETRY_MS) {
      lastHandshakeAttempt = now;

      if (connectToEcu()) {
        Serial.println(F("ECU datalogging connected."));
      } else {
        Serial.println(F("Waiting for ECU connection..."));
      }
    }

    return;
  }

  if (now - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = now;

    clearEcuInput();
    ecuSerial.write(ECU_PACKET_REQUEST);

    if (!receivePacket()) {
      connected = false;
      return;
    }

    if (!checksumValid()) {
      badChecksums++;
      return;
    }

    decodePacket();

#if OUTPUT_MODE == OUTPUT_CSV
    printCsv();
#endif
  }

#if OUTPUT_MODE == OUTPUT_HUMAN
  if (haveValidData &&
      millis() - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = millis();
    printHuman();
  }
#endif
}
