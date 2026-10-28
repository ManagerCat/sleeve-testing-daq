/*
 * Cooling Sleeve Testing Jig - Elegoo Mega 2560 R3 DAQ Firmware
 * 
 * Target Board: Arduino / Elegoo Mega 2560 R3 (ATmega2560)
 * 
 * Description:
 *   High-speed data acquisition firmware streaming real-time sensor measurements
 *   over USB Serial (115200 baud) in JSON format at 20 Hz.
 * 
 * Hardware Wiring:
 *   - Digital Pin 2 (INT4): Koolance Flowmeter pulse output (INPUT_PULLUP)
 *   - Analog Pin A0: Coolant Inlet Thermistor (10k NTC divider)
 *   - Analog Pin A1: Coolant Outlet Thermistor (10k NTC divider)
 *   - Analog Pins A2 - A9: Mock Motor Edge Thermistors T1 - T8 (10k NTC dividers)
 *   - Analog Pins A10 - A15: Reserved for future thermal expansion
 *   - 5V Pin: Excitation power for all thermistor pull-up resistors
 *   - GND Pin: Common ground reference
 * 
 * JSON Output Format (emitted every 50 ms / 20 Hz):
 *   {"t":<millis>,"flow_count":<unsigned_long>,"a":[<a0>,<a1>,...,<a9>]}
 */

// Configuration Constants
const uint8_t FLOW_PIN = 2;              // Digital Pin 2 (Hardware Interrupt INT4 on Mega 2560)
const uint8_t NUM_ANALOG_CHANNELS = 10;  // 2 Coolant (A0, A1) + 8 Motor Edge (A2..A9)
const unsigned long SAMPLE_INTERVAL_MS = 50; // 50 ms = 20 Hz output rate
const unsigned long SERIAL_BAUD = 115200;

// Analog pin mapping array (A0 through A9, expandable to A15)
const uint8_t ANALOG_PINS[NUM_ANALOG_CHANNELS] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9
};

// Flow pulse counter (modified inside ISR, must be volatile)
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTimeMicros = 0;
const unsigned long FLOW_DEBOUNCE_MICROS = 5000; // 5 ms lockout window to ignore bounces/double counting

// Interrupt Service Routine for Flowmeter Pulses
void onFlowPulse() {
  unsigned long now = micros();
  if (now - lastPulseTimeMicros >= FLOW_DEBOUNCE_MICROS) {
    pulseCount++;
    lastPulseTimeMicros = now;
  }
}

unsigned long lastSampleTime = 0;

void setup() {
  // Initialize USB serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000) {
    // Wait for native USB if needed, with timeout
  }

  // Configure Flowmeter pin with internal pull-up resistor
  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), onFlowPulse, FALLING);

  // Configure analog input pins
  for (uint8_t i = 0; i < NUM_ANALOG_CHANNELS; i++) {
    pinMode(ANALOG_PINS[i], INPUT);
  }

  // Print startup greeting comment line (Python parser ignores non-JSON lines)
  Serial.println(F("{\"status\":\"ready\",\"board\":\"Elegoo Mega 2560 R3\",\"channels\":10,\"rate_hz\":20}"));
}

void loop() {
  unsigned long now = millis();

  // Non-blocking 20 Hz periodic sample and transmit
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    // Atomically capture the pulse counter
    noInterrupts();
    unsigned long currentPulseCount = pulseCount;
    interrupts();

    // Read all configured analog channels
    // (A double-read or single fast read ensures analog multiplexer settling)
    int adcValues[NUM_ANALOG_CHANNELS];
    for (uint8_t i = 0; i < NUM_ANALOG_CHANNELS; i++) {
      adcValues[i] = analogRead(ANALOG_PINS[i]);
    }

    // Construct and stream JSON line
    Serial.print(F("{\"t\":"));
    Serial.print(now);
    Serial.print(F(",\"flow_count\":"));
    Serial.print(currentPulseCount);
    Serial.print(F(",\"a\":["));
    for (uint8_t i = 0; i < NUM_ANALOG_CHANNELS; i++) {
      Serial.print(adcValues[i]);
      if (i < NUM_ANALOG_CHANNELS - 1) {
        Serial.print(F(","));
      }
    }
    Serial.println(F("]}"));
  }
}
