#include <Arduino.h>

// ============================================================
// TRACKER SETTING
// Levels 0-3 use glitches OFF according to the PDF.
// Change ONLY this line when required.
// ============================================================

#define TRACKER_GLITCHES 0


// ===== OFFICIAL TRACKER: do not modify =====
// Set TRACKER_GLITCHES to 1 or 0 *above* this block, never inside it.

static uint32_t trackerSeed = 20261001u;
static uint32_t trackerRand() {
  trackerSeed = trackerSeed * 1664525u + 1013904223u;
  return trackerSeed;
}
float nextAltitude() {
  // This flight's secrets. Every tracker uses different ones.
  const int64_t H0 =   31200;   // altitude at t = 0, in cm
  const int64_t V0 =    7850;   // upward speed at t = 0, in cm/s
  const int64_t G  =     981;   // gravity, in cm/s^2
  const int64_t TD =   12000;   // parachute opens at this time, in ms
  const int64_t VD =    -450;   // speed under the parachute, in cm/s
  static int64_t t = 0;      // time of this reading, in ms
  static int quiet = 0;      // readings until a glitch is allowed again
  int64_t tc = (t < TD) ? t : TD;
  int64_t h = H0 + V0 * tc / 1000 - G * tc * tc / 2000000;
  if (t > TD) h += VD * (t - TD) / 1000;
  uint32_t r = trackerRand();
  if (quiet > 0) quiet--;
  if (TRACKER_GLITCHES && t >= 1000 && quiet == 0 && (r >> 24) < 2) {
    quiet = 10;                                     // glitch: 50 to 300 m off
    int64_t off = 5000 + (int64_t)((r >> 4) % 25001);
    h += ((r >> 3) & 1) ? off : -off;
    if (h < 0) h += 2 * off;
  } else {
    h += (int64_t)((r >> 8) % 401) - 200;           // noise: -2.00 to +2.00 m
  }
  t++;
  return (float)h / 100.0f;
}

// =========================================


// ============================================================
// LEVEL 0 GROUND STATION
//
// Rule 2:
// EVERYTHING persistent that belongs to our ground station
// is inside this ONE struct.
// ============================================================

struct GroundState {

  uint32_t readingsReceived;
  uint32_t worstCycles;

  char commandBuffer[32];
  uint8_t commandLength;
  bool commandOverflow;
};

GroundState state;


// ============================================================
// COMMAND TYPES
// ============================================================

enum CommandType {
  CMD_STATS,
  CMD_G,
  CMD_APOGEE,
  CMD_DEPLOY,
  CMD_DESCENT,
  CMD_LAND,
  CMD_GLITCHES,
  CMD_ALT,
  CMD_INVALID
};


// ============================================================
// COMMAND HANDLER
//
// IMPORTANT:
// Parsing happens BEFORE the cycle timer.
//
// Timing:
//     command parsed
//          ↓
//     START timer
//          ↓
//     compute answer
//          ↓
//     STOP timer
//          ↓
//     print answer
//
// Serial printing is NOT included in command cycles.
// ============================================================

void handleCommand(char *cmd) {

  // ----------------------------------------------------------
  // STEP 1: PARSE COMMAND
  // This is deliberately BEFORE timing.
  // ----------------------------------------------------------

  CommandType type = CMD_INVALID;

  bool altValid = false;

  if (strcmp(cmd, "STATS") == 0) {
    type = CMD_STATS;
  }
  else if (strcmp(cmd, "G") == 0) {
    type = CMD_G;
  }
  else if (strcmp(cmd, "APOGEE") == 0) {
    type = CMD_APOGEE;
  }
  else if (strcmp(cmd, "DEPLOY") == 0) {
    type = CMD_DEPLOY;
  }
  else if (strcmp(cmd, "DESCENT") == 0) {
    type = CMD_DESCENT;
  }
  else if (strcmp(cmd, "LAND") == 0) {
    type = CMD_LAND;
  }
  else if (strcmp(cmd, "GLITCHES") == 0) {
    type = CMD_GLITCHES;
  }
  else if (strncmp(cmd, "ALT ", 4) == 0) {

    char *endPtr = nullptr;

    float t = strtof(cmd + 4, &endPtr);

    // Valid only if:
    // 1. A number exists
    // 2. Nothing except whitespace follows it
    // 3. 0 <= t <= 100

    if (endPtr != cmd + 4) {

      while (*endPtr == ' ' ||
             *endPtr == '\t') {
        endPtr++;
      }

      if (*endPtr == '\0' &&
          t >= 0.0f &&
          t <= 100.0f) {

        altValid = true;
      }
    }

    if (altValid) {
      type = CMD_ALT;
    }
    else {
      type = CMD_INVALID;
    }
  }


  // ----------------------------------------------------------
  // STEP 2: START COMMAND TIMING
  //
  // Parsing is already finished.
  // ----------------------------------------------------------

  uint32_t start = ESP.getCycleCount();


  // ----------------------------------------------------------
  // STEP 3: COMPUTE / PREPARE ANSWER
  //
  // Level 0 is only the skeleton.
  // Actual mathematics comes in Levels 1 and 2.
  // ----------------------------------------------------------

  const char *answer = "ERR";

  uint32_t bytesUsed = 0;
  uint32_t worstReading = 0;

  if (type == CMD_STATS) {

    bytesUsed = sizeof(GroundState);
    worstReading = state.worstCycles;

    answer = "STATS";
  }

  else if (type == CMD_G ||
           type == CMD_APOGEE ||
           type == CMD_DEPLOY ||
           type == CMD_DESCENT ||
           type == CMD_LAND ||
           type == CMD_GLITCHES ||
           type == CMD_ALT) {

    // Level 0 placeholder.
    answer = "TODO";
  }

  else {

    // Malformed / unknown command.
    answer = "ERR";
  }


  // ----------------------------------------------------------
  // STEP 4: STOP COMMAND TIMING
  //
  // IMPORTANT:
  // No Serial.print() has happened yet.
  // Therefore printing is NOT included.
  // ----------------------------------------------------------

  uint32_t commandCycles =
      (uint32_t)(ESP.getCycleCount() - start);


  // ----------------------------------------------------------
  // STEP 5: PRINT ANSWER
  // Printing happens AFTER timing.
  // ----------------------------------------------------------

  if (type == CMD_STATS) {

    Serial.print("bytes used: ");
    Serial.print(bytesUsed);
    Serial.println(" of 4096");

    Serial.print("worst reading: ");
    Serial.print(worstReading);
    Serial.println(" cycles");
  }
  else {

    Serial.println(answer);
  }

  Serial.print("command cycles: ");
  Serial.println(commandCycles);
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);

  // Initialize our ONE persistent state structure.

  state.readingsReceived = 0;
  state.worstCycles = 0;
  state.commandLength = 0;
  state.commandOverflow = false;
  state.commandBuffer[0] = '\0';


  Serial.println();
  Serial.println("================================");
  Serial.println(" CATCH THE ROCKET - LEVEL 0");
  Serial.println("================================");
  Serial.println("Receiving 100000 readings...");


  // ==========================================================
  // EXACTLY 100000 calls to nextAltitude()
  //
  // DO NOT call nextAltitude() anywhere else.
  // ==========================================================

  for (uint32_t i = 0; i < 100000; i++) {

    // --------------------------------------------------------
    // Tracker execution is NOT measured.
    // --------------------------------------------------------

    float altitude = nextAltitude();


    // --------------------------------------------------------
    // Start measuring immediately after nextAltitude() returns.
    // --------------------------------------------------------

    uint32_t start = ESP.getCycleCount();


    // --------------------------------------------------------
    // Level 0 does not retain altitude yet.
    // --------------------------------------------------------

    (void)altitude;

    state.readingsReceived++;


    // --------------------------------------------------------
    // End measurement after this reading is completely handled.
    // --------------------------------------------------------

    uint32_t cycles =
        (uint32_t)(ESP.getCycleCount() - start);


    // Keep slowest reading.

    if (cycles > state.worstCycles) {
      state.worstCycles = cycles;
    }
  }


  // ==========================================================
  // RECEIVING FINISHED
  // ==========================================================

  Serial.println();
  Serial.println("Receiving complete.");

  Serial.print("Readings received: ");
  Serial.println(state.readingsReceived);

  Serial.println("READY");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  while (Serial.available() > 0) {

    char c = (char)Serial.read();


    // --------------------------------------------------------
    // END OF COMMAND
    // --------------------------------------------------------

    if (c == '\n') {

      // If command was too long, force ERR.

      if (state.commandOverflow) {

        state.commandBuffer[0] = '\0';

        // Directly generate an invalid command.
        handleCommand(state.commandBuffer);

        state.commandOverflow = false;
        state.commandLength = 0;
        state.commandBuffer[0] = '\0';
      }

      else {

        state.commandBuffer[state.commandLength] = '\0';

        if (state.commandLength > 0) {
          handleCommand(state.commandBuffer);
        }

        state.commandLength = 0;
        state.commandBuffer[0] = '\0';
      }
    }


    // --------------------------------------------------------
    // IGNORE CARRIAGE RETURN
    // --------------------------------------------------------

    else if (c != '\r') {

      // ------------------------------------------------------
      // Store command while there is room.
      // ------------------------------------------------------

      if (state.commandLength <
          sizeof(state.commandBuffer) - 1) {

        state.commandBuffer[state.commandLength++] = c;
      }

      else {

        // Command exceeded buffer capacity.
        // Remember this inside the ONE struct.
        state.commandOverflow = true;
      }
    }
  }
}
