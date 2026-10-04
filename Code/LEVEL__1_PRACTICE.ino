#include <Arduino.h>

// ============================================================
// CATCH THE ROCKET - LEVEL 1
// PRACTICE TRACKER B TEST
//
// Level 1:
//   DESCENT
//   LAND
//
// Glitches OFF.
// ============================================================

#define TRACKER_GLITCHES 0


// ============================================================
// PRACTICE TRACKER B
// DO NOT MODIFY THIS BLOCK
// ============================================================

// ===== PRACTICE TRACKER B: do not modify =====
// Set TRACKER_GLITCHES to 1 or 0 *above* this block, never inside it.
static uint32_t trackerSeed = 4242u;
static uint32_t trackerRand() {
  trackerSeed = trackerSeed * 1664525u + 1013904223u;
  return trackerSeed;
}

float nextAltitude() {
  // This flight's secrets. Every tracker uses different ones.
  const int64_t H0 =  150000;   // altitude at t = 0, in cm
  const int64_t V0 =    4200;   // upward speed at t = 0, in cm/s
  const int64_t G  =     371;   // gravity, in cm/s^2
  const int64_t TD =   24000;   // parachute opens at this time, in ms
  const int64_t VD =    -850;   // speed under the parachute, in cm/s
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
// ONE PERSISTENT STATE STRUCT
// ============================================================

struct GroundState {

  // Reading reception / timing
  uint32_t readingsReceived;
  uint32_t worstCycles;

  // Level 1 least-squares sufficient statistics
  //
  // y = A + B*k
  //
  // k = time in milliseconds relative to 72.5 seconds
  // y = altitude in centimetres

  uint32_t descentN;

  int64_t sumK;
  int64_t sumY;
  int64_t sumK2;
  int64_t sumKY;

  // Serial command state
  char commandBuffer[32];
  uint8_t commandLength;
  bool commandOverflow;
};

GroundState state;


// Must remain <= 4096 bytes.
static_assert(
  sizeof(GroundState) <= 4096,
  "GroundState exceeds 4096 bytes"
);


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
// ALTITUDE CONVERSION
// Tracker returns metres as float.
// Internally tracker altitude is in centimetres.
// ============================================================

static inline int32_t altitudeToCm(float altitude) {

  return (int32_t)(altitude * 100.0f + 0.5f);
}


// ============================================================
// COMMAND HANDLER
//
// IMPORTANT:
// Parsing happens BEFORE timing.
//
// START TIMER
//      |
// compute answer
//      |
// STOP TIMER
//      |
// print answer
//
// Printing is NOT included in command cycles.
// ============================================================

void handleCommand(char *cmd) {

  // ----------------------------------------------------------
  // STEP 1: PARSE COMMAND
  // ----------------------------------------------------------

  CommandType type = CMD_INVALID;


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

    if (endPtr != cmd + 4) {

      while (*endPtr == ' ' || *endPtr == '\t') {
        endPtr++;
      }

      if (*endPtr == '\0' &&
          t >= 0.0f &&
          t <= 100.0f) {

        type = CMD_ALT;
      }
    }
  }


  // ----------------------------------------------------------
  // STEP 2: START COMMAND TIMING
  // ----------------------------------------------------------

  uint32_t start = ESP.getCycleCount();


  // ----------------------------------------------------------
  // STEP 3: COMPUTE ANSWER
  // ----------------------------------------------------------

  char answer[96];

  strcpy(answer, "ERR");


  // ==========================================================
  // STATS
  // ==========================================================

  if (type == CMD_STATS) {

    uint32_t bytesUsed =
        sizeof(GroundState);

    uint32_t worstReading =
        state.worstCycles;


    // Stop timing BEFORE printing.

    uint32_t commandCycles =
        (uint32_t)(
            ESP.getCycleCount() - start
        );


    Serial.print("bytes used: ");
    Serial.print(bytesUsed);
    Serial.println(" of 4096");

    Serial.print("worst reading: ");
    Serial.print(worstReading);
    Serial.println(" cycles");

    Serial.print("command cycles: ");
    Serial.println(commandCycles);

    return;
  }


  // ==========================================================
  // LEVEL 1: DESCENT / LAND
  // ==========================================================

  if (type == CMD_DESCENT ||
      type == CMD_LAND) {

    if (state.descentN < 2) {

      strcpy(answer, "ERR");
    }

    else {

      double N =
          (double)state.descentN;

      double SK =
          (double)state.sumK;

      double SY =
          (double)state.sumY;

      double SK2 =
          (double)state.sumK2;

      double SKY =
          (double)state.sumKY;


      // ------------------------------------------------------
      // Least-squares denominator
      // ------------------------------------------------------

      double denominator =
          N * SK2 - SK * SK;


      if (denominator == 0.0) {

        strcpy(answer, "ERR");
      }

      else {

        // ----------------------------------------------------
        // Slope B
        //
        // B = [N*sum(k*y) - sum(k)*sum(y)]
        //     --------------------------------
        //     [N*sum(k^2) - sum(k)^2]
        //
        // B is centimetres / millisecond.
        // ----------------------------------------------------

        double B =
            (N * SKY - SK * SY) /
            denominator;


        // ----------------------------------------------------
        // Intercept A
        // ----------------------------------------------------

        double A =
            (SY - B * SK) / N;


        // ====================================================
        // DESCENT
        // ====================================================

        if (type == CMD_DESCENT) {

          // cm/ms -> m/s
          //
          // 1 cm/ms = 10 m/s

          double descentMps =
              B * 10.0;


          snprintf(
              answer,
              sizeof(answer),
              "%.3f m/s",
              descentMps
          );
        }


        // ====================================================
        // LAND
        //
        // 0 = A + B*k
        //
        // k = -A/B
        // ====================================================

        else {

          if (B >= 0.0) {

            strcpy(answer, "ERR");
          }

          else {

            double kLand =
                -A / B;


            // k is relative to 72.5 seconds.

            double tLandMs =
                72500.0 + kLand;


            double tLand =
                tLandMs / 1000.0;


            if (tLand < 0.0 ||
                tLand > 1000.0) {

              strcpy(answer, "ERR");
            }

            else {

              snprintf(
                  answer,
                  sizeof(answer),
                  "%.3f s",
                  tLand
              );
            }
          }
        }
      }
    }
  }


  // ==========================================================
  // LATER LEVELS
  // ==========================================================

  else if (type == CMD_G ||
           type == CMD_APOGEE ||
           type == CMD_DEPLOY ||
           type == CMD_GLITCHES ||
           type == CMD_ALT) {

    strcpy(answer, "TODO");
  }


  // ==========================================================
  // INVALID
  // ==========================================================

  else {

    strcpy(answer, "ERR");
  }


  // ----------------------------------------------------------
  // STEP 4: STOP COMMAND TIMING
  // ----------------------------------------------------------

  uint32_t commandCycles =
      (uint32_t)(
          ESP.getCycleCount() - start
      );


  // ----------------------------------------------------------
  // STEP 5: PRINT
  // ----------------------------------------------------------

  Serial.println(answer);

  Serial.print("command cycles: ");
  Serial.println(commandCycles);
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  // ----------------------------------------------------------
  // Initialize ONE persistent state structure.
  // ----------------------------------------------------------

  state.readingsReceived = 0;

  state.worstCycles = 0;

  state.descentN = 0;

  state.sumK = 0;
  state.sumY = 0;
  state.sumK2 = 0;
  state.sumKY = 0;

  state.commandLength = 0;

  state.commandOverflow = false;

  state.commandBuffer[0] = '\0';


  Serial.println();

  Serial.println("================================");

  Serial.println(
      " CATCH THE ROCKET - LEVEL 1"
  );

  Serial.println("================================");

  Serial.println(
      "Receiving 100000 readings..."
  );


  // ==========================================================
  // EXACTLY 100000 nextAltitude() CALLS
  // ==========================================================

  for (uint32_t i = 0;
       i < 100000;
       i++) {


    // --------------------------------------------------------
    // Tracker execution is NOT included in our timing.
    // --------------------------------------------------------

    float altitude =
        nextAltitude();


    // --------------------------------------------------------
    // Start reading timing immediately after tracker returns.
    // --------------------------------------------------------

    uint32_t start =
        ESP.getCycleCount();


    // ========================================================
    // LEVEL 1:
    //
    // Use readings from t >= 45 seconds.
    //
    // Reading i occurs at:
    //
    //     t = i / 1000 seconds
    //
    // Therefore:
    //
    //     i >= 45000
    //
    // ========================================================

    if (i >= 45000) {


      // ------------------------------------------------------
      // Center time around 72.5 seconds.
      //
      // k = t(ms) - 72500
      // ------------------------------------------------------

      const int32_t k =
          (int32_t)i - 72500;


      // ------------------------------------------------------
      // Convert metres to centimetres.
      // ------------------------------------------------------

      const int32_t y =
          altitudeToCm(altitude);


      // ------------------------------------------------------
      // Build sufficient statistics one reading at a time.
      // ------------------------------------------------------

      state.descentN++;

      state.sumK +=
          (int64_t)k;

      state.sumY +=
          (int64_t)y;

      state.sumK2 +=
          (int64_t)k *
          (int64_t)k;

      state.sumKY +=
          (int64_t)k *
          (int64_t)y;
    }


    // --------------------------------------------------------
    // Reading completely processed.
    // --------------------------------------------------------

    state.readingsReceived++;


    // --------------------------------------------------------
    // End reading timing.
    // --------------------------------------------------------

    uint32_t cycles =
        (uint32_t)(
            ESP.getCycleCount() - start
        );


    if (cycles > state.worstCycles) {

      state.worstCycles = cycles;
    }
  }


  // ==========================================================
  // RECEIVING COMPLETE
  // ==========================================================

  Serial.println();

  Serial.println(
      "Receiving complete."
  );

  Serial.print(
      "Readings received: "
  );

  Serial.println(
      state.readingsReceived
  );

  Serial.print(
      "Descent samples: "
  );

  Serial.println(
      state.descentN
  );

  Serial.println("READY");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  while (Serial.available() > 0) {

    char c =
        (char)Serial.read();


    // ========================================================
    // END OF COMMAND
    // ========================================================

    if (c == '\n') {


      // ------------------------------------------------------
      // Overlong command
      // ------------------------------------------------------

      if (state.commandOverflow) {

        // Put an invalid marker into the buffer.
        state.commandBuffer[0] = '\0';

        handleCommand(
            state.commandBuffer
        );


        state.commandOverflow = false;

        state.commandLength = 0;

        state.commandBuffer[0] = '\0';
      }


      // ------------------------------------------------------
      // Normal command
      // ------------------------------------------------------

      else {

        state.commandBuffer[
            state.commandLength
        ] = '\0';


        if (state.commandLength > 0) {

          handleCommand(
              state.commandBuffer
          );
        }


        state.commandLength = 0;

        state.commandBuffer[0] = '\0';
      }
    }


    // ========================================================
    // IGNORE CR
    // ========================================================

    else if (c != '\r') {


      if (state.commandLength <
          sizeof(state.commandBuffer) - 1) {

        state.commandBuffer[
            state.commandLength++
        ] = c;
      }

      else {

        state.commandOverflow = true;
      }
    }
  }
}