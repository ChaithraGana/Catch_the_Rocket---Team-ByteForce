#include <Arduino.h>

// ============================================================
// CATCH THE ROCKET - LEVEL 2
// PRACTICE TRACKER B
//
// Level 1:
//   DESCENT
//   LAND
//
// Level 2:
//   G
//   APOGEE
//
// Glitches OFF for Level 2.
// ============================================================

#define TRACKER_GLITCHES 0


// ============================================================
// ===== PRACTICE TRACKER B: DO NOT MODIFY =====
// ============================================================

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
  static int64_t t = 0;          // time of this reading, in ms
  static int quiet = 0;          // readings until a glitch is allowed again
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

  // ----------------------------------------------------------
  // General
  // ----------------------------------------------------------

  uint32_t readingsReceived;
  uint32_t worstCycles;


  // ----------------------------------------------------------
  // LEVEL 1 - DESCENT
  //
  // y = A + B*k
  //
  // k = milliseconds relative to 72.5 seconds
  // y = centimetres
  // ----------------------------------------------------------

  uint32_t descentN;

  int64_t descentSumK;
  int64_t descentSumY;
  int64_t descentSumK2;
  int64_t descentSumKY;


  // ----------------------------------------------------------
  // LEVEL 2 - COAST
  //
  // First 8 seconds = 8000 readings.
  //
  // q = 2*i - 7999
  //
  // q is centered around zero.
  //
  // Model:
  //
  // y = A + B*q + C*q^2
  //
  // DOUBLE is used deliberately for numerical safety.
  // ----------------------------------------------------------

  uint32_t coastN;

  double coastSumQ2;
  double coastSumQ4;
  double coastSumY;
  double coastSumQY;
  double coastSumQ2Y;


  // ----------------------------------------------------------
  // COMMAND INPUT
  // ----------------------------------------------------------

  char commandBuffer[32];
  uint8_t commandLength;
  bool commandOverflow;
};


// ============================================================
// MEMORY CHECK
// ============================================================

GroundState state;

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
// METRES -> CENTIMETRES
// ============================================================

static inline int32_t altitudeToCm(float altitude) {

  return (int32_t)(altitude * 100.0f + 0.5f);
}


// ============================================================
// COMMAND HANDLER
// ============================================================

void handleCommand(char *cmd) {

  // ==========================================================
  // PARSE
  // ==========================================================

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

      while (*endPtr == ' ' ||
             *endPtr == '\t') {

        endPtr++;
      }

      if (*endPtr == '\0' &&
          t >= 0.0f &&
          t <= 100.0f) {

        type = CMD_ALT;
      }
    }
  }


  // ==========================================================
  // START COMMAND TIMING
  // Parsing is already finished.
  // ==========================================================

  uint32_t start =
      ESP.getCycleCount();


  // ==========================================================
  // STATS
  // ==========================================================

  if (type == CMD_STATS) {

    uint32_t bytesUsed =
        sizeof(GroundState);

    uint32_t worstReading =
        state.worstCycles;


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
  // LEVEL 1
  // DESCENT / LAND
  // ==========================================================

  if (type == CMD_DESCENT ||
      type == CMD_LAND) {

    char answer[64];

    strcpy(answer, "ERR");


    if (state.descentN >= 2) {

      double N =
          (double)state.descentN;

      double SK =
          (double)state.descentSumK;

      double SY =
          (double)state.descentSumY;

      double SK2 =
          (double)state.descentSumK2;

      double SKY =
          (double)state.descentSumKY;


      double denominator =
          N * SK2 - SK * SK;


      if (denominator != 0.0) {

        double B =
          (N * SKY - SK * SY) /
          denominator;

        double A =
          (SY - B * SK) / N;


        // ----------------------------------------------------
        // DESCENT
        // ----------------------------------------------------

        if (type == CMD_DESCENT) {

          double descentMps =
              B * 10.0;

          snprintf(
            answer,
            sizeof(answer),
            "%.3f m/s",
            descentMps
          );
        }


        // ----------------------------------------------------
        // LAND
        // ----------------------------------------------------

        else {

          if (B < 0.0) {

            double kLand =
                -A / B;

            double tLandMs =
                72500.0 + kLand;

            double tLand =
                tLandMs / 1000.0;


            if (tLand >= 0.0 &&
                tLand <= 1000.0) {

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


    uint32_t commandCycles =
        (uint32_t)(
          ESP.getCycleCount() - start
        );


    Serial.println(answer);

    Serial.print("command cycles: ");
    Serial.println(commandCycles);

    return;
  }


  // ==========================================================
  // LEVEL 2
  // G / APOGEE
  // ==========================================================

  if (type == CMD_G ||
      type == CMD_APOGEE) {

    char answer[96];

    strcpy(answer, "ERR");


    if (state.coastN >= 3) {

      double N =
          (double)state.coastN;

      double S2 =
          state.coastSumQ2;

      double S4 =
          state.coastSumQ4;

      double SY =
          state.coastSumY;

      double SQY =
          state.coastSumQY;

      double SQ2Y =
          state.coastSumQ2Y;


      // ------------------------------------------------------
      // Because q is perfectly centered:
      //
      // sum(q)  = 0
      // sum(q^3) = 0
      //
      // Therefore:
      //
      // B = sum(q*y) / sum(q^2)
      //
      // C =
      // [N*sum(q²y) - sum(q²)*sum(y)]
      // --------------------------------
      // [N*sum(q⁴) - sum(q²)²]
      //
      // A =
      // [sum(y) - C*sum(q²)] / N
      // ------------------------------------------------------

      double denominator =
          N * S4 - S2 * S2;


      if (denominator != 0.0 &&
          S2 != 0.0) {

        double B =
            SQY / S2;

        double C =
            (N * SQ2Y - S2 * SY) /
            denominator;

        double A =
            (SY - C * S2) / N;


        // ====================================================
        // G
        // ====================================================

        if (type == CMD_G) {

          // q increases by 2 every 1 ms.
          //
          // q = 2000*t - 7999
          //
          // Therefore:
          //
          // d²y/dt² = C * 2 * 2000²
          //
          // y is centimetres.
          // Convert cm/s² to m/s² by /100.
          //
          // gravity = -80000*C

          double gravity =
              -80000.0 * C;


          snprintf(
            answer,
            sizeof(answer),
            "%.3f m/s^2",
            gravity
          );
        }


        // ====================================================
        // APOGEE
        // ====================================================

        else {

          if (C < 0.0) {

            // dy/dq = B + 2*C*q

            double qApogee =
                -B / (2.0 * C);


            // q = 2000*t - 7999
            //
            // t = (q + 7999) / 2000
            //
            // Center time = 3.9995 s.

            double tApogee =
                3.9995 +
                qApogee / 2000.0;


            // Vertex of quadratic

            double yApogee =
                A -
                (B * B) /
                (4.0 * C);


            double hApogee =
                yApogee / 100.0;


            if (tApogee >= 0.0 &&
                tApogee <= 100.0) {

              snprintf(
                answer,
                sizeof(answer),
                "%.3f s %.2f m",
                tApogee,
                hApogee
              );
            }
          }
        }
      }
    }


    uint32_t commandCycles =
        (uint32_t)(
          ESP.getCycleCount() - start
        );


    Serial.println(answer);

    Serial.print("command cycles: ");
    Serial.println(commandCycles);

    return;
  }


  // ==========================================================
  // LATER LEVELS
  // ==========================================================

  if (type == CMD_DEPLOY ||
      type == CMD_ALT ||
      type == CMD_GLITCHES) {

    uint32_t commandCycles =
        (uint32_t)(
          ESP.getCycleCount() - start
        );

    Serial.println("TODO");

    Serial.print("command cycles: ");
    Serial.println(commandCycles);

    return;
  }


  // ==========================================================
  // INVALID
  // ==========================================================

  uint32_t commandCycles =
      (uint32_t)(
        ESP.getCycleCount() - start
      );

  Serial.println("ERR");

  Serial.print("command cycles: ");
  Serial.println(commandCycles);
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);


  // ==========================================================
  // INITIALIZE STATE
  // ==========================================================

  state.readingsReceived = 0;

  state.worstCycles = 0;


  // Level 1

  state.descentN = 0;

  state.descentSumK = 0;
  state.descentSumY = 0;
  state.descentSumK2 = 0;
  state.descentSumKY = 0;


  // Level 2

  state.coastN = 0;

  state.coastSumQ2 = 0.0;
  state.coastSumQ4 = 0.0;
  state.coastSumY = 0.0;
  state.coastSumQY = 0.0;
  state.coastSumQ2Y = 0.0;


  // Commands

  state.commandLength = 0;
  state.commandOverflow = false;
  state.commandBuffer[0] = '\0';


  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    " CATCH THE ROCKET - LEVEL 2"
  );

  Serial.println(
    " PRACTICE TRACKER B"
  );

  Serial.println(
    "================================"
  );

  Serial.println(
    "Receiving 100000 readings..."
  );


  // ==========================================================
  // EXACTLY 100000 CALLS
  // ==========================================================

  for (uint32_t i = 0;
       i < 100000;
       i++) {

    // --------------------------------------------------------
    // Tracker call
    // --------------------------------------------------------

    float altitude =
        nextAltitude();


    // --------------------------------------------------------
    // Start timing immediately after tracker returns
    // --------------------------------------------------------

    uint32_t start =
        ESP.getCycleCount();


    // ========================================================
    // LEVEL 2 COAST
    //
    // First 8 seconds = i 0 through 7999.
    // ========================================================

    if (i < 8000) {

      const int32_t q =
          2 * (int32_t)i - 7999;

      const double qd =
          (double)q;

      const double q2 =
          qd * qd;

      const double q4 =
          q2 * q2;

      const double y =
          (double)altitudeToCm(altitude);


      state.coastN++;

      state.coastSumQ2 +=
          q2;

      state.coastSumQ4 +=
          q4;

      state.coastSumY +=
          y;

      state.coastSumQY +=
          qd * y;

      state.coastSumQ2Y +=
          q2 * y;
    }


    // ========================================================
    // LEVEL 1 DESCENT
    //
    // t >= 45 seconds
    // ========================================================

    if (i >= 45000) {

      const int32_t k =
          (int32_t)i - 72500;

      const int32_t y =
          altitudeToCm(altitude);


      state.descentN++;

      state.descentSumK +=
          (int64_t)k;

      state.descentSumY +=
          (int64_t)y;

      state.descentSumK2 +=
          (int64_t)k *
          (int64_t)k;

      state.descentSumKY +=
          (int64_t)k *
          (int64_t)y;
    }


    state.readingsReceived++;


    // --------------------------------------------------------
    // Stop reading timing
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
  // COMPLETE
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
    "Coast samples: "
  );

  Serial.println(
    state.coastN
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
    // END COMMAND
    // ========================================================

    if (c == '\n') {

      if (state.commandOverflow) {

        state.commandBuffer[0] = '\0';

        handleCommand(
          state.commandBuffer
        );

        state.commandOverflow = false;

        state.commandLength = 0;

        state.commandBuffer[0] = '\0';
      }

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
