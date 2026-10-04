#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TRACKER_GLITCHES 1


// ============================================================
// ===== OFFICIAL TRACKER: DO NOT MODIFY =====================
// ============================================================

static uint32_t trackerSeed = 20261001u;
static uint32_t trackerRand() {
  trackerSeed = trackerSeed * 1664525u + 1013904223u;
  return trackerSeed;
}

float nextAltitude() {
  // This flight's secrets. Every tracker uses different ones.
  const int64_t H0 =   31200;
  const int64_t V0 =    7850;
  const int64_t G  =     981;
  const int64_t TD =   12000;
  const int64_t VD =    -450;
  static int64_t t = 0;
  static int quiet = 0;
  int64_t tc = (t < TD) ? t : TD;
  int64_t h = H0 + V0 * tc / 1000 - G * tc * tc / 2000000;
  if (t > TD) h += VD * (t - TD) / 1000;
  uint32_t r = trackerRand();
  if (quiet > 0) quiet--;
  if (TRACKER_GLITCHES && t >= 1000 && quiet == 0 && (r >> 24) < 2) {
    quiet = 10;
    int64_t off = 5000 + (int64_t)((r >> 4) % 25001);
    h += ((r >> 3) & 1) ? off : -off;
    if (h < 0) h += 2 * off;
  } else {
    h += (int64_t)((r >> 8) % 401) - 200;
  }
  t++;
  return (float)h / 100.0f;
}


// ============================================================
// CONSTANTS
// ============================================================

#define TOTAL_READINGS       100000UL
#define FIRST_COAST          8000UL
#define DETECT_WINDOW        1024UL
#define RING_SIZE            DETECT_WINDOW
#define GLITCH_HISTORY_SIZE  5UL

// Time coordinate used for fitting, in milliseconds.
#define FIT_CENTER_MS        22500LL

// Deployment detection threshold.
// Difference is measured in centimetres.
#define DETECT_THRESHOLD_CM  500.0f


// ============================================================
// ONE PERSISTENT STATE STRUCT
// ============================================================

struct GroundState {

  // ---------------- Reading statistics ----------------
  uint32_t readingsReceived;
  uint32_t worstCycles;

  // ---------------- First 8-second fit ----------------
  uint32_t firstN;

  int64_t firstSumQ2;
  int64_t firstSumY;
  int64_t firstSumQY;
  int64_t firstSumQ2Y;

  double firstA;
  double firstB;
  double firstC;

  bool firstReady;

  // ---------------- Coast accumulators ----------------
  uint32_t coastN;

  int64_t coastSumY;
  int64_t coastSumXY;
  int64_t coastSumX2Y;

  uint32_t coastLastIndex;

  // ---------------- Parachute accumulators ----------------
  uint32_t paraN;

  int64_t paraSumY;
  int64_t paraSumX;
  int64_t paraSumX2;
  int64_t paraSumXY;

  // ---------------- Deployment ring ----------------
  uint16_t ring[RING_SIZE];

  // Last five RAW readings for causal glitch detection.
  // A glitch is at least 50 m from truth and glitches are
  // at least 10 readings apart, so a 5-point median is robust.
  uint16_t rawHistory[GLITCH_HISTORY_SIZE];
  uint8_t rawHistoryCount;
  uint8_t rawHistoryPos;
  uint32_t glitchCount;

  // ---------------- Detection state ----------------
  bool deploymentDetected;
  bool deploymentFinalized;

  uint32_t detectionIndex;
  uint32_t frozenStart;


  // ---------------- Final results ----------------
  double deploymentTime;
  double deploymentAltitude;

  // Coast:
  // h(t) = A + B(t-center) + C(t-center)^2
  double coastA;
  double coastB;
  double coastC;
  double coastCenter;

  // Parachute:
  // h(t) = A + B(t-center)
  double paraA;
  double paraB;
  double paraCenter;

  // ---------------- Command input ----------------
  char commandBuffer[32];
  uint8_t commandLength;
  bool commandOverflow;
};


// MUST be the only persistent reading-derived state.
GroundState state;


// Level 3 requirement: persistent state <= 4096 bytes.
static_assert(
  sizeof(GroundState) <= 4096,
  "GroundState exceeds 4096 bytes"
);


// ============================================================
// CONVERSION
// ============================================================

static inline int64_t altitudeToCm(float h) {

  return (int64_t)(h * 100.0f + 0.5f);
}


// Store ring-buffer altitude in 5 cm units.
// Derived from the already converted cm value to avoid
// another floating-point multiplication.
static inline uint16_t cmTo5cm(int64_t cm) {

  int64_t v = (cm + 2) / 5;

  if (v < 0)
    v = 0;

  if (v > 60000)
    v = 60000;

  return (uint16_t)v;
}


// ============================================================
// ROBUST GLITCH DETECTION
// ============================================================

static inline uint16_t median5(
  uint16_t a,
  uint16_t b,
  uint16_t c,
  uint16_t d,
  uint16_t e
) {

  uint16_t v[5] = {a, b, c, d, e};

  // Small fixed-size insertion sort. This is local working
  // memory, not persistent reading-derived state.
  for (int i = 1; i < 5; i++) {
    uint16_t key = v[i];
    int j = i - 1;

    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }

    v[j + 1] = key;
  }

  return v[2];
}


static inline uint16_t robustCurrent(
  uint16_t current,
  bool &isGlitch
) {

  isGlitch = false;

  // No glitches exist in the first 1000 readings.
  // We still build the history during this period.
  if (state.rawHistoryCount < GLITCH_HISTORY_SIZE) {

    state.rawHistory[state.rawHistoryPos] = current;

    state.rawHistoryPos++;
    if (state.rawHistoryPos >= GLITCH_HISTORY_SIZE)
      state.rawHistoryPos = 0;

    state.rawHistoryCount++;
    return current;
  }

  // Add current RAW reading first. The five entries now contain
  // the current sample and the previous four raw samples.
  state.rawHistory[state.rawHistoryPos] = current;

  state.rawHistoryPos++;
  if (state.rawHistoryPos >= GLITCH_HISTORY_SIZE)
    state.rawHistoryPos = 0;

  // Read the five values. Order does not matter for median5().
  uint16_t a = state.rawHistory[0];
  uint16_t b = state.rawHistory[1];
  uint16_t c = state.rawHistory[2];
  uint16_t d = state.rawHistory[3];
  uint16_t e = state.rawHistory[4];

  uint16_t med = median5(a, b, c, d, e);

  // Stored in 5 cm units. 2000 cm = 20 m.
  // Normal noise can differ by at most about 4 m between two
  // clean readings, while a glitch is at least 50 m away.
  int32_t residual =
    abs((int32_t)current - (int32_t)med) * 5;

  if (state.readingsReceived >= 1000UL && residual > 2000) {
    isGlitch = true;
    state.glitchCount++;

    // Replace the corrupted sample by the local median.
    return med;
  }

  return current;
}


// ============================================================
// FIRST 8-SECOND QUADRATIC FIT
// ============================================================

void calculateFirstFit() {

  const double N = (double)state.firstN;

  // q = 2*i - 7999
  // q represents time in half-millisecond units,
  // centered around 3.9995 s.

  const double m = 4000.0;

  const double S2 =
    2.0 * m *
    (2.0 * m - 1.0) *
    (2.0 * m + 1.0) / 3.0;

  const double S4 =
    2.0 * m *
    (2.0 * m - 1.0) *
    (2.0 * m + 1.0) *
    (12.0 * m * m - 7.0) / 15.0;

  const double Sy =
    (double)state.firstSumY;

  const double Sqy =
    (double)state.firstSumQY;

  const double Sq2y =
    (double)state.firstSumQ2Y;

  const double denominator =
    N * S4 - S2 * S2;

  if (fabs(denominator) < 1e-12) {

    state.firstReady = false;
    return;
  }

  double Bq =
    Sqy / S2;

  double Cq =
    (N * Sq2y - S2 * Sy) /
    denominator;

  double A =
    (Sy - Cq * S2) / N;

  // q = 2*t_ms - 7999
  // Therefore:
  //
  // B(seconds) = Bq * 2000
  // C(seconds²) = Cq * 4,000,000

  state.firstA = A;
  state.firstB = Bq * 2000.0;
  state.firstC = Cq * 4000000.0;

  state.firstReady = true;
}


// ============================================================
// ACCUMULATE COAST SAMPLE
// ============================================================

void addCoast(
  uint32_t index,
  int64_t y
) {

  int64_t x =
    (int64_t)index - FIT_CENTER_MS;

  int64_t x2 =
    x * x;

  state.coastN++;

  state.coastSumY += y;

  state.coastSumXY +=
    x * y;

  state.coastSumX2Y +=
    x2 * y;

  state.coastLastIndex =
    index;
}


// ============================================================
// ACCUMULATE PARACHUTE SAMPLE
// ============================================================

void addParachute(
  uint32_t index,
  int64_t y
) {

  int64_t x =
    (int64_t)index - FIT_CENTER_MS;

  state.paraN++;

  state.paraSumY += y;

  state.paraSumX += x;

  state.paraSumX2 +=
    x * x;

  state.paraSumXY +=
    x * y;
}


// ============================================================
// POWER SUM
// ============================================================

double prefixPower(
  long long n,
  int p
) {

  if (n <= 0)
    return 0.0;

  double x = (double)n;

  if (p == 1)
    return x * (x + 1.0) / 2.0;

  if (p == 2)
    return x * (x + 1.0) *
           (2.0 * x + 1.0) / 6.0;

  if (p == 3) {

    double s =
      x * (x + 1.0) / 2.0;

    return s * s;
  }

  return
    x * (x + 1.0) *
    (2.0 * x + 1.0) *
    (3.0 * x * x + 3.0 * x - 1.0)
    / 30.0;
}


double rangePower(
  long long lo,
  long long hi,
  int p
) {

  if (lo > hi)
    return 0.0;

  if (lo >= 0) {

    return
      prefixPower(hi, p) -
      prefixPower(lo - 1, p);
  }

  if (hi < 0) {

    double magnitude =
      prefixPower(-lo, p) -
      prefixPower(-hi - 1, p);

    if (p & 1)
      return -magnitude;

    return magnitude;
  }

  double negativeMagnitude =
    prefixPower(-lo, p);

  double positive =
    prefixPower(hi, p);

  if (p & 1)
    return -negativeMagnitude + positive;

  return negativeMagnitude + positive;
}


// ============================================================
// FINAL COAST QUADRATIC FIT
// ============================================================

bool solveCoast(
  double &A,
  double &B,
  double &C,
  double &centerTime
) {

  if (state.coastN < 3)
    return false;

  long long last =
    (long long)state.coastLastIndex;

  long long lo =
    -FIT_CENTER_MS;

  long long hi =
    last - FIT_CENTER_MS;

  double S1 =
    rangePower(lo, hi, 1);

  double S2 =
    rangePower(lo, hi, 2);

  double S3 =
    rangePower(lo, hi, 3);

  double S4 =
    rangePower(lo, hi, 4);

  double N =
    (double)state.coastN;

  double SY =
    (double)state.coastSumY;

  double SXY =
    (double)state.coastSumXY;

  double SX2Y =
    (double)state.coastSumX2Y;


  // Shift coordinate from FIT_CENTER_MS
  // to actual center of available coast interval.

  double newCenterMs =
    (double)last / 2.0;

  double shift =
    (double)FIT_CENTER_MS -
    newCenterMs;


  double Z1 =
    S1 + shift * N;

  double Z2 =
    S2 +
    2.0 * shift * S1 +
    shift * shift * N;

  double Z3 =
    S3 +
    3.0 * shift * S2 +
    3.0 * shift * shift * S1 +
    shift * shift * shift * N;

  double Z4 =
    S4 +
    4.0 * shift * S3 +
    6.0 * shift * shift * S2 +
    4.0 * shift * shift * shift * S1 +
    shift * shift * shift * shift * N;


  double ZY =
    SY;

  double ZY1 =
    SXY +
    shift * SY;

  double ZY2 =
    SX2Y +
    2.0 * shift * SXY +
    shift * shift * SY;


  // Convert ms powers to seconds powers.

  double M1 =
    Z1 / 1000.0;

  double M2 =
    Z2 / 1000000.0;

  double M3 =
    Z3 / 1000000000.0;

  double M4 =
    Z4 / 1000000000000.0;

  double R1 =
    ZY1 / 1000.0;

  double R2 =
    ZY2 / 1000000.0;


  // Normal equations:
  //
  // [N  M1 M2] [A]   [SY]
  // [M1 M2 M3] [B] = [R1]
  // [M2 M3 M4] [C]   [R2]

  double m[3][4] = {

    {N,  M1, M2, SY},

    {M1, M2, M3, R1},

    {M2, M3, M4, R2}
  };


  // Gaussian elimination.

  for (int col = 0; col < 3; col++) {

    int pivot = col;

    double largest =
      fabs(m[col][col]);

    for (
      int row = col + 1;
      row < 3;
      row++
    ) {

      double v =
        fabs(m[row][col]);

      if (v > largest) {

        largest = v;
        pivot = row;
      }
    }

    if (largest < 1e-12)
      return false;


    if (pivot != col) {

      for (
        int j = col;
        j < 4;
        j++
      ) {

        double temp =
          m[col][j];

        m[col][j] =
          m[pivot][j];

        m[pivot][j] =
          temp;
      }
    }


    double divisor =
      m[col][col];

    for (
      int j = col;
      j < 4;
      j++
    ) {

      m[col][j] /=
        divisor;
    }


    for (
      int row = 0;
      row < 3;
      row++
    ) {

      if (row == col)
        continue;

      double factor =
        m[row][col];

      for (
        int j = col;
        j < 4;
        j++
      ) {

        m[row][j] -=
          factor * m[col][j];
      }
    }
  }


  A = m[0][3];
  B = m[1][3];
  C = m[2][3];

  centerTime =
    newCenterMs / 1000.0;

  return true;
}


// ============================================================
// FINAL PARACHUTE LINEAR FIT
// ============================================================

bool solveParachute(
  double &A,
  double &B,
  double &centerTime
) {

  if (state.paraN < 2)
    return false;

  double N =
    (double)state.paraN;

  // x is stored in milliseconds relative to 22.5 s.
  double Sx =
    (double)state.paraSumX /
    1000.0;

  double Sx2 =
    (double)state.paraSumX2 /
    1000000.0;

  double Sy =
    (double)state.paraSumY;

  double Sxy =
    (double)state.paraSumXY /
    1000.0;

  double denominator =
    N * Sx2 -
    Sx * Sx;

  if (fabs(denominator) < 1e-12)
    return false;

  // Slope in cm/s.
  B =
    (N * Sxy - Sx * Sy) /
    denominator;

  // The parachute accumulators contain only safely
  // post-deployment samples, so the long-range descent
  // slope is not biased by the transition region.
  //
  // Use the mean time as the coordinate origin.
  // This is IMPORTANT:
  //
  //   z = t - centerTime
  //
  // At the mean time, the fitted straight line has
  // altitude equal to mean altitude.
  double meanX =
    Sx / N;

  double meanY =
    Sy / N;

  A =
    meanY;

  centerTime =
    22.5 + meanX;

  return true;
}


// ============================================================
// EVALUATE COAST
// ============================================================

double evalCoast(
  double t
) {

  double z =
    t - state.coastCenter;

  return
    state.coastA +
    state.coastB * z +
    state.coastC * z * z;
}


// ============================================================
// EVALUATE PARACHUTE
// ============================================================

double evalParachute(
  double t
) {

  double z =
    t - state.paraCenter;

  return
    state.paraA +
    state.paraB * z;
}


// ============================================================
// FIND DEPLOYMENT CROSSING
// ============================================================

bool findCrossing(
  double coastA,
  double coastB,
  double coastC,
  double coastCenter,

  double paraA,
  double paraB,
  double paraCenter,

  double &deployTime
) {

  // coast(t) = parachute(t)

  double qa =
    coastC;

  double qb =
    coastB - paraB;

  double qc =
    coastA -
    paraA -
    paraB *
    (coastCenter - paraCenter);


  if (fabs(qa) < 1e-15)
    return false;


  double discriminant =
    qb * qb -
    4.0 * qa * qc;

  if (discriminant < 0.0)
    return false;


  double root =
    sqrt(discriminant);


  double z1 =
    (-qb + root) /
    (2.0 * qa);

  double z2 =
    (-qb - root) /
    (2.0 * qa);


  double t1 =
    coastCenter + z1;

  double t2 =
    coastCenter + z2;


  double candidates[2] =
  {
    t1,
    t2
  };


  bool found = false;

  double best = 0.0;


  for (int k = 0; k < 2; k++) {

    double t =
      candidates[k];


    // Guaranteed deployment range.
    if (t < 8.0 ||
        t > 45.0)
      continue;


    // Coast velocity in cm/s.
    double coastVelocity =
      coastB +
      2.0 *
      coastC *
      (t - coastCenter);


    // Parachute velocity in cm/s.
    double paraVelocity =
      paraB;


    // At deployment, rocket is falling
    // at least 10 m/s faster than parachute.
    //
    // Downward velocities are negative.
    //
    // Example:
    // coast = -14 m/s
    // parachute = -4.5 m/s
    //
    // coast <= parachute - 10.

    if (coastVelocity >
        paraVelocity - 1000.0)
      continue;


    if (!found ||
        t > best) {

      best = t;
      found = true;
    }
  }


  if (!found)
    return false;


  deployTime =
    best;

  return true;
}


// ============================================================
// FINALIZE DEPLOYMENT
// ============================================================

bool finalizeDeployment() {
  // Once the final models have been built, reuse them.
  // This prevents repeating the expensive Level-3 fitting
  // work for every command.

  if (!state.deploymentDetected)
    return false;

  if (state.deploymentFinalized)
    return true;


  // ----------------------------------------------------------
  // Initial fits.
  // ----------------------------------------------------------

  double ca;
  double cb;
  double cc;
  double ccenter;

  if (!solveCoast(
        ca,
        cb,
        cc,
        ccenter
      )) {

    return false;
  }


  double pa;
  double pb;
  double pcenter;

  if (!solveParachute(
        pa,
        pb,
        pcenter
      )) {

    return false;
  }


  // ----------------------------------------------------------
  // First estimate of deployment.
  // ----------------------------------------------------------

  double preliminaryTime;

  if (!findCrossing(
        ca,
        cb,
        cc,
        ccenter,

        pa,
        pb,
        pcenter,

        preliminaryTime
      )) {

    return false;
  }


  uint32_t boundary =
    (uint32_t)lround(
      preliminaryTime *
      1000.0
    );


  // Keep boundary inside frozen region.

  if (boundary <
      state.frozenStart) {

    boundary =
      state.frozenStart;
  }


  if (boundary >=
      state.detectionIndex) {

    boundary =
      state.detectionIndex - 1;
  }


  // ----------------------------------------------------------
  // The transition region is intentionally not added again.
  // Coast samples are already accumulated only when they leave
  // the rolling window, and the final parachute fit starts only
  // after a full detection window. This avoids storing a second
  // copy of the transition region.
  // ----------------------------------------------------------

  // ----------------------------------------------------------
  // Refit after classification.
  // ----------------------------------------------------------

  if (!solveCoast(
        ca,
        cb,
        cc,
        ccenter
      )) {

    return false;
  }


  if (!solveParachute(
        pa,
        pb,
        pcenter
      )) {

    return false;
  }


  // ----------------------------------------------------------
  // Final deployment crossing.
  // ----------------------------------------------------------

  double finalTime;

  if (!findCrossing(
        ca,
        cb,
        cc,
        ccenter,

        pa,
        pb,
        pcenter,

        finalTime
      )) {

    return false;
  }


  // ----------------------------------------------------------
  // Save final models.
  // ----------------------------------------------------------

  state.coastA =
    ca;

  state.coastB =
    cb;

  state.coastC =
    cc;

  state.coastCenter =
    ccenter;


  state.paraA =
    pa;

  state.paraB =
    pb;

  state.paraCenter =
    pcenter;


  state.deploymentTime =
    finalTime;


  state.deploymentAltitude =
    evalCoast(
      finalTime
    ) / 100.0;


  state.deploymentFinalized =
    true;

  return true;
}


// ============================================================
// COMMAND PARSER
// ============================================================

int parseCommand(
  char *cmd,
  double &altTime
) {

  altTime = 0.0;


  if (strcmp(
        cmd,
        "STATS"
      ) == 0) {

    return 0;
  }


  if (strcmp(
        cmd,
        "G"
      ) == 0) {

    return 1;
  }


  if (strcmp(
        cmd,
        "APOGEE"
      ) == 0) {

    return 2;
  }


  if (strcmp(
        cmd,
        "DEPLOY"
      ) == 0) {

    return 3;
  }


  if (strcmp(
        cmd,
        "DESCENT"
      ) == 0) {

    return 4;
  }


  if (strcmp(
        cmd,
        "LAND"
      ) == 0) {

    return 5;
  }


  if (strcmp(
        cmd,
        "GLITCHES"
      ) == 0) {

    return 6;
  }


  if (strncmp(
        cmd,
        "ALT ",
        4
      ) == 0) {

    char *endPtr;

    float t =
      strtof(
        cmd + 4,
        &endPtr
      );


    if (endPtr ==
        cmd + 4) {

      return -1;
    }


    while (
      *endPtr == ' ' ||
      *endPtr == '\t'
    ) {

      endPtr++;
    }


    if (*endPtr != '\0')
      return -1;


    if (t < 0.0f ||
        t > 100.0f) {

      return -1;
    }


    altTime =
      (double)t;

    return 7;
  }


  return -1;
}


// ============================================================
// COMMAND HANDLER
// ============================================================

void handleCommand(
  char *cmd
) {

  double altTime;

  int type =
    parseCommand(
      cmd,
      altTime
    );


  // Timing starts AFTER parsing.
  uint32_t start =
    ESP.getCycleCount();


  // ----------------------------------------------------------
  // Invalid command
  // ----------------------------------------------------------

  if (type < 0) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    Serial.println("ERR");

    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ----------------------------------------------------------
  // STATS
  // ----------------------------------------------------------

  if (type == 0) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      "bytes used: "
    );

    Serial.print(
      sizeof(GroundState)
    );

    Serial.println(
      " of 4096"
    );


    Serial.print(
      "worst reading: "
    );

    Serial.print(
      state.worstCycles
    );

    Serial.println(
      " cycles (budget 12000)"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ----------------------------------------------------------
  // GLITCHES
  // ----------------------------------------------------------

  if (type == 6) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.println(state.glitchCount);


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ----------------------------------------------------------
  // All analytical commands need final model.
  // ----------------------------------------------------------

  if (!finalizeDeployment()) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.println("ERR");


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // G
  // ==========================================================

  if (type == 1) {

    double gravity =
      -2.0 *
      state.coastC /
      100.0;


    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      gravity,
      3
    );

    Serial.println(
      " m/s^2"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // APOGEE
  // ==========================================================

  if (type == 2) {

    if (state.coastC >= 0.0) {

      uint32_t cycles =
        ESP.getCycleCount() -
        start;


      Serial.println(
        "ERR"
      );


      Serial.print(
        "command cycles: "
      );

      Serial.println(
        cycles
      );

      return;
    }


    double t =
      state.coastCenter -
      state.coastB /
      (2.0 *
       state.coastC);


    double h =
      evalCoast(
        t
      );


    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      t,
      3
    );

    Serial.print(
      " s "
    );

    Serial.print(
      h / 100.0,
      2
    );

    Serial.println(
      " m"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // DEPLOY
  // ==========================================================

  if (type == 3) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      state.deploymentTime,
      3
    );

    Serial.print(
      " s "
    );

    Serial.print(
      state.deploymentAltitude,
      2
    );

    Serial.println(
      " m"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // DESCENT
  // ==========================================================

  if (type == 4) {

    double v =
      state.paraB /
      100.0;


    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      v,
      3
    );

    Serial.println(
      " m/s"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // LAND
  // ==========================================================

  if (type == 5) {

    if (state.paraB >= 0.0) {

      uint32_t cycles =
        ESP.getCycleCount() -
        start;


      Serial.println(
        "ERR"
      );


      Serial.print(
        "command cycles: "
      );

      Serial.println(
        cycles
      );

      return;
    }


    double landTime =
      state.paraCenter -
      state.paraA /
      state.paraB;


    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      landTime,
      3
    );

    Serial.println(
      " s"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // ==========================================================
  // ALT t
  // ==========================================================

  if (type == 7) {

    double h;


    if (
      altTime <=
      state.deploymentTime
    ) {

      h =
        evalCoast(
          altTime
        );

    } else {

      h =
        evalParachute(
          altTime
        );
    }


    uint32_t cycles =
      ESP.getCycleCount() -
      start;


    Serial.print(
      h / 100.0,
      2
    );

    Serial.println(
      " m"
    );


    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }


  // Safety fallback.

  uint32_t cycles =
    ESP.getCycleCount() -
    start;


  Serial.println(
    "ERR"
  );


  Serial.print(
    "command cycles: "
  );

  Serial.println(
    cycles
  );
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );

  delay(500);


  // Clear the ONE persistent state object.

  memset(
    &state,
    0,
    sizeof(state)
  );


  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    " CATCH THE ROCKET - LEVEL 4"
  );

  Serial.println(
    "================================"
  );

  Serial.println(
    "Receiving 100000 readings..."
  );


  // ==========================================================
  // EXACTLY 100000 TRACKER CALLS
  // ==========================================================

  for (
    uint32_t i = 0;

    i < TOTAL_READINGS;

    i++
  ) {

    // Tracker call is deliberately OUTSIDE the measured section.
    float altitude =
      nextAltitude();

    // Reading processing starts here.
    uint32_t start =
      ESP.getCycleCount();

    int64_t rawCm =
      altitudeToCm(altitude);

    uint16_t raw =
      cmTo5cm(rawCm);

    // Causal glitch filtering: no future reading is used.
    bool currentGlitch = false;

    uint16_t clean =
      robustCurrent(raw, currentGlitch);

    int64_t y =
      (int64_t)clean * 5LL;

    // The rolling ring contains CLEAN readings only.
    // Therefore a glitch can never contaminate the 1024-reading
    // deployment detector or either fitted curve.
    uint16_t oldValue = 0;

    if (i >= DETECT_WINDOW) {
      uint32_t oldIndex = i - DETECT_WINDOW;
      oldValue = state.ring[oldIndex % RING_SIZE];
    }

    state.ring[i % RING_SIZE] = clean;

    // ========================================================
    // FIRST 8-SECOND QUADRATIC FIT
    // ========================================================

    if (i < FIRST_COAST) {

      int64_t q =
        2LL * (int64_t)i - 7999LL;

      int64_t q2 = q * q;

      state.firstN++;
      state.firstSumQ2 += q2;
      state.firstSumY += y;
      state.firstSumQY += q * y;
      state.firstSumQ2Y += q2 * y;

      if (i == FIRST_COAST - 1UL) {
        calculateFirstFit();
      }
    }

    // ========================================================
    // STREAMING DEPLOYMENT DETECTION
    // ========================================================

    if (
      !state.deploymentDetected &&
      state.firstReady &&
      i >= DETECT_WINDOW &&
      ((i & 15U) == 0U)
    ) {

      uint32_t oldIndex =
        i - DETECT_WINDOW;

      int64_t actualDifference =
        y - (int64_t)oldValue * 5LL;

      float x1 =
        (float)i / 1000.0f - 3.9995f;

      float x0 =
        (float)oldIndex / 1000.0f - 3.9995f;

      float dx = x1 - x0;

      float predictedDifference =
        (float)state.firstB * dx +
        (float)state.firstC *
        (x1 * x1 - x0 * x0);

      // Before deployment, actual change follows the coast
      // model. After deployment, the parachute makes the
      // actual fall substantially LESS negative than the
      // coast prediction. Therefore: actual - predicted.
      if (
        (float)actualDifference - predictedDifference >
        DETECT_THRESHOLD_CM
      ) {

        state.deploymentDetected = true;
        state.detectionIndex = i;
        state.frozenStart = oldIndex;
      }
    }

    // ========================================================
    // STREAMING FIT ACCUMULATION
    // ========================================================

    if (!state.deploymentDetected) {

      if (i >= DETECT_WINDOW) {

        uint32_t oldIndex =
          i - DETECT_WINDOW;

        // oldValue is already clean.
        addCoast(
          oldIndex,
          (int64_t)oldValue * 5LL
        );
      }

    } else {

      // Ignore the first full detection window after the
      // deployment estimate. This removes the uncertain
      // transition region from the parachute fit.
      if (
        i >=
        state.detectionIndex + DETECT_WINDOW
      ) {

        addParachute(
          i,
          y
        );
      }
    }

    state.readingsReceived++;

    uint32_t cycles =
      ESP.getCycleCount() - start;

    if (cycles > state.worstCycles) {
      state.worstCycles = cycles;
    }
  }

  // ==========================================================
  // RECEIVING FINISHED
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
    "Deployment detected: "
  );

  Serial.println(
    state.deploymentDetected
      ? "YES"
      : "NO"
  );


  Serial.print(
    "Worst reading: "
  );

  Serial.print(
    state.worstCycles
  );

  Serial.println(
    " cycles"
  );


  Serial.println(
    "READY"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  while (
    Serial.available() > 0
  ) {

    char c =
      (char)Serial.read();


    // --------------------------------------------------------
    // End of command.
    // --------------------------------------------------------

    if (c == '\n') {

      if (
        state.commandOverflow
      ) {

        state.commandBuffer[0] =
          '\0';


        handleCommand(
          state.commandBuffer
        );

      } else {

        state.commandBuffer[
          state.commandLength
        ] =
          '\0';


        if (
          state.commandLength > 0
        ) {

          handleCommand(
            state.commandBuffer
          );
        }
      }


      // Reset command parser.

      state.commandLength =
        0;

      state.commandOverflow =
        false;

      state.commandBuffer[0] =
        '\0';


    } else if (
      c != '\r'
    ) {

      // ------------------------------------------------------
      // Normal character.
      // ------------------------------------------------------

      if (
        state.commandLength <
        sizeof(state.commandBuffer) - 1
      ) {

        state.commandBuffer[
          state.commandLength
        ] =
          c;


        state.commandLength++;

      } else {

        // Command too long.

        state.commandOverflow =
          true;
      }
    }
  }
}