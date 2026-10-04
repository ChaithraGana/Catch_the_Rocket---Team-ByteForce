#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define TRACKER_GLITCHES 0

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

  int64_t h =
      H0
    + V0 * tc / 1000
    - G * tc * tc / 2000000;

  if (t > TD)
    h += VD * (t - TD) / 1000;

  uint32_t r = trackerRand();

  if (quiet > 0)
    quiet--;

  if (TRACKER_GLITCHES &&
      t >= 1000 &&
      quiet == 0 &&
      (r >> 24) < 2) {

    quiet = 10;

    int64_t off =
      5000 + (int64_t)((r >> 4) % 25001);

    h += ((r >> 3) & 1)
       ? off
       : -off;

    if (h < 0)
      h += 2 * off;

  } else {

    h +=
      (int64_t)((r >> 8) % 401) - 200;
  }

  t++;

  return (float)h / 100.0f;
}

// ============================================================
// COMMAND CONSTANTS
// ============================================================

enum CommandType {
  CMD_STATS,
  CMD_G,
  CMD_APOGEE,
  CMD_DEPLOY,
  CMD_DESCENT,
  CMD_LAND,
  CMD_ALT,
  CMD_GLITCHES,
  CMD_INVALID
};

// ============================================================
// CONSTANTS
// ============================================================

const uint32_t TOTAL_READINGS = 100000;
const uint32_t FIRST_COAST = 8000;
const uint32_t WINDOW = 1024;

// ============================================================
// ONE PERSISTENT STATE STRUCT
// ============================================================

struct GroundState {

  uint32_t readingsReceived;
  uint32_t worstCycles;

  // First 8-second quadratic fit
  uint32_t firstN;

  int64_t firstSumQ2;
  int64_t firstSumY;
  int64_t firstSumQY;
  int64_t firstSumQ2Y;

  double firstSumQ4;

  double firstA;
  double firstB;
  double firstC;

  bool firstReady;

  // Full coast quadratic
  uint32_t coastN;

  int64_t coastSumX;
  int64_t coastSumX2;
  int64_t coastSumX3;

  int64_t coastSumY;
  int64_t coastSumXY;
  int64_t coastSumX2Y;

  double coastSumX4;

  // Parachute line
  uint32_t paraN;

  int64_t paraSumX;
  int64_t paraSumX2;
  int64_t paraSumY;
  int64_t paraSumXY;

  // Descent
  uint32_t descentN;

  int64_t descentSumX;
  int64_t descentSumX2;
  int64_t descentSumY;
  int64_t descentSumXY;

  // Detection ring
  uint16_t ring[WINDOW];

  int64_t ringSum;

  // Deployment
  bool deploymentDetected;
  bool deploymentFinalized;

  uint32_t detectionIndex;
  uint32_t frozenStart;
  uint16_t evictedValue;

  double deploymentTime;
  double deploymentAltitude;

  // Serial command state
  char commandBuffer[32];
  uint8_t commandLength;
  bool commandOverflow;
};

GroundState state;

static_assert(
  sizeof(GroundState) <= 4096,
  "GroundState exceeds 4096 bytes"
);

// ============================================================
// CONVERSIONS
// ============================================================

int64_t altitudeCm(float h) {
  return (int64_t)lroundf(h * 100.0f);
}

uint16_t altitude5cm(float h) {

  long v =
    lroundf(h * 20.0f);

  if (v < 0)
    v = 0;

  if (v > 60000)
    v = 60000;

  return (uint16_t)v;
}

// ============================================================
// DESCENT DATA
// ============================================================

void addDescent(
  uint32_t i,
  int64_t y
) {

  int64_t x =
    (int64_t)i - 72500LL;

  state.descentN++;

  state.descentSumX += x;
  state.descentSumX2 += x * x;
  state.descentSumY += y;
  state.descentSumXY += x * y;
}

// ============================================================
// FIRST 8 SECOND FIT
// q = 2i - 7999
// ============================================================

void calculateFirstFit() {

  double N =
    (double)state.firstN;

  double S2 =
    (double)state.firstSumQ2;

  double S4 =
    state.firstSumQ4;

  double SY =
    (double)state.firstSumY;

  double SQY =
    (double)state.firstSumQY;

  double SQ2Y =
    (double)state.firstSumQ2Y;

  double denominator =
    N * S4 - S2 * S2;

  if (fabs(denominator) < 1e-12) {
    state.firstReady = false;
    return;
  }

  state.firstB =
    SQY / S2;

  state.firstC =
    (N * SQ2Y - S2 * SY) /
    denominator;

  state.firstA =
    (SY - state.firstC * S2) /
    N;

  state.firstReady = true;
}

// ============================================================
// CONVERT q MODEL TO x = t - 20
//
// q = 2000x + 32001
// ============================================================

void firstModel(
  double &a,
  double &b,
  double &c
) {

  const double K = 32001.0;

  a =
    state.firstA
    + state.firstB * K
    + state.firstC * K * K;

  b =
    state.firstB * 2000.0
    + state.firstC * 4000.0 * K;

  c =
    state.firstC * 4000000.0;
}

// ============================================================
// FULL COAST DATA
// ============================================================

void addCoast(
  uint32_t i,
  int64_t y
) {

  int64_t x =
    (int64_t)i - 20000LL;

  int64_t x2 =
    x * x;

  int64_t x3 =
    x2 * x;

  state.coastN++;

  state.coastSumX += x;
  state.coastSumX2 += x2;
  state.coastSumX3 += x3;

  state.coastSumY += y;
  state.coastSumXY += x * y;
  state.coastSumX2Y += x2 * y;

  double xd =
    (double)x;

  state.coastSumX4 +=
    xd * xd * xd * xd;
}

// ============================================================
// SOLVE FULL COAST
// ============================================================

bool solveCoast(
  double &a,
  double &b,
  double &c
) {

  if (state.coastN < 3)
    return false;

  double N =
    (double)state.coastN;

  double S0 = N;

  double S1 =
    (double)state.coastSumX /
    1000.0;

  double S2 =
    (double)state.coastSumX2 /
    1000000.0;

  double S3 =
    (double)state.coastSumX3 /
    1000000000.0;

  double S4 =
    state.coastSumX4 /
    1000000000000.0;

  double Y0 =
    (double)state.coastSumY;

  double Y1 =
    (double)state.coastSumXY /
    1000.0;

  double Y2 =
    (double)state.coastSumX2Y /
    1000000.0;

  double m[3][4] = {

    {S0, S1, S2, Y0},
    {S1, S2, S3, Y1},
    {S2, S3, S4, Y2}
  };

  // Gaussian elimination
  for (int col = 0; col < 3; col++) {

    int pivot = col;

    double largest =
      fabs(m[col][col]);

    for (int row = col + 1;
         row < 3;
         row++) {

      double value =
        fabs(m[row][col]);

      if (value > largest) {
        largest = value;
        pivot = row;
      }
    }

    if (largest < 1e-12)
      return false;

    if (pivot != col) {

      for (int j = col;
           j < 4;
           j++) {

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

    for (int j = col;
         j < 4;
         j++) {

      m[col][j] /=
        divisor;
    }

    for (int row = 0;
         row < 3;
         row++) {

      if (row == col)
        continue;

      double factor =
        m[row][col];

      for (int j = col;
           j < 4;
           j++) {

        m[row][j] -=
          factor * m[col][j];
      }
    }
  }

  a = m[0][3];
  b = m[1][3];
  c = m[2][3];

  return true;
}

// ============================================================
// PARACHUTE LINE
// ============================================================

void addParachute(
  uint32_t i,
  int64_t y
) {

  int64_t x =
    (int64_t)i - 20000LL;

  state.paraN++;

  state.paraSumX += x;
  state.paraSumX2 += x * x;
  state.paraSumY += y;
  state.paraSumXY += x * y;
}

bool solveParachute(
  double &d,
  double &e
) {

  if (state.paraN < 2)
    return false;

  double N =
    (double)state.paraN;

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
    N * Sx2 - Sx * Sx;

  if (fabs(denominator) < 1e-12)
    return false;

  e =
    (N * Sxy - Sx * Sy) /
    denominator;

  d =
    (Sy - e * Sx) /
    N;

  return true;
}

// ============================================================
// DEPLOYMENT CROSSING
// ============================================================

bool findDeployment(
  double ca,
  double cb,
  double cc,
  double pd,
  double pe,
  double &t
) {

  double qa = cc;
  double qb = cb - pe;
  double qc = ca - pd;

  if (fabs(qa) < 1e-15)
    return false;

  double discriminant =
    qb * qb -
    4.0 * qa * qc;

  if (discriminant < 0.0)
    return false;

  double root =
    sqrt(discriminant);

  double x1 =
    (-qb + root) /
    (2.0 * qa);

  double x2 =
    (-qb - root) /
    (2.0 * qa);

  double t1 =
    20.0 + x1;

  double t2 =
    20.0 + x2;

  bool valid1 =
    (t1 >= 8.0 &&
     t1 <= 45.0);

  bool valid2 =
    (t2 >= 8.0 &&
     t2 <= 45.0);

  if (!valid1 && !valid2)
    return false;

  if (valid1 && valid2)
    t = (t1 > t2) ? t1 : t2;
  else if (valid1)
    t = t1;
  else
    t = t2;

  return true;
}

// ============================================================
// RING ACCESS
// ============================================================

uint16_t getFrozenValue(
  uint32_t i
) {

  if (i <
      state.frozenStart)
    return state.evictedValue;

  return state.ring[
    i % WINDOW
  ];
}

// ============================================================
// FINALIZE DEPLOYMENT
// ============================================================

bool finalizeDeployment() {

  if (!state.deploymentDetected)
    return false;

  if (state.deploymentFinalized)
    return true;

  double paraD;
  double paraE;

  if (!solveParachute(
        paraD,
        paraE))
    return false;

  double firstA;
  double firstB;
  double firstC;

  firstModel(
    firstA,
    firstB,
    firstC
  );

  double preliminaryTime;

  if (!findDeployment(
        firstA,
        firstB,
        firstC,
        paraD,
        paraE,
        preliminaryTime))
    return false;

  uint32_t boundary =
    (uint32_t)lround(
      preliminaryTime * 1000.0
    );

  if (boundary <
      state.frozenStart)
    boundary =
      state.frozenStart;

  if (boundary >
      state.detectionIndex)
    boundary =
      state.detectionIndex;

  // Recover the readings that were still
  // inside the detection ring.

  for (
    uint32_t i = state.frozenStart;
    i <= boundary;
    i++
  ) {

    uint16_t u =
      getFrozenValue(i);

    int64_t y =
      (int64_t)u * 5LL;

    addCoast(
      i,
      y
    );
  }

  // Recover parachute readings.

  for (
    uint32_t i = boundary + 1;
    i < state.detectionIndex;
    i++
  ) {

    uint16_t u =
      getFrozenValue(i);

    int64_t y =
      (int64_t)u * 5LL;

    addParachute(
      i,
      y
    );
  }

  double coastA;
  double coastB;
  double coastC;

  if (!solveCoast(
        coastA,
        coastB,
        coastC))
    return false;

  if (!solveParachute(
        paraD,
        paraE))
    return false;

  double finalTime;

  if (!findDeployment(
        coastA,
        coastB,
        coastC,
        paraD,
        paraE,
        finalTime))
    return false;

  double x =
    finalTime - 20.0;

  double altitude =
    coastA
    + coastB * x
    + coastC * x * x;

  state.deploymentTime =
    finalTime;

  state.deploymentAltitude =
    altitude / 100.0;

  state.deploymentFinalized =
    true;

  return true;
}

// ============================================================
// COMMAND PARSER
// IMPORTANT: return type is INT to avoid Arduino
// automatic-prototype problem.
// ============================================================

int parseCommand(
  char *cmd,
  double &altTime
) {

  altTime = 0.0;

  if (strcmp(cmd, "STATS") == 0)
    return CMD_STATS;

  if (strcmp(cmd, "G") == 0)
    return CMD_G;

  if (strcmp(cmd, "APOGEE") == 0)
    return CMD_APOGEE;

  if (strcmp(cmd, "DEPLOY") == 0)
    return CMD_DEPLOY;

  if (strcmp(cmd, "DESCENT") == 0)
    return CMD_DESCENT;

  if (strcmp(cmd, "LAND") == 0)
    return CMD_LAND;

  if (strcmp(cmd, "GLITCHES") == 0)
    return CMD_GLITCHES;

  if (strncmp(cmd, "ALT ", 4) == 0) {

    char *endPtr;

    float t =
      strtof(
        cmd + 4,
        &endPtr
      );

    if (endPtr == cmd + 4)
      return CMD_INVALID;

    while (*endPtr == ' ' ||
           *endPtr == '\t')
      endPtr++;

    if (*endPtr != '\0')
      return CMD_INVALID;

    if (t < 0.0f ||
        t > 100.0f)
      return CMD_INVALID;

    altTime =
      (double)t;

    return CMD_ALT;
  }

  return CMD_INVALID;
}

// ============================================================
// COMMAND HANDLER
// ============================================================

void handleCommand(
  char *cmd
) {

  double altTime;

  uint32_t start =
    ESP.getCycleCount();

  int type =
    parseCommand(
      cmd,
      altTime
    );

  // ----------------------------------------------------------
  // STATS
  // ----------------------------------------------------------

  if (type == CMD_STATS) {

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
      " cycles"
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
  // INVALID
  // ----------------------------------------------------------

  if (type == CMD_INVALID) {

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
  // GLITCHES
  // ----------------------------------------------------------

  if (type == CMD_GLITCHES) {

    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    Serial.println("0");

    Serial.print(
      "command cycles: "
    );

    Serial.println(
      cycles
    );

    return;
  }

  // ----------------------------------------------------------
  // DEPLOYMENT REQUIRED
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

  // ----------------------------------------------------------
  // FULL COAST
  // ----------------------------------------------------------

  double coastA;
  double coastB;
  double coastC;

  if (!solveCoast(
        coastA,
        coastB,
        coastC)) {

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
  // G
  // ----------------------------------------------------------

  if (type == CMD_G) {

    double gravity =
      -2.0 * coastC / 100.0;

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

  // ----------------------------------------------------------
  // APOGEE
  // ----------------------------------------------------------

  if (type == CMD_APOGEE) {

    if (coastC >= 0.0) {

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

    double x =
      -coastB /
      (2.0 * coastC);

    double t =
      20.0 + x;

    double h =
      coastA -
      (coastB * coastB) /
      (4.0 * coastC);

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

  // ----------------------------------------------------------
  // DEPLOY
  // ----------------------------------------------------------

  if (type == CMD_DEPLOY) {

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

  // ----------------------------------------------------------
  // DESCENT
  // ----------------------------------------------------------

  if (type == CMD_DESCENT) {

    double paraD;
    double paraE;

    if (!solveParachute(
          paraD,
          paraE)) {

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

    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    Serial.print(
      paraE / 100.0,
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

  // ----------------------------------------------------------
  // LAND
  // ----------------------------------------------------------

  if (type == CMD_LAND) {

    double paraD;
    double paraE;

    if (!solveParachute(
          paraD,
          paraE) ||
        paraE >= 0.0) {

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

    double x =
      -paraD / paraE;

    double t =
      20.0 + x;

    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    Serial.print(
      t,
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

  // ----------------------------------------------------------
  // ALT t
  // ----------------------------------------------------------

  if (type == CMD_ALT) {

    double x =
      altTime - 20.0;

    double altitudeCm;

    if (
      altTime <=
      state.deploymentTime
    ) {

      altitudeCm =
        coastA
        + coastB * x
        + coastC * x * x;

    } else {

      double paraD;
      double paraE;

      if (!solveParachute(
            paraD,
            paraE)) {

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

      altitudeCm =
        paraD +
        paraE * x;
    }

    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    Serial.print(
      altitudeCm / 100.0,
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

  // ----------------------------------------------------------
  // FALLBACK
  // ----------------------------------------------------------

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
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(500);

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
    " CATCH THE ROCKET - LEVEL 3"
  );
  Serial.println(
    "================================"
  );
  Serial.println(
    "Receiving 100000 readings..."
  );

  // EXACTLY 100000 calls
  for (
    uint32_t i = 0;
    i < TOTAL_READINGS;
    i++
  ) {

    float altitude =
      nextAltitude();

    uint32_t start =
      ESP.getCycleCount();

    int64_t y =
      altitudeCm(
        altitude
      );

    uint16_t current =
      altitude5cm(
        altitude
      );

    // --------------------------------------------------------
    // ROLLING WINDOW
    // --------------------------------------------------------

    uint16_t oldValue = 0;

    bool full =
      (i >= WINDOW);

    if (full) {

      oldValue =
        state.ring[
          i % WINDOW
        ];

      state.ringSum -=
        oldValue;
    }

    state.ring[
      i % WINDOW
    ] =
      current;

    state.ringSum +=
      current;

    // --------------------------------------------------------
    // FIRST 8 SECONDS
    // --------------------------------------------------------

    if (i < FIRST_COAST) {

      int64_t q =
        2LL * (int64_t)i -
        7999LL;

      int64_t q2 =
        q * q;

      state.firstN++;

      state.firstSumQ2 +=
        q2;

      state.firstSumY +=
        y;

      state.firstSumQY +=
        q * y;

      state.firstSumQ2Y +=
        q2 * y;

      double qd =
        (double)q;

      state.firstSumQ4 +=
        qd * qd * qd * qd;

      if (i == 7999)
        calculateFirstFit();
    }

    // --------------------------------------------------------
    // DESCENT DATA
    // --------------------------------------------------------

    if (i >= 45000) {

      addDescent(
        i,
        y
      );
    }

    // --------------------------------------------------------
    // DEPLOYMENT DETECTION
    // --------------------------------------------------------

    if (!state.deploymentDetected &&
        state.firstReady &&
        i >= WINDOW) {

      uint32_t firstIndex =
        i - WINDOW + 1;

      int64_t q0 =
        2LL * (int64_t)firstIndex -
        7999LL;

      int64_t n =
        WINDOW;

      int64_t qLast =
        q0 + 2LL * (n - 1);

      double qSum =
        (double)n *
        ((double)q0 +
         (double)qLast) /
        2.0;

      double nn =
        (double)n;

      double q0d =
        (double)q0;

      double nm1 =
        (double)(n - 1);

      double q2Sum =
        nn * q0d * q0d
        +
        q0d * 2.0 * nn * nm1
        +
        4.0 * nn * nm1 *
        (double)(2 * n - 1) /
        6.0;

      double predicted =
        state.firstA * nn
        + state.firstB * qSum
        + state.firstC * q2Sum;

      double actual =
        (double)
        state.ringSum *
        5.0;

      double residual =
        actual - predicted;

      if (residual >
          270000.0) {

        state.deploymentDetected =
          true;

        state.detectionIndex =
          i;

        state.frozenStart =
          i - WINDOW;

        state.evictedValue =
          oldValue;

        // Current reading is considered
        // post-deployment.
        addParachute(
          i,
          y
        );

      } else {

        if (full) {

          int64_t oldY =
            (int64_t)oldValue *
            5LL;

          addCoast(
            i - WINDOW,
            oldY
          );
        }
      }

    } else if (
      state.deploymentDetected
    ) {

      if (
        i >
        state.detectionIndex
      ) {

        addParachute(
          i,
          y
        );
      }
    }

    state.readingsReceived++;

    // End of per-reading work
    uint32_t cycles =
      ESP.getCycleCount() -
      start;

    if (
      cycles >
      state.worstCycles
    ) {

      state.worstCycles =
        cycles;
    }
  }

  // ----------------------------------------------------------
  // FINISHED
  // ----------------------------------------------------------

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

    if (c == '\n') {

      if (state.commandOverflow) {

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

      state.commandLength =
        0;

      state.commandOverflow =
        false;

      state.commandBuffer[0] =
        '\0';
    }

    else if (c != '\r') {

      if (
        state.commandLength <
        sizeof(state.commandBuffer) - 1
      ) {

        state.commandBuffer[
          state.commandLength
        ] = c;

        state.commandLength++;

      } else {

        state.commandOverflow =
          true;
      }
    }
  }
}