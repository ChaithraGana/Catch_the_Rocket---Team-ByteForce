// =====================================================================
// CATCH THE ROCKET - LEVEL 3 (DEPLOY + ALT, tight limits)  -- glitches OFF
// Tracker: PRACTICE TRACKER B
//
// DESIGN
//  * The 100 000 readings are cut into 100 blocks of 1000 readings.
//    For each block we keep only 3 exact integer sums (h in cm, u = j-499.5
//    is the position of the reading inside the block, w = 2u is an integer):
//        s0 = sum h      s1 = sum w*h      s2 = sum w*w*h
//    That is enough to rebuild, for ANY run of whole blocks, the
//    least-squares sums  sum x^k h  (k=0,1,2)  because x = (D+u)/W.
//    The sums of x^k alone (k=0..4) are known in closed form.
//  * After receiving (first command):
//      1. grow a parabola over blocks 0..k-1 until block k sits >1 m off it
//         -> the parachute is open inside block k-1 or k.
//      2. coast parabola P = fit on whole blocks before the deployment,
//         descent line  L = fit on whole blocks after the deployment.
//      3. deployment time = LATER root of P(t)-L(t)=0 (the earlier root is
//         the line extended backwards, before the real opening).
//      4. re-pick the blocks with that time and refit (3 rounds).
//  * All fitting maths is in double, centred + scaled to x in [-1,1].
//    Per reading we only do integer work (3 int adds, 2 int64 mults).
// =====================================================================
#define TRACKER_GLITCHES 0 // 1 = glitches on, 0 = off. Change only this line.

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

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

// ------------------------- state (everything lives here) -------------
#define NB 100        // number of blocks
#define BS 1000       // readings per block

struct Solution {     // results of the fits, cached after first command
  double aP, bP, gP, mP, WP;   // coast:   h = aP + bP x + gP x^2, x=(t_ms-mP)/WP  (h in cm)
  double aL, bL, mL, WL;       // descent: h = aL + bL x,          x=(t_ms-mL)/WL
  double tdms;                 // deployment time in ms
};

struct State {
  int64_t  s1[NB];    // sum of w*h        per block
  int64_t  s2[NB];    // sum of w*w*h      per block
  Solution sol;       // cached fit results
  int32_t  s0[NB];    // sum of h (cm)     per block
  uint32_t worst;     // slowest reading, CPU cycles
  uint16_t blk, j;    // current block / position inside it
  bool     solved, ok;
};
static State gs;

// NOTE: no function below has a Solution/State in its signature
// (Arduino's auto-generated prototypes would appear before the structs).

// ------------------------- receiving ---------------------------------
static inline void ingest(float x) {
  int32_t h = (int32_t)lroundf(x * 100.0f);          // cm
  int32_t w = 2 * (int32_t)gs.j - (BS - 1);          // 2u, integer
  uint16_t b = gs.blk;
  gs.s0[b] += h;
  gs.s1[b] += (int64_t)w * h;
  gs.s2[b] += (int64_t)(w * w) * h;
  if (++gs.j == BS) { gs.j = 0; gs.blk++; }
}

// ------------------------- fitting maths -----------------------------
// Sums over blocks lo..hi of x^k (k=0..4) -> M[k], and of x^k * h (k=0..2) -> R[k]
// with x = (t_ms - m) / W.
static void accum(int lo, int hi, double m, double W, double* M, double* R) {
  const double N = BS;
  const double inv = 1.0 / W;
  const double Q2 = N * (N * N - 1.0) / 12.0 * inv * inv;
  const double Q4 = N * (N * N - 1.0) * (3.0 * N * N - 7.0) / 240.0 * inv * inv * inv * inv;
  for (int k = 0; k < 5; k++) M[k] = 0;
  for (int k = 0; k < 3; k++) R[k] = 0;
  for (int b = lo; b <= hi; b++) {
    double d  = (b * (double)BS + (BS - 1) / 2.0 - m) * inv;   // block centre
    double d2 = d * d;
    M[0] += N;
    M[1] += N * d;
    M[2] += N * d2 + Q2;
    M[3] += N * d2 * d + 3.0 * d * Q2;
    M[4] += N * d2 * d2 + 6.0 * d2 * Q2 + Q4;
    double Sh  = (double)gs.s0[b];
    double Qh  = 0.5  * (double)gs.s1[b] * inv;                // sum q h
    double Q2h = 0.25 * (double)gs.s2[b] * inv * inv;          // sum q^2 h
    R[0] += Sh;
    R[1] += d * Sh + Qh;
    R[2] += d2 * Sh + 2.0 * d * Qh + Q2h;
  }
}

// Gaussian elimination with partial pivoting, n = 2 or 3
static bool solveLin(int n, double A[3][3], double* r, double* x) {
  for (int c = 0; c < n; c++) {
    int p = c;
    for (int i = c + 1; i < n; i++) if (fabs(A[i][c]) > fabs(A[p][c])) p = i;
    if (fabs(A[p][c]) < 1e-12) return false;
    if (p != c) {
      for (int j = 0; j < n; j++) { double t = A[c][j]; A[c][j] = A[p][j]; A[p][j] = t; }
      double t = r[c]; r[c] = r[p]; r[p] = t;
    }
    for (int i = c + 1; i < n; i++) {
      double f = A[i][c] / A[c][c];
      for (int j = c; j < n; j++) A[i][j] -= f * A[c][j];
      r[i] -= f * r[c];
    }
  }
  for (int i = n - 1; i >= 0; i--) {
    double s = r[i];
    for (int j = i + 1; j < n; j++) s -= A[i][j] * x[j];
    x[i] = s / A[i][i];
  }
  return true;
}

// least-squares polynomial (deg 1 or 2) over whole blocks lo..hi
static bool fitPoly(int lo, int hi, int deg, double* c, double& m, double& W) {
  m = (1000.0 * lo + 1000.0 * hi + (BS - 1)) / 2.0;
  W = 500.0 * (hi - lo + 1);
  double M[5], R[3];
  accum(lo, hi, m, W, M, R);
  double A[3][3], r[3];
  for (int i = 0; i <= deg; i++) {
    r[i] = R[i];
    for (int j = 0; j <= deg; j++) A[i][j] = M[i + j];
  }
  return solveLin(deg + 1, A, r, c);
}

// later root of P - L = 0, in ms (uses gs.sol)
static bool crossing(double& tdms) {
  const Solution& s = gs.sol;
  double A = s.gP / (s.WP * s.WP);
  double B = s.bP / s.WP - s.bL / s.WL;
  double C = s.aP - s.aL - s.bL * (s.mP - s.mL) / s.WL;
  if (fabs(A) < 1e-18) return false;
  double disc = B * B - 4.0 * A * C;
  if (disc < 0) return false;
  double sq = sqrt(disc);
  double r1 = (-B + sq) / (2.0 * A), r2 = (-B - sq) / (2.0 * A);
  tdms = s.mP + (r1 > r2 ? r1 : r2);
  return true;
}

// fills gs.sol
static bool solveAll() {
  Solution& s = gs.sol;
  // 1. grow the coasting parabola until the next block disagrees by > 1 m
  int k;
  for (k = 8; k < NB; k++) {
    double c[3], m, W, M[5], R[3];
    if (!fitPoly(0, k - 1, 2, c, m, W)) return false;
    accum(k, k, m, W, M, R);
    double res = R[0] - (c[0] * M[0] + c[1] * M[1] + c[2] * M[2]);   // cm * readings
    if (fabs(res) > 100.0 * BS) break;
  }
  if (k >= NB) return false;
  int cLast = (k - 2 > 7) ? k - 2 : 7;
  int lFirst = k + 1;

  // 2-4. fit, intersect, re-pick blocks, repeat
  for (int it = 0; it < 3; it++) {
    if (lFirst > NB - 3) return false;
    double cp[3], cl[2];
    if (!fitPoly(0, cLast, 2, cp, s.mP, s.WP)) return false;
    if (!fitPoly(lFirst, NB - 1, 1, cl, s.mL, s.WL)) return false;
    s.aP = cp[0]; s.bP = cp[1]; s.gP = cp[2];
    s.aL = cl[0]; s.bL = cl[1];
    double td;
    if (!crossing(td)) return false;
    s.tdms = td;
    if (td < 0 || td > 100000.0) return false;
    cLast = (int)floor((td - 30.0) / 1000.0) - 1;
    if (cLast < 7) cLast = 7;
    lFirst = (int)ceil((td + 30.0) / 1000.0);
  }
  return true;
}

static void ensureSolved() {
  if (!gs.solved) { gs.ok = solveAll(); gs.solved = true; }
}

static double evalP(double tms) {   // coast parabola, cm
  const Solution& s = gs.sol;
  double x = (tms - s.mP) / s.WP;
  return s.aP + s.bP * x + s.gP * x * x;
}
static double evalL(double tms) {   // descent line, cm
  const Solution& s = gs.sol;
  return s.aL + s.bL * (tms - s.mL) / s.WL;
}

// ------------------------- commands ----------------------------------
static void handle(char* line) {
  char* p = line;
  while (*p == ' ' || *p == '\t') p++;
  char* cmd = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  char* arg = NULL;
  if (*p) {
    *p++ = 0;
    while (*p == ' ' || *p == '\t') p++;
    if (*p) {
      arg = p;
      char* e = arg + strlen(arg);
      while (e > arg && (e[-1] == ' ' || e[-1] == '\t')) *--e = 0;
    }
  }
  if (!*cmd) return;

  bool isAlt = !strcmp(cmd, "ALT");
  if (isAlt ? (arg == NULL) : (arg != NULL)) { Serial.println("ERR"); return; }

  uint32_t c0, cyc;

  if (!strcmp(cmd, "STATS")) {
    Serial.printf("bytes used: %u of 4096\n", (unsigned)sizeof(State));
    Serial.printf("worst reading: %lu cycles (budget 12000)\n", (unsigned long)gs.worst);
    return;
  }
  if (!strcmp(cmd, "GLITCHES")) {
    Serial.println("TODO (Level 4) (0 cycles)");
    return;
  }

  if (!strcmp(cmd, "G")) {
    c0 = ESP.getCycleCount();
    ensureSolved();
    double g = -2.0 * gs.sol.gP / (gs.sol.WP * gs.sol.WP) * 1e4;   // cm/ms^2 -> m/s^2
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.3f m/s^2 (%lu cycles)\n", g, (unsigned long)cyc);
  } else if (!strcmp(cmd, "APOGEE")) {
    c0 = ESP.getCycleCount();
    ensureSolved();
    const Solution& s = gs.sol;
    double xs = -s.bP / (2.0 * s.gP);
    double t = (s.mP + xs * s.WP) / 1000.0;
    double h = (s.aP - s.bP * s.bP / (4.0 * s.gP)) / 100.0;
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.3f s %.2f m (%lu cycles)\n", t, h, (unsigned long)cyc);
  } else if (!strcmp(cmd, "DEPLOY")) {
    c0 = ESP.getCycleCount();
    ensureSolved();
    double t = gs.sol.tdms / 1000.0;
    double h = evalP(gs.sol.tdms) / 100.0;
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.3f s %.2f m (%lu cycles)\n", t, h, (unsigned long)cyc);
  } else if (!strcmp(cmd, "DESCENT")) {
    c0 = ESP.getCycleCount();
    ensureSolved();
    double v = gs.sol.bL / gs.sol.WL * 10.0;                         // cm/ms -> m/s
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.3f m/s (%lu cycles)\n", v, (unsigned long)cyc);
  } else if (!strcmp(cmd, "LAND")) {
    c0 = ESP.getCycleCount();
    ensureSolved();
    double x = -gs.sol.aL / gs.sol.bL;
    double t = (gs.sol.mL + x * gs.sol.WL) / 1000.0;
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.3f s (%lu cycles)\n", t, (unsigned long)cyc);
  } else if (isAlt) {
    char* e;
    double t = strtod(arg, &e);
    if (e == arg || *e || !(t >= 0.0 && t <= 100.0)) { Serial.println("ERR"); return; }
    c0 = ESP.getCycleCount();
    ensureSolved();
    double tms = t * 1000.0;
    double h = ((tms < gs.sol.tdms) ? evalP(tms) : evalL(tms)) / 100.0;
    cyc = ESP.getCycleCount() - c0;
    if (!gs.ok) { Serial.println("ERR"); return; }
    Serial.printf("%.2f m (%lu cycles)\n", h, (unsigned long)cyc);
  } else {
    Serial.println("ERR");
  }
}

// ------------------------- Arduino entry points ----------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  memset(&gs, 0, sizeof(gs));
  for (uint32_t i = 0; i < 100000; i++) {
    float x = nextAltitude();
    uint32_t c0 = ESP.getCycleCount();
    ingest(x);
    uint32_t dt = ESP.getCycleCount() - c0;
    if (dt > gs.worst) gs.worst = dt;
  }
  Serial.println("Received 100000 readings. Ready.");
}

void loop() {
  char buf[64];
  int n = 0;
  bool overflow = false;
  for (;;) {                                   // read one line (local buffer)
    if (!Serial.available()) { delay(1); continue; }
    int c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n == 0 && !overflow) continue;
      break;
    }
    if (n < (int)sizeof(buf) - 1) buf[n++] = (char)toupper(c); else overflow = true;
  }
  buf[n] = 0;
  if (overflow) { Serial.println("ERR"); return; }
  handle(buf);
}