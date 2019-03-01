/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 *
 */

#include "estimator_kalman.h"
//#include "outlierFilter.h"
#include <math.h>
#include "cf_math.h"
#include "mat_math.h"
#include "contiki.h"
#include <string.h>

#ifndef __errno
#define __errno 1
#endif
#define configASSERT(x)

//#define KALMAN_USE_BARO_UPDATE
//#define KALMAN_NAN_CHECK


/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
static void stateEstimatorPredict(float dt);
static void stateEstimatorAddProcessNoise(float dt);

/*  - Measurement updates based on sensors */
static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);

/*  - Finalization to incorporate attitude error into body attitude */
static void stateEstimatorFinalize();

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
static void stateEstimatorExternalizeState(point_t *position, uint32_t tick);


/**
 * Additionally, the filter supports the incorporation of additional sensors into the state estimate
 *
 * This is done via the external functions:
 * - bool estimatorKalmanEnqueueUWBPacket(uwbPacket_t *uwb)
 * - bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
 * - bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
 *
 * As well as by the following internal functions and datatypes
 */

// Measurements of a UWB Tx/Rx
static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *uwb);

/**
 * Constants used in the estimator
 */
#define GRAVITY_MAGNITUDE (9.81f) // we use the magnitude such that the sign/direction is explicit in calculations

/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// The bounds on states, these shouldn't be hit...
#define MAX_POSITION (100) //meters

// Initial variances, uncertain of position, but know we're stationary and roughly flat
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;

static float procNoisePos = 0;

static float initialX = 0.5;
static float initialY = 0.5;
static float initialZ = 0.0;

// We track a TDOA skew as part of the Kalman filter
static const float stdDevInitialSkew = 0.1;
static float procNoiseSkew = 10e-6f; // seconds per second^2 (is multiplied by dt to give skew noise)

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 * - SKEW: the skew from anchor system clock to quad clock
 *
 * For more information, refer to the paper
 */

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_DIM
} stateIdx_t;

static float S[STATE_DIM];

// The covariance matrix
static float P[STATE_DIM][STATE_DIM];
static arm_matrix_instance_f32 Pm = {STATE_DIM, STATE_DIM, (float *)P};


/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;
static bool resetEstimation = true;
static int32_t lastPrediction;
static int32_t lastPNUpdate;
static uint32_t tdoaCount;

/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
//static inline float arm_sqrt(float32_t in)
//{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

#ifdef KALMAN_NAN_CHECK
static void stateEstimatorAssertNotNaN() {
  if ((isnan(S[STATE_X])) ||
      (isnan(S[STATE_Y])) ||
      (isnan(S[STATE_Z]))) { resetEstimation = true; }

  for(int i=0; i<STATE_DIM; i++) {
    for(int j=0; j<STATE_DIM; j++)
    {
      if (isnan(P[i][j]))
      {
        resetEstimation = true;
      }
    }
  }
}
#else
#endif

#ifdef KALMAN_DECOUPLE_XY
// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(stateIdx_t state)
{
  // Set all covariance to 0
  for(int i=0; i<STATE_DIM; i++) {
    P[state][i] = 0;
    P[i][state] = 0;
  }
  // Set state variance to maximum
  P[state][state] = MAX_COVARIANCE;
  // set state to zero
  S[state] = 0;
}
#endif

// --------------------------------------------------

void estimatorKalman(point_t *position, const uint32_t tick, const tdoaMeasurement_t *tdoa)
{
  // If the client (via a parameter update) triggers an estimator reset:
  if (resetEstimation) { estimatorKalmanInit(); resetEstimation = false; }

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool doneUpdate = false;

  uint32_t osTick = clock_time(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
  // Decouple position states
  decoupleState(STATE_X)
  decoupleState(STATE_Y);
#endif

  //float dt = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;
  float dt = (float)(osTick-lastPrediction)/RATE_1000_HZ;
  stateEstimatorPredict(dt);

  lastPrediction = osTick;
  doneUpdate = true;

  /**
   * Add process noise every loop, rather than every prediction
   */
  //stateEstimatorAddProcessNoise((float)(osTick-lastPNUpdate)/configTICK_RATE_HZ);
  stateEstimatorAddProcessNoise((float)(osTick-lastPNUpdate)/RATE_1000_HZ);
  lastPNUpdate = osTick;

  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */
/*   tdoaMeasurement_t tdoa;
  while (stateEstimatorHasTDOAPacket(&tdoa))
  {
    stateEstimatorUpdateWithTDOA(&tdoa);
    doneUpdate = true;
  } */
  stateEstimatorUpdateWithTDOA(tdoa);
  doneUpdate = true;

  /**
   * If an update has been made, the state is finalized:
   * - the attitude error is moved into the body attitude quaternion,
   * - the body attitude is converted into a rotation matrix for the next prediction, and
   * - correctness of the covariance matrix is ensured
   */

  if (doneUpdate)
  {
    stateEstimatorFinalize();
  }

  /**
   * Finally, the internal state is externalized.
   * This is done every round, since the external state includes some sensor data
   */
  stateEstimatorExternalizeState(position, osTick);
}

static void stateEstimatorPredict(float dt)
{
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d, skew are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // The linearized update matrix
  static float A[STATE_DIM][STATE_DIM];
  static arm_matrix_instance_f32 Am = { STATE_DIM, STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM, STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[STATE_X][STATE_X] = 1;
  A[STATE_Y][STATE_Y] = 1;
  A[STATE_Z][STATE_Z] = 1;

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======
  // The prediction depends on whether we're on the ground, or in flight.
  // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

  // TODO: Find a better check for whether the quad is flying
  // Assume that the flight begins when the thrust is large enough and for now we never stop "flying"
}

static void stateEstimatorAddProcessNoise(float dt)
{
  if (dt>0)
  {
    P[STATE_X][STATE_X] += powf(procNoisePos, 2);  // add process noise on position
    P[STATE_Y][STATE_Y] += powf(procNoisePos, 2);  // add process noise on position
    P[STATE_Z][STATE_Z] += powf(procNoisePos, 2);  // add process noise on position
  }

  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }
}

static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // The Kalman gain as a column vector
  static float K[STATE_DIM];
  static arm_matrix_instance_f32 Km = {STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

  static float tmpNN3d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

  static float HTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

  static float PHTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &HTm);
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  configASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    S[i] = S[i] + K[i] * error; // state update
  }

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<STATE_DIM; i++) { tmpNN1d[STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*P[i][j] + 0.5f*P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }
}

static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *tdoa)
{
  if (tdoaCount >= 100)
  {
    /**
     * Measurement equation:
     * dR = dT + d1 - d0
     */

    float measurement = tdoa->distanceDiff;

    // predict based on current state
    float x = S[STATE_X];
    float y = S[STATE_Y];
    float z = S[STATE_Z];

    float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
    float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

    float dx1 = x - x1;
    float dy1 = y - y1;
    float dz1 = z - z1;

    float dx0 = x - x0;
    float dy0 = y - y0;
    float dz0 = z - z0;

    //float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
    //float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));
float d1 = 1;
float d0 = 1;

    float predicted = d1 - d0;
    float error = measurement - predicted;

    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

    if ((d0 != 0.0f) && (d1 != 0.0f)) {
      h[STATE_X] = (dx1 / d1 - dx0 / d0);
      h[STATE_Y] = (dy1 / d1 - dy0 / d0);
      h[STATE_Z] = (dz1 / d1 - dz0 / d0);

      vector_t jacobian = {
        .x = h[STATE_X],
        .y = h[STATE_Y],
        .z = h[STATE_Z],
      };

      point_t estimatedPosition = {
        .x = S[STATE_X],
        .y = S[STATE_Y],
        .z = S[STATE_Z],
      };

      stateEstimatorScalarUpdate(&H, error, tdoa->stdDev);
      //bool sampleIsGood = outlierFilterVaildateTdoaSteps(tdoa, error, &jacobian, &estimatedPosition);
      //if (sampleIsGood) {
      //  stateEstimatorScalarUpdate(&H, error, tdoa->stdDev);
      //}
    }
  }

  tdoaCount++;
}

static void stateEstimatorFinalize()
{
  // constrain the states
  for (int i=0; i<3; i++)
  {
    if (S[STATE_X+i] < -MAX_POSITION) { S[STATE_X+i] = -MAX_POSITION; }
    else if (S[STATE_X+i] > MAX_POSITION) { S[STATE_X+i] = MAX_POSITION; }
  }

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }
}

static void stateEstimatorExternalizeState(point_t *position, uint32_t tick)
{
  // position state is already in world frame
  position->timestamp = tick;
  position->x = S[STATE_X];
  position->y = S[STATE_Y];
  position->z = S[STATE_Z];
}

void estimatorKalmanInit(void) {
  lastPrediction = clock_time();
  lastPNUpdate = clock_time();

  // Reset all matrices to 0 (like uppon system reset)
  memset(P, 0, sizeof(S));

  // TODO: Can we initialize this more intelligently?
  S[STATE_X] = initialX;
  S[STATE_Y] = initialY;
  S[STATE_Z] = initialZ;

  for (int i=0; i< STATE_DIM; i++) {
    for (int j=0; j < STATE_DIM; j++) {
      P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  P[STATE_X][STATE_X]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Y][STATE_Y]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Z][STATE_Z]  = powf(stdDevInitialPosition_z, 2);

  tdoaCount = 0;
  isInit = true;
}
