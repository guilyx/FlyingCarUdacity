# Code Abstract #

## Sensor Noise

Completed in a small python script :

```
from numpy import loadtxt as nploadtxt
from numpy import std as npstd

gps = nploadtxt('./data/GPS_error.txt', delimiter=',', dtype='Float64', skiprows=1)
ax = nploadtxt('./data/AX_error.txt', delimiter=',', dtype='Float64', skiprows=1)

gps_std = npstd(gps[:,1])
ax_std = npstd(ax[:,1])
```

Then I just had to enter the values returned in Sensornoise.txt

## Attitude Estimation

```
// Use Quaternion template
Quaternion<float> quadAttitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
V3D bodyRates = V3D(gyro.x, gyro.y, gyro.z);
Quaternion<float> predictedQuadAttitude = quadAttitude.IntegrateBodyRate(bodyRates, dtIMU);

double predictedQuadPitch = predictedQuadAttitudePitch();
double predictedQuadRoll = predictedQuadAttitudeRol();
ekfState(6) = predictedQuadAttitude.Yaw();
```

## Prediction

### Predict State
```
V3F accelWorldFrame = attitude.Rotate_BtoI(accel);

//Position update
predictedState[0] += curState[3] * dt;
predictedState[1] += curState[4] * dt;
predictedState[2] += curState[5] * dt;

//Velocity update
predictedState[3] += accelWorldFrame[0] * dt;
predictedState[4] += accelWorldFrame[1] * dt;
predictedState[5] += (accelWorldFrame[2] - CONST_GRAVITY) * dt;
```

### Get RBG Prime 
```
float cosTheta = cos(pitch);
float sinTheta = sin(pitch);
float cosPhi = cos(roll);
float sinPhi = sin(roll);
float sinPsi = sin(yaw);
float cosPsi = cos(yaw);
  
RbgPrime(0,0) = -cosTheta * sinPsi;
RbgPrime(0,1) = -sinPhi  * sinTheta * sinPsi - cosTheta * cosPsi;
RbgPrime(1,0) = cosTheta * cosPsi;
RbgPrime(1,1) = sinPhi  * sinTheta * cosPsi - cosPhi * sinPsi;
RbgPrime(0,2) = -cosPhi  * sinTheta * sinPsi + sinPhi   * cosPsi;
RbgPrime(1,2) = cosPhi  * sinTheta * cosPsi + sinPhi * sinPsi;
```

### Predict
```
gPrime(0, 3) = dt;
gPrime(1, 4) = dt;
gPrime(2, 5) = dt;
gPrime(3, 6) = (RbgPrime(0, 0) * accel.x + RbgPrime(0, 1) * accel.y + RbgPrime(0, 2) * (accel.z - 9.81f)) * dt;
gPrime(4, 6) = (RbgPrime(1, 0) * accel.x + RbgPrime(1, 1) * accel.y + RbgPrime(1, 2) * (accel.z - 9.81f)) * dt;
gPrime(5, 6) = (RbgPrime(2, 0) * accel.x + RbgPrime(2, 1) * accel.y + RbgPrime(2, 2) * (accel.z - 9.81f)) * dt;

MatrixXf gPrimeTrs(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);
MatrixXf sigG(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES);

gPrimeTrs = gPrime;
gPrimeTrs.transposeInPlace();

sigG = gPrime * (ekfCov * gPrimeTrs) + Q;
ekfCov = sigG;
```

## Magnetometer Update

```
zFromX(0) = ekfState(6);
float delta = magYaw - ekfState(6);
if ( delta > F_PI ) {
  zFromX(0) += 2*F_PI;
} else if ( delta < -F_PI ) {
  zFromX(0) -= 2*F_PI;
}

hPrime(0, 6) = 1;
```

## Closed Loop + GPS update

```
std::size_t matrixSize = 6;
for (std::size_t i = 0; i < matrixSize; i++) {
    zFromX(i) = ekfState(i);
    hPrime(i, i) = 1;
}
```

## Controller

```
# Position control gains
kpPosXY = 30
kpPosZ = 21
KiPosZ = 40

# Velocity control gains
kpVelXY = 13
kpVelZ = 8

# Angle control gains
kpBank = 10
kpYaw = 3

# Angle rate gains
kpPQR = 95, 95, 6
```

## Flight Evaluation

## Sensor Noise
```
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time

PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 68% of the time
```

## Attitude Estimation
```
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```

## Predict State
```
No criterias : prediction matches true state.
```

## Predict Covariance
```
No criterias again, but the estimations grows with the std.
```

## Magnetometer Update
```
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 76% of the time
```

# GPS Update
```
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```



## Demo 

<p align="center">
    <img src=drone_scenar5.gif" alt="Scenario 5" width="700" height="300">
</p>