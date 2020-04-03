# Code Abstract #

## Body Rate Control

```
V3F I(Ixx, Iyy, Izz);
momentCmd = I * kpPQR * (pqrCmd - pqr);
```

## Roll/Pitch Control

```
if ( collThrustCmd > 0 ) {
  float c = - collThrustCmd / mass;
  float b_x_cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
  float b_x_err = b_x_cmd - R(0,2);
  float b_x_p_term = kpBank * b_x_err;
  
  float b_y_cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
  float b_y_err = b_y_cmd - R(1,2);
  float b_y_p_term = kpBank * b_y_err;
  
  pqrCmd.x = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
  pqrCmd.y = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);
} else {
  pqrCmd.x = 0.0;
  pqrCmd.y = 0.0;
}

pqrCmd.z = 0;
```

## Altitude Control

```
float errorPosZ = posZCmd - posZ;
float errorVelZ = velZCmd - velZ;

integratedAltitudeError += errorPosZ * dt;

float u_ = kpPosZ * errorPosZ + (kpVelZ * errorVelZ + velZ) + KiPosZ * integratedAltitudeError + accelZCmd;
thrust = - mass * CONSTRAIN((u_ - 9.81) / R(2, 2), -maxAscentRate / dt, maxAscentRate / dt);
```

## Lateral Control

```
V3F kpPos;
kpPos.x = kpPosXY;
kpPos.y = kpPosXY;
kpPos.z = 0.f;

V3F kpVel;
kpVel.x = kpVelXY;
kpVel.y = kpVelXY;
kpVel.z = 0.f;

V3F capVelCmd;
if ( velCmd.mag() > maxSpeedXY ) {
  capVelCmd = velCmd.norm() * maxSpeedXY;
} else {
  capVelCmd = velCmd;
}

accelCmd = kpPos * ( posCmd - pos ) + kpVel * ( capVelCmd - vel ) + accelCmd;

if ( accelCmd.mag() > maxAccelXY ) {
  accelCmd = accelCmd.norm() * maxAccelXY;
}
```


## Yaw Control

```
float desiredYaw;

if (yawCmd > 0) {
  desiredYaw = fmodf(yawCmd, 2 * F_PI);
} else {
  desiredYaw = -fmodf(-yawCmd, 2 * F_PI);
}
float errorYaw = desiredYaw - yaw;

if (errorYaw > F_PI) {
  errorYaw -= 2 * F_PI;
} else if ( errorYaw <= -F_PI ) {
  errorYaw += 2* F_PI;
}
yawRateCmd = kpYaw * errorYaw;
```

## Motor Commands

```
float length = L / sqrt(2);
float A = momentCmd.x / length;
float B = momentCmd.y / length;
float C = (-1) * momentCmd.z / kappa;
float D = collThrustCmd;

cmd.desiredThrustsN[0] = (A + B + C + D) / 4.0; // front left
cmd.desiredThrustsN[1] = (-A + B - C + D) / 4.0; // front right
cmd.desiredThrustsN[2] = (A - B - C + D) / 4.0; // rear left
cmd.desiredThrustsN[3] = (-A -B + C + D) / 4.0; // rear right
```

## Flight Evaluation

## Scenario 1
```
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```

## Scenario 2
```
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

## Scenario 3
```
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```

## Scenario 4
```
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
```
## Scenario 5
```
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```