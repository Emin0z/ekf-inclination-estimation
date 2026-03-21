## Project Description

This project focuses on inclination estimation using an IMU consisting of two 3-axis gyroscopes and one accelerometer.

In dynamic conditions, each sensor has inherent limitations:
- Accelerometers are sensitive to external accelerations, leading to noisy measurements and sudden spikes
- Gyroscopes provide smooth short-term estimates but suffer from drift over time

To overcome these issues, a sensor fusion approach based on the Extended Kalman Filter (EKF) is implemented.

## EKF Framework

The EKF is a recursive state estimation algorithm designed for nonlinear systems. It estimates the system state by combining:
- A prediction model (system dynamics)
- A correction step (sensor measurements)

The filter operates in two main steps:

### Prediction Step

The state is propagated using the nonlinear system model:

$$
\hat{\mathbf{x}}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k)
$$

$$
\hat{\mathbf{P}}_k = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

### Correction Step

The prediction is corrected using measurements:

$$
\mathbf{v}_k = \mathbf{z}_k - h(\mathbf{x}_k)
$$

$$
\mathbf{K}_k = \mathbf{P}_k \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k \mathbf{H}_k^T + \mathbf{R}_k)^{-1}
$$

$$
\mathbf{x}_k = \hat{\mathbf{x}}_k + \mathbf{K}_k \mathbf{v}_k
$$

$$
\mathbf{P}_k = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)\hat{\mathbf{P}}_k
$$

The Kalman gain $\mathbf{K}$ determines how much the filter trusts the measurements versus the model. :contentReference[oaicite:0]{index=0}

## Sensor Fusion Strategy

- Gyroscope data is used for short-term state propagation (prediction step)
- Accelerometer data is used in the correction step to compensate for drift

This combination allows the filter to provide a more stable and physically consistent estimate of inclination.

## Noise Modeling

The EKF explicitly accounts for uncertainty in both:
- System dynamics (process noise $\mathbf{Q}$)
- Sensor measurements (measurement noise $\mathbf{R}$)

These matrices were estimated using sensor data collected in static conditions, ensuring realistic noise characterization.

## Measurement Model

The accelerometer measurement is modeled as the projection of the gravity vector into the body frame:

$$
\mathbf{z} = \mathbf{R}(\mathbf{q})^T \mathbf{g}
$$

This assumption is valid only when no significant external acceleration is present.



## Limitations

A key limitation of this approach is:

- The accelerometer is assumed to measure only gravity
- During dynamic motion, it also captures external accelerations
- This violates the measurement model and can degrade accuracy

## Implementation Notes

Due to limited datasets, extensive parameter tuning was not performed.

In practice, EKF performance is highly dependent on:
- Noise covariance matrices ($Q$, $R$)
- System dynamics
- Application-specific motion profiles

For example:
- Automotive systems
- Railway systems

Each requires dedicated tuning for optimal performance.

## Conclusion

This project demonstrates a complete EKF-based sensor fusion pipeline for inclination estimation, including:

- Nonlinear system modeling
- State estimation using quaternion representation
- Sensor fusion between gyroscope and accelerometer
- Bias-aware estimation framework

While further tuning is required for deployment, the implemented framework provides a solid and extensible foundation for real-world applications.
