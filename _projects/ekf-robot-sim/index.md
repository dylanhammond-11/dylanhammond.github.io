---
layout: post
title: EKF Localization Robot
order: 1
description: "ROS2 differential drive robot using a hand-derived Extended Kalman Filter to fuse GPS, IMU, and encoder data for real-time pose estimation during autonomous waypoint navigation."
skills:
  - Python
  - ROS2
  - Extended Kalman Filter
  - Sensor Fusion
  - Control Theory
main-image: /EKFMain.png
---
The **EKF Localization Robot** is a ROS2 simulation of a differential drive robot that estimates its own position and heading in real time by fusing noisy GPS, IMU, and wheel encoder measurements through an Extended Kalman Filter implemented from scratch.

### **The Problem** 

A robot's sensors are individually unreliable: GPS is noisy and slow, encoders drift over time, and IMUs accumulate error. The Extended Kalman Filter (EKF) combines all three, weighting each by how much it should be trusted, to produce a pose estimate that's more accurate than any single sensor alone.

### **The Model and the Filter**

The robot's state is `[x, y, θ]`: position and heading. A standard Kalman filter assumes linear motion and measurement models, but a differential drive robot model is nonlinear. The **Extended** Kalman Filter (EKF) handles this by linearizing the model around the current estimate at every timestep, using a Jacobian, then running the same predict/update math a standard Kalman filter would use.
The filter runs five equations total, split into two steps.

**Predict**: propagate the state forward using the motion model, using forward velocity `v` from the wheel encoders and angular velocity `ω` from the IMU gyro:

<p align="center">
  <img src="/_projects/ekf-robot-sim/EKFPredictEquations.png" width="400">
</p>

Neither sensor is perfect, so every prediction also grows the filter's uncertainty, tracked as a covariance matrix `P`, since it's extrapolating from a model without outside confirmation. This step runs continuously at the same rate as the encoder/IMU updates (50 Hz), since prediction only depends on having a fresh velocity input.

**Update**: correct the prediction whenever a new GPS reading arrives, pulling the estimate toward that measurement by however much it should be trusted relative to the prediction. GPS is realistically much slower than the other sensors, so I ran the sampling rate at 1Hz. The filter predicts many steps forward between each correction:

<p align="center">
  <img src="/_projects/ekf-robot-sim/EKFUpdateEquations.png" width="400">
</p>

GPS only observes position, not heading, so this step corrects x and y directly, while heading is corrected indirectly through the correlations the filter has learned between position and heading error. The same update also shrinks `P`.  You can actually watch this happen in the tuning GIFs below, where `P` is drawn directly as a covariance ellipse around the robot's estimated position.

That trust balance between model and sensors (how much the filter believes its own prediction versus a new measurement) is governed by two tunable matrices, **Q** and **R**, and getting it right is the difference between a smooth, accurate estimate and one that either lags behind reality or jitters around chasing noise, shown in the tuning comparison below.

### **Waypoint Controller**

Navigation is handled by a simple waypoint controller: steer toward the next target point using the filter's current pose estimate, advance once close enough, repeat. One adjustment I made along the way: rather than driving and turning simultaneously, the controller first rotates in place to face the next waypoint, then drives straight toward it. It's a simpler control strategy, but it kept heading error from compounding into position error during turns, which made for cleaner, more accurate navigation.

### **Tuning the Filter**

Below are three runs with identical trajectories and comparable sensor noise — only the filter's Q and R parameters change.

**1. Well-tuned**

<p align="center">
  <img src="/_projects/ekf-robot-sim/ekf_default.gif" width="700">
</p>

The estimate tracks the true trajectory closely with little variance, smoothing sensor noise without lagging behind real motion. There is some noise, but the robot follows a smooth trajectory to the endpoint.

**2. Over-trusting measurements (R too low)**

<p align="center">
  <img src="/_projects/ekf-robot-sim/ekf_measurement.gif" width="700">
</p>

The estimate jitters, and the robot follows the GPS noise. This causes the robot to get off-course.

**3. Under-trusting measurements (R too high)**

<p align="center">
  <img src="/_projects/ekf-robot-sim/ekf_model.gif" width="700">
</p>

The estimate lags and drifts, relying too heavily on the motion model and slow to correct against GPS. This causes the robot to drift off course slowly, and the EKF estimate variance stays high.


### **Conclusion**

This project was honestly one of the more fun projects I have worked on. Understanding the concepts was difficult, but now I feel I have a better understanding of the Extended Kalman Filter and its challenges, strengths, and potential. I still have much to learn but this project opened the door to localization methods for me.

A few directions I'd like to take this further:

- **Gazebo simulation**: moving past a kinematic sim into a full physics engine, with more realistic sensor noise, actuator dynamics, and (eventually) a path toward real hardware
- **Additional sensors**: bringing in LIDAR for scan matching or obstacle detection, which would also open the door to SLAM instead of localization against known waypoints
- **Alternative filters**: comparing the EKF against a UKF or particle filter


Full code is up on GitHub: [EKF Localization Robot](https://github.com/dylanhammond-11/ekf-diff-drive-robot)
