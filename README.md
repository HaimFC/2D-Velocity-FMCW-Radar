
# Radar Simulation Project

## Overview
This project involves the simulation of an FMCW (Frequency Modulated Continuous Wave) radar system designed for detecting and tracking UAVs (Unmanned Aerial Vehicles). The simulation is implemented in MATLAB and covers key aspects such as signal generation, noise modeling, range-velocity mapping, and real-time tracking using a Kalman filter.

---

## Key Objectives
1. Simulate radar signals to detect and track UAV motion.
2. Generate Range-Velocity maps using 2D FFT.
3. Estimate angle of arrival (AoA) using interferometric principles.
4. Apply a Kalman filter for accurate trajectory tracking under noisy conditions.

---

## Simulation Components

### 1. FMCW Radar Parameters
- **Carrier Frequency**: \( f_0 = 24 \, \text{GHz} \)
- **Bandwidth**: \( B = 500 \, \text{MHz} \)
- **Chirp Duration**: \( T_\text{chirp} = 0.1 \, \text{s} \)
- **Sampling Frequency**: \( f_s = 128 \, \text{kHz} \)
- **Range Resolution**: \( \Delta R = \frac{c}{2B} \)
- **Velocity Resolution**: Determined by the Doppler shift over the chirp duration.

---

### 2. Signal Generation
1. **Beat Signal**:
   - Simulated using:
     \[
     s(t) = \exp(\mathrm{j} 2 \pi f_\text{beat} t)
     \]
   - Beat frequency is derived from the time delay of the reflected signal.
   - Gaussian noise is added to simulate realistic conditions.

2. **Range-Velocity Mapping**:
   - Computed using a 2D FFT of the beat signal.
   - Range and velocity bins are plotted in a Range-Velocity Map.

---

### 3. Real-Time Drone Tracking
1. **Drone Motion Simulation**:
   - UAV trajectory simulated with random angular velocity fluctuations to model turbulence.
   - Positions updated iteratively based on velocity and angular changes.

2. **Angle of Arrival (AoA) Estimation**:
   - Calculated from the phase difference between signals received at two antennas:
     \[
     \phi = \arcsin\left(\frac{c \cdot \Delta\phi}{2 \pi f_0 \cdot d}\right)
     \]
     where \( d \) is the antenna spacing.

3. **Kalman Filter**:
   - Tracks UAV trajectory based on noisy measurements.
   - State vector includes position \([x, y]\) and velocity \([v_x, v_y]\).

---

### MATLAB Highlights

#### **Range-Velocity Map**
```matlab
range_velocity_map = fftshift(abs(fft2(beat_signal, N_samples, N_chirps)), 2);
figure;
imagesc(range_bins, velocity_bins, 20 * log10(range_velocity_map));
title('Range-Velocity Map');
xlabel('Range (m)');
ylabel('Radial Velocity (m/s)');
colorbar;
axis xy;
```

#### **Kalman Filter**
```matlab
for t = 2:n_steps
    % Prediction step
    predicted_state = F * state;
    predicted_covariance = F * state_covariance * F' + process_noise;

    % Measurement step
    measured_position = radar_position + ...
        [measured_range * cos(measured_angle), measured_range * sin(measured_angle)];
    innovation = measured_position' - H * predicted_state;

    % Update step
    innovation_covariance = H * predicted_covariance * H' + measurement_noise;
    kalman_gain = predicted_covariance * H' / innovation_covariance;
    state = predicted_state + kalman_gain * innovation;
    state_covariance = (eye(4) - kalman_gain * H) * predicted_covariance;

    % Store results
    kalman_positions(t, :) = state(1:2)';
end
```

---

## Summary
This simulation demonstrates the capabilities of an FMCW radar system for UAV detection and tracking. By combining interferometric techniques with real-time processing and Kalman filtering, the system achieves robust performance under noisy conditions.
