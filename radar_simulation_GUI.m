% Simulation parameters
c = 3e8; % Speed of light (m/s)
f0 = 24e9; % Carrier frequency (Hz)
B = 500e6; % Bandwidth (Hz)
dt = 0.1; % Time step (s)
T_chirp = dt; % Chirp duration (s)
t_total = 20; % Total simulation time (s)
N_chirps = t_total / T_chirp; % Number of chirps
fs = 128e3; % Sampling frequency (Hz)
N_samples = 128; % Samples per chirp
SNR = 10; % Signal-to-noise ratio (dB)

% Derived parameters
lambda = c / f0; % Wavelength (m)
range_resolution = c / (2 * B); % Range resolution (m)
range_bins = (0:N_samples-1) * range_resolution; % Range axis
velocity_bins = linspace(-c / (2 * T_chirp), c / (2 * T_chirp), N_chirps); % Velocity axis

% Initialize beat signal
beat_signal = zeros(N_samples, N_chirps);

% Simulate UAV motion
R0 = 5; % Initial range (m)
vr = -1; % Radial velocity (m/s)

for chirp = 1:N_chirps
    t_fast = (0:N_samples-1) / fs; % Fast time
    range = R0 + vr * (chirp * T_chirp); % Updated range (m)
    time_delay = 2 * range / c; % Time delay (s)
    beat_freq = 2 * B * time_delay / T_chirp; % Beat frequency (Hz)
    
    % Simulate beat signal
    beat_signal(:, chirp) = exp(1j * 2 * pi * beat_freq * t_fast);
end

% Add Gaussian noise to simulate realistic conditions
noise = (randn(size(beat_signal)) + 1j * randn(size(beat_signal))) / sqrt(2);
beat_signal = beat_signal + 10^(-SNR/20) * noise;

% Perform 2D-FFT to generate Range-Velocity Map
range_velocity_map = fftshift(abs(fft2(beat_signal, N_samples, N_chirps)), 2);

% Plot Range-Velocity Map
figure;
imagesc(range_bins, velocity_bins, 20 * log10(range_velocity_map));
title('Range-Velocity Map');
xlabel('Range (m)');
ylabel('Radial Velocity (m/s)');
colorbar;
axis xy;

%% Section 2: Real-Time Drone Movement Visualization
% Initial drone parameters
initial_position = [0, 5]; % [x, y] initial position (m)
initial_velocity = [-1, 0]; % [vx, vy] initial velocity (m/s)
initial_angular_velocity = deg2rad(10); % Angular velocity (rad/s)

% Simulation parameters
time_steps = 0:dt:t_total; % Time array
n_steps = length(time_steps);

% Initialize arrays for storing drone position and velocity
positions = zeros(n_steps, 2); % [x, y]
velocities = zeros(n_steps, 2); % [vx, vy]

% Set initial conditions
positions(1, :) = initial_position;
velocities(1, :) = initial_velocity;

% Create figure for real-time visualization
figure;
hold on;
grid on;
axis equal;
title('Drone Real-Time Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
xlim([-15, 15]); % Adjust axis limits based on expected movement
ylim([-15, 15]);
trajectory_plot = plot(positions(1, 1), positions(1, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
drone_marker = plot(positions(1, 1), positions(1, 2), 'ro', 'MarkerSize', 8, 'DisplayName', 'Drone');
legend;

% Simulate drone movement
for t = 2:n_steps
    % Update angular velocity with random fluctuation (e.g., turbulence effect)
    angular_velocity = initial_angular_velocity + 0.05 * randn();
    
    % Compute velocity rotation due to angular velocity
    rotation_matrix = [cos(angular_velocity * dt), -sin(angular_velocity * dt);
                       sin(angular_velocity * dt),  cos(angular_velocity * dt)];
    velocities(t, :) = (rotation_matrix * velocities(t-1, :)')';
    
    % Update position based on velocity
    positions(t, :) = positions(t-1, :) + velocities(t, :) * dt;
    
    % Update trajectory plot
    set(trajectory_plot, 'XData', positions(1:t, 1), 'YData', positions(1:t, 2));
    set(drone_marker, 'XData', positions(t, 1), 'YData', positions(t, 2));
    
    % Pause to simulate real-time movement
    pause(dt);
end

% Highlight initial and final positions
plot(initial_position(1), initial_position(2), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start Position');
plot(positions(end, 1), positions(end, 2), 'mo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End Position');
legend('Trajectory', 'Drone', 'Start Position', 'End Position');

%% Section 3: Antenna Reception and Drone Tracking
% Parameters for antenna configuration
antenna_distance = 1.3; % Distance between antennas (m)
radar_position = [0, 0]; % Radar position at the origin
antenna_positions = [0, 0; antenna_distance, 0]; % Antenna 1 at (0,0), Antenna 2 at (1.3,0)

% Initialize signals received by both antennas
signal_antenna1 = zeros(N_samples, N_chirps); % Signal at Antenna 1
signal_antenna2 = zeros(N_samples, N_chirps); % Signal at Antenna 2

% Simulate reception of the signal at both antennas
for chirp = 1:N_chirps
    % Current drone position
    drone_position = positions(chirp + 1, :); % From Section 2
    
    % Calculate range to both antennas
    range_antenna1 = norm(drone_position - antenna_positions(1, :));
    range_antenna2 = norm(drone_position - antenna_positions(2, :));
    
    % Time delay and Doppler shift
    time_delay1 = 2 * range_antenna1 / c;
    time_delay2 = 2 * range_antenna2 / c;
    
    % Beat frequencies
    beat_freq1 = 2 * B * time_delay1 / T_chirp;
    beat_freq2 = 2 * B * time_delay2 / T_chirp;
    
    % Fast time
    t_fast = (0:N_samples-1) / fs;
    
    % Generate beat signals
    signal_antenna1(:, chirp) = exp(1j * 2 * pi * beat_freq1 * t_fast);
    signal_antenna2(:, chirp) = exp(1j * 2 * pi * beat_freq2 * t_fast);
end

% Add Gaussian noise to the signals
noise_antenna1 = (randn(size(signal_antenna1)) + 1j * randn(size(signal_antenna1))) / sqrt(2);
noise_antenna2 = (randn(size(signal_antenna2)) + 1j * randn(size(signal_antenna2))) / sqrt(2);
signal_antenna1 = signal_antenna1 + 10^(-SNR/20) * noise_antenna1;
signal_antenna2 = signal_antenna2 + 10^(-SNR/20) * noise_antenna2;

% Calculate phase difference between antennas at a specific sample index
sample_idx = 1; % Choose a sample index
phase_difference_per_chirp = angle(signal_antenna1(sample_idx, :)) - angle(signal_antenna2(sample_idx, :));

% Estimate angle of arrival (AoA)
aoa_estimate = asin((c / (2 * pi * f0)) * phase_difference_per_chirp / antenna_distance);

% Perform FFT on signals for range-Doppler analysis
fft_antenna1 = fftshift(abs(fft2(signal_antenna1, N_samples, N_chirps)), 2);
fft_antenna2 = fftshift(abs(fft2(signal_antenna2, N_samples, N_chirps)), 2);

% Plot FFT results for both antennas
figure;
subplot(2, 1, 1);
imagesc(range_bins, velocity_bins, 20 * log10(fft_antenna1));
title('Range-Velocity Map - Antenna 1');
xlabel('Range (m)');
ylabel('Radial Velocity (m/s)');
colorbar;
axis xy;

subplot(2, 1, 2);
imagesc(range_bins, velocity_bins, 20 * log10(fft_antenna2));
title('Range-Velocity Map - Antenna 2');
xlabel('Range (m)');
ylabel('Radial Velocity (m/s)');
colorbar;
axis xy;

%% Kalman Filter for Drone Tracking
% State vector: [x; y; vx; vy]
state = [positions(2, 1); positions(2, 2); initial_velocity(1); initial_velocity(2)];
state_covariance = eye(4) * 10; % Initial covariance

% Process and measurement noise covariances
process_noise = diag([0.1, 0.1, 0.01, 0.01]); % Process noise (small for position, velocity)
measurement_noise = diag([0.5, 0.5]); % Measurement noise (higher for AoA)

% Kalman filter matrices
F = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1]; % State transition matrix
H = [1, 0, 0, 0; 0, 1, 0, 0]; % Measurement matrix

% Initialize arrays for storing tracking results
kalman_positions = zeros(n_steps, 2);
kalman_positions(1, :) = positions(2, :); % Initialize with second position

for t = 2:n_steps
    % Prediction step
    predicted_state = F * state;
    predicted_covariance = F * state_covariance * F' + process_noise;
    
    % Measurement (position derived from AoA and range)
    measured_range = norm(positions(t, :) - radar_position);
    measured_angle = aoa_estimate(t - 1); % Adjust index
    
    measured_position = radar_position + [measured_range * cos(measured_angle), measured_range * sin(measured_angle)];
    
    % Update step
    innovation = measured_position' - H * predicted_state;
    innovation_covariance = H * predicted_covariance * H' + measurement_noise;
    kalman_gain = predicted_covariance * H' / innovation_covariance;
    state = predicted_state + kalman_gain * innovation;
    state_covariance = (eye(4) - kalman_gain * H) * predicted_covariance;
    
    % Store position
    kalman_positions(t, :) = state(1:2)';
end

% Plot tracking results
figure;
plot(positions(:, 1), positions(:, 2), 'b--', 'DisplayName', 'True Path');
hold on;
plot(kalman_positions(:, 1), kalman_positions(:, 2), 'r-', 'DisplayName', 'Kalman Filtered Path');
scatter(radar_position(1), radar_position(2), 100, 'k', 'filled', 'DisplayName', 'Radar');
title('Drone Tracking Using Kalman Filter');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend;
grid on;
axis equal;
