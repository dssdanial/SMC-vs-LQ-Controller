clear;
clc;
close all;
format short;

% Constants and Parameters
Ts = 1e-3;               % Sampling time
l_bar = 2.0;             % Length of bar [m]
M = 1.0;                 % Mass of cart [kg]
m = 0.3;                 % Mass of pendulum [kg]
g = 9.8;                 % Gravitational acceleration [m/s^2]
eta = 1e-1;              % Smoothing constant for control

% Define state-space matrices for the system
A = [0.0 1.0 0.0 0.0;
     0.0 0.0 m * g / M 0.0;
     0.0 0.0 0.0 1.0;
     0.0 0.0 g * (M + m) / (l_bar * M) 0.0];

B = [0.0;
     1.0 / M;
     0.0;
     1.0 / (l_bar * M)];

% Initial state
x0 = [0; 0; 0.3; 0];

% Design feedback gains using the Linear Quadratic Regulator (LQR)
Q_lqr = diag([1 1 1 1]) * 1e1;   % State penalty
R_lqr = 1;                       % Control penalty
Klq = lqr(A, B, Q_lqr, R_lqr);

% Design hyperplane using a different LQR approach for sliding mode
Q_hyper = diag([1e1 1e-1 1e0 1e0]) * 1e2;
R_hyper = 1;
S = lqr(A, B, Q_hyper, R_hyper);

% Design sliding mode controller
Ksmcl = inv(S * B) * S * A;  % Linear state feedback gain
Ksmcnl = 4;                  % Nonlinear gain for sliding mode

% Simulation with and without deadzone
amp_values = [0, 1];         % Deadzone values: 0 for without, 1 for with deadzone
metrics = struct();          % Store metrics for both simulations

for amp = amp_values
    % Run Simulink model simulation with current deadzone value
    sim('designSmcLq');

    % Compute Performance Metrics: RMSE, MAE, and MAPE
    error_pos = X(:,1) - Xlq(:,1);   % Position error between SMC and LQ
    error_angle = X(:,3) - Xlq(:,3); % Angle error between SMC and LQ
    
    % Root Mean Square Error (RMSE)
    RMSE_pos = sqrt(mean(error_pos.^2));
    RMSE_angle = sqrt(mean(error_angle.^2));
    
    % Mean Absolute Error (MAE)
    MAE_pos = mean(abs(error_pos));
    MAE_angle = mean(abs(error_angle));
    
    % Mean Absolute Percentage Error (MAPE)
    MAPE_pos = mean(abs(error_pos ./ Xlq(:,1))) * 100;
    MAPE_angle = mean(abs(error_angle ./ Xlq(:,3))) * 100;

    % Store metrics in a struct for each deadzone scenario
    metrics(amp+1).deadzone = amp;
    metrics(amp+1).RMSE = [RMSE_pos, RMSE_angle];
    metrics(amp+1).MAE = [MAE_pos, MAE_angle];
    metrics(amp+1).MAPE = [MAPE_pos, MAPE_angle];

    % Plot results for each deadzone configuration
    figure;
    subplot(3,1,1);
    plot(simTime, u(:));
    hold on; grid on;
    plot(simTime, ulq(:));
    title(sprintf('Control input u with deadzone = %d', amp));
    legend('SMC', 'LQ');

    subplot(3,1,2);
    plot(simTime, X(:,1));
    hold on; grid on;
    plot(simTime, Xlq(:,1));
    title(sprintf('Horizontal position with deadzone = %d', amp));

    subplot(3,1,3);
    plot(simTime, X(:,3));
    hold on; grid on;
    plot(simTime, Xlq(:,3));
    title(sprintf('Theta with deadzone = %d', amp));
    xlabel('time');
end

% Display calculated metrics
for i = 1:length(metrics)
    fprintf('Deadzone = %d\n', metrics(i).deadzone);
    fprintf('  RMSE (Position, Angle): %.4f, %.4f\n', metrics(i).RMSE(1), metrics(i).RMSE(2));
    fprintf('  MAE (Position, Angle): %.4f, %.4f\n', metrics(i).MAE(1), metrics(i).MAE(2));
    fprintf('  MAPE (Position, Angle): %.2f%%, %.2f%%\n\n', metrics(i).MAPE(1), metrics(i).MAPE(2));
end

% Decimate data for animation (reduce sampling rate)
X1lqd = decimate(Xlq(:,1), 100);
X3lqd = decimate(Xlq(:,3), 100);
X1d = decimate(X(:,1), 100);
X3d = decimate(X(:,3), 100);

% Animation setup for both LQ and SMC controllers
h = figure('Position', [100 100 800 400]);
axis tight manual;          % Consistent frame size for GIF
filename = 'testAnimated.gif';

% Create and save animated GIF
for k = 1:(length(simTime)-1)/100+1
    % Plot LQ control
    subplot(1, 2, 1);
    yy = cos(X3lqd(k));
    yx = sin(X3lqd(k));
    x = [X1lqd(k) - 1, X1lqd(k) + 1];
    plot(x, [0 0], 'g-', 'LineWidth', 5.5);
    ylim([-0.1, 1.1]);
    xlim([-3, 3]);
    hold on; grid on;
    plot([X1lqd(k) X1lqd(k) + yx], [0 yy], 'b-', 'LineWidth', 8.0);
    title({'Linear Quadratic Control', ['pos: ', num2str(X1lqd(k), 2)], ['angle: ', num2str(X3lqd(k), 2)]});
    hold off;

    % Plot SMC control
    subplot(1, 2, 2);
    yy = cos(X3d(k));
    yx = sin(X3d(k));
    x = [X1d(k) - 1, X1d(k) + 1];
    plot(x, [0 0], 'g-', 'LineWidth', 5.5);
    ylim([-0.1, 1.1]);
    xlim([-3, 3]);
    hold on; grid on;
    plot([X1d(k) X1d(k) + yx], [0 yy], 'k-', 'LineWidth', 1.2);
    title({'Sliding Mode Control', ['position (m): ', num2str(X1d(k), 2)], ['angle: ', num2str(X3d(k), 2)]});
    hold off;

     
end
