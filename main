clc; clear; close all;
format long;

%% Settings
show_data = true;

% name = "test_level.txt"; bias_indx = 14000;
name = "test_nude.txt"; bias_indx = 7500;
% name = "test_sine.txt"; bias_indx = 7500;

Fs = 500;
dt = 1/Fs;
g = [0, 0, 1];

%% Load Data
data = importdata(name);
num = size(data,1);
t = (0:num-1)*dt;

gyro1 = data(:,1:3);
gyro2 = data(:,4:6);
gyro = 0.5 * (gyro1 + gyro2);

acc = data(:,7:9);

%% Show Raw Data
if show_data
    figure("Name","Raw Data")

    subplot(2,1,1)
    plot(gyro)
    legend("\omega_x","\omega_y","\omega_z")
    title("Gyroscope")

    subplot(2,1,2)
    plot(acc)
    legend("a_x","a_y","a_z")
    title("Accelerometer")
end

%% Accelerometer Filtering (EMA)
alpha = 0.05;
acc_f = zeros(size(acc));
acc_f(1,:) = acc(1,:);

for i = 2:num
    acc_f(i,:) = alpha * acc(i,:) + (1 - alpha) * acc_f(i-1,:);
end

figure("Name","Accel Filtering")
subplot(2,1,1)
plot(acc)
title("Raw")

subplot(2,1,2)
plot(acc_f)
title("Filtered")

%% Bias & Noise
gyro = gyro * pi/180;

gyro_bias = mean(gyro(1:bias_indx,:));
gyro = gyro - gyro_bias;

GyCov = 1e-7 * [0.1463, 0.1644, 0.1435];
AccCov = 1e-6 * [0.2683, 0.2965, 0.6876];

VarQ = [GyCov, 1e-5 * GyCov];
VarR = AccCov;

Rk = diag(VarR);

%% Initialization
X = zeros(7,num);
X(:,1) = [1 0 0 0 0 0 0]';

P = diag([1e-5 1e-5 1e-5 1e-5 0 0 0]);

err = zeros(3,num);
P_hist = zeros(1,num);

angle_gyro = zeros(3,num);
angle_ekf = zeros(3,num);

fnc = MyFunctions;

%% EKF Loop
for i = 2:num

    % Prediction
    [F, X(:,i)] = fnc.DynamicModel(X(:,i-1), gyro(i,:), dt);
    Q = fnc.Qupdate(VarQ, X(:,i-1), dt);
    P = F*P*F.' + Q;

    % Gyro-based angle
    Rr = fnc.q2R(X(1:4,i));
    gr = Rr.' * g.';

    angle_gyro(:,i) = [
        atan(gr(1)/sqrt(gr(2)^2 + gr(3)^2));
        atan(gr(2)/sqrt(gr(1)^2 + gr(3)^2));
        atan(gr(3)/sqrt(gr(1)^2 + gr(2)^2))
    ] * 180/pi;

    % Correction
    [H, ht] = fnc.MeasurmentModel(X(:,i), g);

    err(:,i) = acc_f(i,:).' - ht;

    S = H*P*H.' + Rk;
    K = P*H.'/S;

    X(:,i) = X(:,i) + K*err(:,i);

    P = (eye(7)-K*H)*P*(eye(7)-K*H).' + K*Rk*K.';

    % Normalize quaternion
    X(1:4,i) = X(1:4,i) / norm(X(1:4,i));

    % EKF angle
    Rr = fnc.q2R(X(1:4,i));
    gr = Rr.' * g.';

    angle_ekf(:,i) = [
        atan(gr(1)/sqrt(gr(2)^2 + gr(3)^2));
        atan(gr(2)/sqrt(gr(1)^2 + gr(3)^2));
        atan(gr(3)/sqrt(gr(1)^2 + gr(2)^2))
    ] * 180/pi;

    P_hist(i) = max(max(P));
end

disp("DONE")

%% Results

figure("Name","Covariance")
plot(t, P_hist)
title("Covariance Evolution")

figure("Name","Residual Error")
plot(t, err)
title("Measurement Residual")

figure("Name","Bias")
plot(t, X(5:7,:))
legend("Bx","By","Bz")

figure("Name","Tilt (EKF)")
plot(t, angle_ekf)
legend("Roll","Pitch","Yaw")

figure("Name","Comparison")
plot(t, angle_gyro(1,:)); hold on
plot(t, angle_ekf(1,:))
legend("Gyro Only","EKF")
title("Roll Comparison")
