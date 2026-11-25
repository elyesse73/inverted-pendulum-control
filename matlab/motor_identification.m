function [sys_left, sys_right] = motor_identification(method)
%% Motor & Drivetrain System Identification (2nd-Order Transfer Function)
% Acquires multi-excitation telemetry from firmware/motor_identification and
% fits continuous-time 2nd-order transfer functions for left and right wheels.
%
% Usage:
%   [sys_left, sys_right] = motor_identification()        % defaults to "WiFi"
%   [sys_left, sys_right] = motor_identification("USB")

arguments
    method (1,1) string {mustBeMember(method, ["WiFi", "USB"])} = "WiFi"
end

%% 1. Acquisition Settings
WIFI_IP     = '192.168.4.1';
WIFI_PORT   = 80;
USB_PORT    = 'COM7';
USB_BAUD    = 1000000;
DURATION_S  = 18;
SUPPLY_V    = 12.0;            % Motor battery supply voltage [V]
GAINS_EMPTY = [];

if method == "WiFi"
    [log_time, data_values, ~] = get_data_WiFi(WIFI_IP, WIFI_PORT, DURATION_S, GAINS_EMPTY);
else
    [log_time, data_values, ~] = get_data_USB(USB_PORT, DURATION_S, USB_BAUD, GAINS_EMPTY);
end

disp('Logged telemetry channels:');
disp(data_values.keys);

%% 2. Plot Raw Excitation and Wheel Responses
figure('Name', 'Motor Identification Telemetry');
hold on;
legend_labels = data_values.keys;
for idx = 1:numel(legend_labels)
    channel = legend_labels{idx};
    plot(log_time, data_values(channel), 'DisplayName', channel);
end
hold off;
xlabel('Time [s]');
ylabel('Signal');
title('Open-Loop Multi-Excitation Motor Telemetry');
legend('Location', 'best');
grid on;

%% 3. Continuous-Time Transfer Function Estimation (2 poles, 0 zeros)
Ts = median(diff(log_time));
u_volts = data_values('uDuty') * SUPPLY_V;
u_volts = u_volts(:);

y_left  = deg2rad(data_values('LeftPos'));
y_left  = y_left(:);

y_right = deg2rad(data_values('RightPos'));
y_right = y_right(:);

data_left  = iddata(y_left, u_volts, Ts);
data_right = iddata(y_right, u_volts, Ts);

sys_left  = tfest(data_left, 2, 0);
sys_right = tfest(data_right, 2, 0);

[num_l, den_l] = tfdata(sys_left, 'v');
[num_r, den_r] = tfdata(sys_right, 'v');

fprintf('\nSampling period Ts = %.4f s (%.1f Hz)\n', Ts, 1 / Ts);
disp('Left motor TF numerator / denominator:');
disp(num_l); disp(den_l);
disp('Right motor TF numerator / denominator:');
disp(num_r); disp(den_r);
end