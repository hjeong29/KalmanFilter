% Real-time 3D Plot for WT901 Sensor Data (Setup Phase for Accurate Header Detection and Main Loop Stabilization)
clc;
clear all;
close all;

% Serial Port Setup
serialPort = 'COM10';
baudRate = 115200;
s = serialport(serialPort, baudRate, 'Timeout', 1);
flush(s);

% Total packet length
packet_length = 99;

% Data block start indices (Refer to sections 7.1.2 ~ 7.1.5)
accel_start_index = 11;    % Accelerometer block header (0x55, 0x51)
gyro_start_index = 22;     % Gyroscope block header (0x55, 0x52)
quaternion_start_index = 33; % Quaternion block header (0x55, 0x53)
magnetic_start_index = 44; % Magnetic block header (0x55, 0x54)

% Data storage structure and settings
batch_size = 10000; % Maximum data per batch
count = 1; % Current structure index
data_array = struct("data1", []);

% Real-time 3D plot setup
drawing = 1; % Enable/Disable drawing (1: Enable, 0: Disable)
figure;
h = scatter3(nan, nan, nan, 'bo', 'filled');
axis([-20000 20000 -20000 20000 -20000 20000]);
grid on;
xlabel('Mag X');
ylabel('Mag Y');
zlabel('Mag Z');
title('Real-time 3D Scatter Plot of WT901 Sensor Data');


%% 2. Main Loop: Stable Data Reception and Processing
while isvalid(s)
    if s.NumBytesAvailable >= packet_length
        data = read(s, packet_length * floor(s.NumBytesAvailable / packet_length), 'uint8');
        num_packets = length(data) / packet_length;
        data_matrix = reshape(data, packet_length, num_packets)';

        % Convert internal timestamp to seconds
        t = getTimestampInSeconds(data_matrix);

        % Accelerometer data conversion
        Ax = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 4)), 8), int16(data_matrix(:, accel_start_index + 3))));
        Ay = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 6)), 8), int16(data_matrix(:, accel_start_index + 5))));
        Az = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 8)), 8), int16(data_matrix(:, accel_start_index + 7))));

        % Gyroscope data conversion
        Gx = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 4)), 8), int16(data_matrix(:, gyro_start_index + 3))));
        Gy = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 6)), 8), int16(data_matrix(:, gyro_start_index + 5))));
        Gz = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 8)), 8), int16(data_matrix(:, gyro_start_index + 7))));

        % Magnetic data conversion
        Hx = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 4)), 8), int16(data_matrix(:, magnetic_start_index + 3))));
        Hy = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 6)), 8), int16(data_matrix(:, magnetic_start_index + 5))));
        Hz = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 8)), 8), int16(data_matrix(:, magnetic_start_index + 7))));

        % Quaternion data conversion
        Q0 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 4)), 8), int16(data_matrix(:, quaternion_start_index + 3))));
        Q1 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 6)), 8), int16(data_matrix(:, quaternion_start_index + 5))));
        Q2 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 8)), 8), int16(data_matrix(:, quaternion_start_index + 7))));
        Q3 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 10)), 8), int16(data_matrix(:, quaternion_start_index + 9))));

        % Combine sensor data into a single row
        sensor_data = [t'; Ax'; Ay'; Az'; Gx'; Gy'; Gz'; Hx'; Hy'; Hz'; Q0'; Q1'; Q2'; Q3'];

        field_name = "data" + count;
        if size(data_array.(field_name), 2) + size(sensor_data, 2) > batch_size
            count = count + 1;
            field_name = "data" + count;
            data_array.(field_name) = [];
        end
        data_array.(field_name) = [data_array.(field_name), sensor_data];

        % Update the scatter plot (if drawing is enabled)
        if drawing == 1
            set(h, 'XData', Hx, 'YData', Hy, 'ZData', Hz);
            drawnow;
        end
    end
end

clear s;

function t = getTimestampInSeconds(data)
    DD = double(data(:,5));
    hh = double(data(:,6));
    mm = double(data(:,7));
    ss = double(data(:,8));
    msL = double(data(:,9));
    msH = double(data(:,10));
    ms = double(bitor(bitshift(uint16(msH), 8), uint16(msL))) / 1000.0;
    t = (((((DD * 24 + hh) * 60 + mm) * 60) + ss) + ms);
end