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
accel_start_index = 11;     % Accelerometer block header (0x55, 0x51)
gyro_start_index = 22;      % Gyroscope block header (0x55, 0x52)
quaternion_start_index = 66;% Quaternion block header (0x55, 0x56)
magnetic_start_index = 44;  % Magnetic block header (0x55, 0x54)

% Data storage structure and settings
batch_size = 10000;  % Maximum data per batch
count = 1;           % Current structure index
data_array = struct("data1", []);

% Real-time 3D plot setup
drawing = 1; % Enable/Disable scatter plot (1: Enable, 0: Disable)
figure;
h = scatter3(nan, nan, nan, 'bo', 'filled');
axis([-60 60 -60 60 -60 60]);
grid on;
xlabel('Mag X');
ylabel('Mag Y');
zlabel('Mag Z');
title('Real-time 3D Scatter Plot of WT901 Sensor Data');

%% 1. Setup Phase: Header Detection and Initialization
buffer = [0, 0];
found = 0;
header_valid = 0;
flush(s)

while isvalid(s)
    while ~header_valid
        if found == 0
            % Buffer shift and read 1 byte
            buffer(1) = buffer(2);
            if s.NumBytesAvailable > 0
                buffer(2) = read(s, 1, 'uint8');
            end
            
            % Detect Magnetic data header (0x55, 0x50)
            if buffer(1) == 0x55 && buffer(2) == 0x50
                disp('Magnetic data header detected. Preparing for initialization.');
                found = 1;
            end
        elseif found == 1
            % Read the remaining 97 bytes to initialize
            if s.NumBytesAvailable >= 97
                read(s, 97, 'uint8');
                found = 0;
                header_valid = 1;
                disp('Buffer initialized and header verification complete.');
                break;
            end
        end
    end
    
    % Verification step for the setup phase
    if s.NumBytesAvailable >= packet_length
        data = read(s, packet_length, 'uint8');
        disp('Validating setup phase data...');
        
        % Check if the data header is (0x55, 0x50)
        if data(1) == 0x55 && data(2) == 0x50
            disp('Setup verification successful! Proceeding to the main loop.');
            break;
        else
            disp('Setup verification failed. Returning to header detection phase.');
            data
            header_valid = 0;
        end
    end
end

%% 2. Main Loop: Stable Data Reception and Processing
while isvalid(s)
    if s.NumBytesAvailable >= packet_length
        % Read available data in multiples of the packet length
        data = read(s, packet_length * floor(s.NumBytesAvailable / packet_length), 'uint8');
        num_packets = length(data) / packet_length;
        data_matrix = reshape(data, packet_length, num_packets)';

        % Convert internal timestamp to seconds
        t = getTimestampInSeconds(data_matrix);

        % Accelerometer data conversion (divide by 2048 for range ±16g)
        Ax = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 4)), 8), ...
                          int16(data_matrix(:, accel_start_index + 3)))) / 2048;
        Ay = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 6)), 8), ...
                          int16(data_matrix(:, accel_start_index + 5)))) / 2048;
        Az = double(bitor(bitshift(int16(data_matrix(:, accel_start_index + 8)), 8), ...
                          int16(data_matrix(:, accel_start_index + 7)))) / 2048;

        % Gyroscope data conversion (multiply by 2000/32768 for range ±2000°/s)
        Gx = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 4)), 8), ...
                          int16(data_matrix(:, gyro_start_index + 3)))) * 2000 / 32768;
        Gy = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 6)), 8), ...
                          int16(data_matrix(:, gyro_start_index + 5)))) * 2000 / 32768;
        Gz = double(bitor(bitshift(int16(data_matrix(:, gyro_start_index + 8)), 8), ...
                          int16(data_matrix(:, gyro_start_index + 7)))) * 2000 / 32768;

        % Magnetic data conversion (divide by 1000 for scale)
        Hx = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 4)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 3)))) / 1000;
        Hy = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 6)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 5)))) / 1000;
        Hz = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 8)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 7)))) / 1000;

        % Quaternion data conversion (multiply by 180/32768 for angle in degrees)
        Q0 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 4)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 3)))) * 180 / 32768;
        Q1 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 6)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 5)))) * 180 / 32768;
        Q2 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 8)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 7)))) * 180 / 32768;
        Q3 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 10)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 9)))) * 180 / 32768;

        % Combine sensor data into a single row
        sensor_data = [t'; Ax'; Ay'; Az'; Gx'; Gy'; Gz'; Hx'; Hy'; Hz'; Q0'; Q1'; Q2'; Q3'];

        % Check if we need a new substructure for batch storage
        field_name = "data" + count;
        if size(data_array.(field_name), 2) + size(sensor_data, 2) > batch_size
            count = count + 1;
            field_name = "data" + count;
            data_array.(field_name) = [];
        end
        data_array.(field_name) = [data_array.(field_name), sensor_data];

        % Update scatter plot if drawing is enabled
        if drawing == 1
            set(h, 'XData', Hx, 'YData', Hy, 'ZData', Hz);
            drawnow;
        end
    end
end

clear s;

function t = getTimestampInSeconds(data)
    % Convert internal timestamp to seconds
    DD = double(data(:,5));
    hh = double(data(:,6));
    mm = double(data(:,7));
    ss = double(data(:,8));
    msL = double(data(:,9));
    msH = double(data(:,10));
    ms = double(bitor(bitshift(uint16(msH), 8), uint16(msL))) / 1000.0;
    t = (((((DD * 24 + hh) * 60 + mm) * 60) + ss) + ms);
end
