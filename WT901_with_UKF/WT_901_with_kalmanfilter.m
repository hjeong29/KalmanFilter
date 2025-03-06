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
quaternion_start_index = 77;% Quaternion block header (0x55, 0x56)
magnetic_start_index = 44;  % Magnetic block header (0x55, 0x54)

% Data storage structure and settings
batch_size = 10000;  % Maximum data per batch
count = 1;           % Current structure index
data_array = struct("data1", []);

% Real-time 3D plot setup
drawing = 1; % Enable/Disable scatter plot (1: Enable, 0: Disable)
% figure;
% h = scatter3(nan, nan, nan, 'bo', 'filled');
% axis([-60 60 -60 60 -60 60]);
% grid on;
% xlabel('Mag X');
% ylabel('Mag Y');
% zlabel('Mag Z');
% title('Real-time 3D Scatter Plot of WT901 Sensor Data');
Eular_angles = zeros(4,0);
Eular_angles1 = zeros(4,0);
imu_data = zeros(4,0);
kalman_data = zeros(4,0);
%% 2) UKF 객체 생성
% (초기값: q=[1 0 0 0], gyro bias=[0 0 0], 기타 파라미터 임의)
ukf = UKF_9Axis([1;0;0;0], [0;0;0], ...
                0.01, ...   % initVar
                0.01, ...   % gyroNoise
                0.001, ...  % biasNoise
                0.5,  ...   % accelNoise
                1.0);       % magNoise

% 위 파라미터(초기공분산, 잡음 등)는 실제 센서 스펙에 따라 조정해야 함.

%% 3) 결과 저장용

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
        data = read(s, packet_length * floor(s.NumBytesAvailable / packet_length), 'uint8'); % read in bulk because buffer may be stored way more than packet length.
        num_packets = length(data) / packet_length;
        data_matrix = reshape(data, packet_length, num_packets)'; % build data matrix in "num_packets x packet_length"

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

        % Magnetic data conversion (divide by 1000 for scale for uT scale)
        Hx = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 4)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 3)))) / 160;
        Hy = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 6)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 5)))) / 160;
        Hz = double(bitor(bitshift(int16(data_matrix(:, magnetic_start_index + 8)), 8), ...
                          int16(data_matrix(:, magnetic_start_index + 7)))) / 160;

        % Quaternion data conversion (multiply by 180/32768)
        Q0 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 4)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 3)))) / 32768;
        Q1 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 6)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 5)))) / 32768;
        Q2 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 8)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 7)))) / 32768;
        Q3 = double(bitor(bitshift(int16(data_matrix(:, quaternion_start_index + 10)), 8), ...
                          int16(data_matrix(:, quaternion_start_index + 9)))) / 32768;
        Acc=[Ax,Ay,Az]'*9.805;
        Gyr=[Gx,Gy,Gz]'*pi/180;
        Mag=[Hx,Hy,Hz]';
        Q=[Q0,Q1,Q2,Q3]';
        N=size(Acc,2);
        qEst_all = zeros(4, N);
        for i=1:N
            dt = 0.01;
            if dt <= 0
                dt = 0.01; % 혹시 시간 간격이 0이거나 감소하면 예비처리
            end
            
            % 현재 센서 측정
            acc_i = Acc(:,i);   % (3x1)
            gyr_i = Gyr(:,i);   % (3x1)
            mag_i = Mag(:,i);   % (3x1)
            
            % (a) 예측
            ukf.predict(gyr_i, dt);
            
            % (b) 갱신
            ukf.update(acc_i, mag_i);
            
            % 추정 쿼터니언 저장
            qEst_all(:,i) = ukf.x(1:4);
        end
        % Combine sensor data into a single row
        sensor_data = [t';Acc;Gyr;Mag;qEst_all];
        
        % 새로운 데이터를 append
        Eular_angles1 = [Eular_angles1,quaternionsToZYXAngles(Q)];
        Eular_angles = [Eular_angles,quaternionsToZYXAngles(-qEst_all)];
        imu_data = [imu_data, Q];
        kalman_data = [kalman_data, -qEst_all];
        
        % 데이터 길이 제한 (최대 800개)
        if size(imu_data,2) > 800
            Eular_angles = Eular_angles(:, end-799:end);
            Eular_angles1 = Eular_angles1(:, end-799:end);
            imu_data = imu_data(:, end-799:end);
            kalman_data = kalman_data(:, end-799:end);
        end
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
            figure(1);
            
            subplot(2,2,1);
            plot(Eular_angles(1,:), 'b'); 
            hold on;
            plot(Eular_angles1(1,:), 'r'); 
            hold off;
            ylim([-180,180])
            title('Yaw angle degree');
            legend('ukf','internal');
            ylabel('Value'); grid on;
            
            
            subplot(2,2,2);
            plot(Eular_angles(2,:), 'b');
            hold on;
            plot(Eular_angles1(2,:), 'r'); 
            hold off;
            ylim([-180,180])

            title('Roll angle degree');
            legend('ukf','internal');
            ylabel('Value'); grid on;
            
            subplot(2,2,3);
            plot(Eular_angles(3,:), 'b');
            hold on;
            plot(Eular_angles1(3,:), 'r'); 
            hold off;
            ylim([-180,180])

            title('Pitch angle degree');
            legend('ukf','internal');
            ylabel('Value'); grid on;
            
            drawnow;

        end

        if drawing == 3
            figure(1);
            
            subplot(4,1,1);
            plot(imu_data(1,:), 'b'); hold on;
            plot(kalman_data(1,:), 'r'); hold off;
            title('Quaternion Component q0');
            legend('IMU','Kalman Filter');
            ylabel('Value'); grid on;
            
            subplot(4,1,2);
            plot(imu_data(2,:), 'b'); hold on;
            plot(kalman_data(2,:), 'r'); hold off;
            title('Quaternion Component q1');
            legend('IMU','Kalman Filter');
            ylabel('Value'); grid on;
            
            subplot(4,1,3);
            plot(imu_data(3,:), 'b'); hold on;
            plot(kalman_data(3,:), 'r'); hold off;
            title('Quaternion Component q2');
            legend('IMU','Kalman Filter');
            ylabel('Value'); grid on;
            
            subplot(4,1,4);
            plot(imu_data(4,:), 'b'); hold on;
            plot(kalman_data(4,:), 'r'); hold off;
            title('Quaternion Component q3');
            legend('IMU','Kalman Filter');
            ylabel('Value'); grid on;
            xlabel('Samples');
            
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
function angles = quaternionsToZYXAngles(q)
    % 입력: q = 4 x N 행렬 (N개의 샘플에 대한 쿼터니언)
    % 출력: angles = 3 x N 행렬 ([Yaw; Pitch; Roll] 각도, 단위: 도)

    % 쿼터니언 성분 분리
    q0 = q(1, :); % 스칼라 성분
    q1 = q(2, :); % 벡터 성분 x
    q2 = q(3, :); % 벡터 성분 y
    q3 = q(4, :); % 벡터 성분 z

    % Yaw (θz) - Z축 회전
    theta_z = atan2(2 * (q0 .* q3 + q1 .* q2), 1 - 2 * (q2.^2 + q3.^2));

    % Pitch (θy) - Y축 회전
    sin_theta_y = 2 * (q0 .* q2 - q3 .* q1);
    sin_theta_y = max(-1.0, min(1.0, sin_theta_y)); % asin 입력값 범위 제한
    theta_y = asin(sin_theta_y);

    % Roll (θx) - X축 회전
    theta_x = atan2(2 * (q0 .* q1 + q2 .* q3), 1 - 2 * (q1.^2 + q2.^2));

    % 각도를 도(degree) 단위로 변환
    theta_z = rad2deg(theta_z);
    theta_y = rad2deg(theta_y);
    theta_x = rad2deg(theta_x)+11.5;

    % 결과를 3 x N 행렬로 반환 ([Yaw; Pitch; Roll])
    angles = [theta_z; theta_y; theta_x];
end
