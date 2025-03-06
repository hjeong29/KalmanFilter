%% -------------------- 아래는 보조 클래스/함수들 --------------------
classdef UKF_9Axis < handle
    % 9축 IMU(ACC+GYR+MAG) 이용 UKF
    % 상태 x=[q0,q1,q2,q3, bwx,bwy,bwz]^T, 총7차원
    % 예:
    %   ukf = UKF_9Axis([1;0;0;0],[0;0;0], 0.01, 0.01, 0.001,0.5,1.0);
    %   for each step:
    %       ukf.predict(gyro, dt);
    %       ukf.update(acc, mag);

    properties
        x   % (7x1) [q,bias]
        P   % (7x7)
        Q   % (7x7)
        R   % (6x6) -> accel(3) + mag(3)
        alpha=1e-3
        beta=2
        kappa=0
        lambda
        w_m
        w_c
        g0=9.80665
        m0=[-1.397;19.3247;-49.2114] % 임의 자북벡터 예시 (사용자 환경에 맞춰 바꿔야 함)
    end

    methods
        function obj=UKF_9Axis(q0, b0, initVar, gyroNoise, biasNoise, accelNoise, magNoise)
            if nargin<1
                q0=[1;0;0;0]; b0=[0;0;0];
                initVar=0.01; gyroNoise=0.01; biasNoise=0.001;
                accelNoise=1.0; magNoise=1.0;
            end
            q0 = normalizeQuat(q0);
            obj.x= [q0; b0(:)];
            obj.P= eye(7)*initVar;

            % 프로세스잡음
            obj.Q= eye(7)*(gyroNoise^2);
            obj.Q(5:7,5:7)= eye(3)*(biasNoise^2);

            % 측정잡음(가속도3 + 자기장3 => 6x6)
            obj.R= blkdiag(eye(3)*(accelNoise^2), eye(3)*(magNoise^2));

            n=7;
            obj.lambda= obj.alpha^2*(n+obj.kappa)-n;
            obj.initWeights(n);
        end

        function predict(obj, gyro, dt)
            n_x=7;
            Xsigma= obj.sigmaPoints(obj.x, obj.P);

            Xsigma_pred= zeros(n_x, 2*n_x+1);
            for i=1:(2*n_x+1)
                Xsigma_pred(:,i)= processModel_9Axis(Xsigma(:,i), gyro, dt);
            end

            x_pred= weightedMeanState_Quat(Xsigma_pred, obj.w_m);
            x_pred(1:4)= normalizeQuat(x_pred(1:4));

            P_pred= zeros(n_x);
            for i=1:(2*n_x+1)
                dx= Xsigma_pred(:,i)- x_pred;
                P_pred= P_pred + obj.w_c(i)*(dx*dx');
            end
            P_pred= P_pred + obj.Q;

            obj.x= x_pred;
            obj.P= P_pred;
        end

        function update(obj, acc, mag)
            % z=[acc; mag], 6x1
            n_x=7; n_z=6;
            Xsigma= obj.sigmaPoints(obj.x, obj.P);
            Zsigma= zeros(n_z, 2*n_x+1);
            for i=1:(2*n_x+1)
                Zsigma(:,i)= measurementModel_9Axis(Xsigma(:,i), obj.g0, obj.m0);
            end
            z_pred= zeros(n_z,1);
            for i=1:(2*n_x+1)
                z_pred= z_pred + obj.w_m(i)*Zsigma(:,i);
            end
            S= zeros(n_z);
            Pxz= zeros(n_x,n_z);
            for i=1:(2*n_x+1)
                dz= Zsigma(:,i)- z_pred;
                dx= Xsigma(:,i)- obj.x;
                S= S + obj.w_c(i)*(dz*dz');
                Pxz= Pxz + obj.w_c(i)*(dx*dz');
            end
            S= S + obj.R;

            z_meas= [acc; mag];
            y= z_meas - z_pred;

            K= Pxz/S;
            obj.x= obj.x + K*y;
            obj.x(1:4)= normalizeQuat(obj.x(1:4));

            obj.P= obj.P - K*S*K';
        end
    end

    methods (Access=private)
        function initWeights(obj,n)
            obj.w_m= zeros(1,2*n+1);
            obj.w_c= zeros(1,2*n+1);
            obj.w_m(1)= obj.lambda/(n+obj.lambda);
            obj.w_c(1)= obj.lambda/(n+obj.lambda)+(1-obj.alpha^2+obj.beta);
            for i=2:(2*n+1)
                obj.w_m(i)= 1/(2*(n+obj.lambda));
                obj.w_c(i)= 1/(2*(n+obj.lambda));
            end
        end

        function Xsigma= sigmaPoints(obj, x, P)
            n= length(x);
            A= sqrtm((n+obj.lambda)*P);
            Xsigma= zeros(n, 2*n+1);
            Xsigma(:,1)= x;
            for i=1:n
                Xsigma(:,1+i)= x + A(:,i);
                Xsigma(:,1+n+i)= x - A(:,i);
            end
        end
    end
end

%% ----------- 보조 함수들 -------------
function x_next= processModel_9Axis(x, gyro, dt)
% x=[q0,q1,q2,q3, bx,by,bz]
q= x(1:4); b= x(5:7);
omega= gyro - b;  % bias 보정

rv= omega*dt;     % 회전벡터
dq= rotvec_to_quat(rv);
q_new= quat_multiply(q, dq);
q_new= normalizeQuat(q_new);

x_next= [q_new; b];
end

function z= measurementModel_9Axis(x, g0, m0)
% z=[acc; mag], acc= R(q)^T*[0,0,-g0], mag= R(q)^T*m0
q= x(1:4);
R= quat2rotm(q);

accBody= R'*[0;0;g0];
magBody= R'*m0;

z= [accBody; magBody];
end

function qm= weightedMeanState_Quat(Xsigma, w)
[dim, numPts]= size(Xsigma);
x_lin= zeros(dim,1);
for i=1:numPts
    x_lin= x_lin + w(i)*Xsigma(:,i);
end

% 쿼터니언 부분(1:4)은 나이브 선형합->정규화
qw=0; qx=0; qy=0; qz=0;
for i=1:numPts
    qw= qw + w(i)*Xsigma(1,i);
    qx= qx + w(i)*Xsigma(2,i);
    qy= qy + w(i)*Xsigma(3,i);
    qz= qz + w(i)*Xsigma(4,i);
end
q_approx= normalizeQuat([qw;qx;qy;qz]);

qm= x_lin;
qm(1:4)= q_approx;
end

function qn= normalizeQuat(q)
n= norm(q);
if n<1e-15
    qn= [1;0;0;0];
else
    qn= q/n;
end
end

function qm= quat_multiply(q1,q2)
w1=q1(1);x1=q1(2);y1=q1(3);z1=q1(4);
w2=q2(1);x2=q2(2);y2=q2(3);z2=q2(4);
w= w1*w2 - x1*x2 - y1*y2 - z1*z2;
x= w1*x2 + x1*w2 + y1*z2 - z1*y2;
y= w1*y2 - x1*z2 + y1*w2 + z1*x2;
z= w1*z2 + x1*y2 - y1*x2 + z1*w2;
qm=[w;x;y;z];
end

function R= quat2rotm(q)
w=q(1); x=q(2); y=q(3); z=q(4);
w2=w*w; x2=x*x; y2=y*y; z2=z*z;
R=[ w2+x2-y2-z2, 2*(x*y-w*z),   2*(w*y+x*z);
    2*(x*y+w*z), w2 - x2 + y2 - z2, 2*(y*z - w*x);
    2*(x*z-w*y), 2*(y*z+w*x), w2 - x2 - y2 + z2];
end

function q= rotvec_to_quat(rv)
angle= norm(rv);
if angle < 1e-12
    q=[1;0;0;0];
    return;
end
axis= rv/angle;
half= angle/2;
s= sin(half);
q= [cos(half); axis(1)*s; axis(2)*s; axis(3)*s];
end