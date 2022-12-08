function [] = ekf_localization()
 
% Homework for ekf localization
% Modified by YH on 09/09/2019, thanks to the original open source
% Any questions please contact: zjuyinhuan@gmail.com

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PxEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
    %-------------------------------------------------
    convQ=noiseQ.^2;
    % convQ=eye(3);
    % convQ=noiseQ;
    %-------------------------------------------------

    % Covariance Matrix for observation
    %-------------------------------------------------
    convR=noiseR.^2;
    % convR=eye(3);
    % convR=noiseR;
    %-------------------------------------------------

    % Other Intial
    %-------------------------------------------------
    G = zeros(3);
    H = zeros(3);
    %-------------------------------------------------

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        %------------------------------
        xPred =  doMotion(xEkf, u); % 利用运动模型求得预测置信分布的均值
        G = jacobF(xEkf, u); % 计算雅可比矩阵
        PxEkfPred = G * PxEkf * G' + convQ;  % 计算协方差预测值
        %------------------------------

        % Update
        % -----------------------------
        H = jacobH(xPred);  % 计算雅克比矩阵
        K = PxEkfPred*H'*inv(H*PxEkfPred*H'+convR);  % 计算卡尔曼增益
        xEkf = xPred + K * (doObservation(z)-doObservation(xPred)); % 置信度分布的均值更新
        PxEkf=(eye(3)-K*H)*PxEkfPred; % 置信度分布的协方差更新
        % -----------------------------

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    % -----------------------------
    % DWA模型
    v = sqrt(u(1)^2+u(2)^2);
    w = u(3);
    dx = -v/w*sin(x(3))+v/w*sin(x(3)+w*dt);
    dy = v/w*cos(x(3))-v/w*cos(x(3)+w*dt);
    dtheta = w*dt;
    x(1) = x(1)+dx;
    x(2) = x(2)+dy;
    x(3) = x(3)+dtheta;
    % -----------------------------
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    % -----------------------------
    v = sqrt(u(1)^2+u(2)^2);
    w = u(3);
    jF13 = -v/w*cos(x(3))+v/w*cos(x(3)+w*dt);
    jF23 = -v/w*sin(x(3))+v/w*sin(x(3)+w*dt);
    jF=[1 0 jF13;0 1 jF23;0 0 1];
%     syms xt yt theta;
%     a = [xt yt theta];
%     f = [xt-v/w*sin(theta)+v/w*sin(theta+w*dt);
%          yt+v/w*cos(theta)-v/w*cos(theta+w*dt);
%          theta+w*dt];
%     jac = jacobian(f,a);
%     jF = eval(subs(jac,a,x'));
    % -----------------------------
end

%Observation Model
function x = doObservation(z)
    % -----------------------------
    x = z;
    % -----------------------------
 end

%Jacobian of Observation Model
function jH = jacobH(x)
    % -----------------------------
    jH = eye(3);
    % -----------------------------
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % -----------------------------
    % calculate error
    disp('GPS误差：');
    GPS_error_dist_vector = sqrt((estimation.GPS(:,1)-estimation.xTruth(:,1)).^2 +(estimation.GPS(:,2)-estimation.xTruth(:,2)).^2);
    GPS_error=mean(GPS_error_dist_vector);
    disp(['距离平均绝对误差：',num2str(GPS_error)]);
    GPS_error_angle_vector = abs(estimation.GPS(:,3)-estimation.xTruth(:,3));
    GPS_error_angle = mean(GPS_error_angle_vector);
    disp(['角度平均绝对误差：',num2str(GPS_error_angle/pi*180),'°']);   
    disp(' ');
    disp('纯里程计误差：');
    Odom_error_dist_vector = sqrt((estimation.xOdom(:,1)-estimation.xTruth(:,1)).^2 +(estimation.xOdom(:,2)-estimation.xTruth(:,2)).^2);
    Odom_error=mean(Odom_error_dist_vector);
    disp(['距离平均绝对误差：',num2str(Odom_error)]);
    Odom_error_angle_vector = abs(estimation.xOdom(:,3)-estimation.xTruth(:,3));
    Odom_error_angle = mean(Odom_error_angle_vector);
    disp(['角度平均绝对误差：',num2str(Odom_error_angle/pi*180),'°']);   
    disp(' ');
    disp('EKF定位误差：');
    EKF_error_dist_vector = sqrt((estimation.xEkf(:,1)-estimation.xTruth(:,1)).^2 +(estimation.xEkf(:,2)-estimation.xTruth(:,2)).^2);
    EKF_error = mean(EKF_error_dist_vector);
    disp(['距离平均绝对误差：',num2str(EKF_error)]);
    EKF_error_angle_vector = abs(estimation.xEkf(:,3)-estimation.xTruth(:,3));
    EKF_error_angle=mean(EKF_error_angle_vector);
    disp(['角度平均绝对误差：',num2str(EKF_error_angle/pi*180),'°']);
    
    figure;
    % plot distance error
    subplot(2,1,1);
    plot(GPS_error_dist_vector,'g','Linewidth',1.2);
    hold on;
    plot(Odom_error_dist_vector,'r','Linewidth',1.2);
    hold on;
    plot(EKF_error_dist_vector,'b','Linewidth',1.1);
    title('距离误差');
    legend('GPS','纯里程计','EKF定位');
    xlabel('index', 'fontsize', 12);
    ylabel('distance error(m)', 'fontsize', 12);
    grid on;
    
    % plot angle error
    subplot(2,1,2);
    plot(GPS_error_angle_vector,'g','Linewidth',1.2);
    hold on;
    plot(Odom_error_angle_vector,'r','Linewidth',1.2);
    hold on;
    plot(EKF_error_angle_vector,'b','Linewidth',1.1);
    title('角度误差');
    legend('GPS','纯里程计','EKF定位');
    xlabel('index', 'fontsize', 12);
    ylabel('angle error(°)', 'fontsize', 12);
    grid on;
    % -----------------------------
end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end