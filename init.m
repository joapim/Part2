%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init()                                                                  %
%                                                                         %              
% Set initial parameters for part1.slx and part2.slx                      %
%                                                                         %
% Created:      2018.07.12	Jon Bjørnø  
% Modified:     2020.10.01  Group 1
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all

load('supply.mat');
load('supplyABC.mat');
load('thrusters_sup.mat')

%% INITIAL CONDITION
% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

%Simulation time
StartTime="0.0";
StopTime="1000";


%% CURRENT INPUT
Current_Speed = 0;              % m/s
Current_input_type = 0;         % 0 = constant heading, 1 = linear varied heading
Curent_Slaw_var = 0;            % 1 for heading slow variartion, 0 othewise

CurrHead.Slope = 0.01;          %[rad/s] slop of the linear varied heading
CurrHead.TimeStart = 300;       %[s] start at which the linear varied heading starts
% current heading given aaccording N-W coodinates from whee the current is
% flowing. An angle of pi() is added in the code to uniform it with the
% reference sstem
CurrHead.HeadStart = 0;         %[rad] initial heading value (alway to be defined, it 
                                % is used also for he case of "consat
                                % heading"
CurrHead.HeadEnd = pi/2;          %[rad] final heading value


% Current Heaing Slow variation input
Current_Head_mu = 0.001;
Current_Head_noise_power = 0.001;
Current_Head_Sample_time = 1;
Current_Head_Seed = 30000;



%% WIND INPUT
Wind_Heading_type = 0;          % 0 = No Slow variation for Heading, 1 = Slow variation for Heading
Wind_U_type = 0;                % 0 = No Slow variation for Wind Speed, 1 = Slow variation for Wind Speed
Wind_Gust_type = 0;             % 0 = No Wind Gust , 1 = Wind Gust

Wind_U10 = 0;                  % Wind speed at 10 m levation 
Wind_Head_mean = pi/2;          % Wind Angle defined as where the wind is coming from [rad]

k = 0.003;
z = 10;                         % [m]; wind calculation at a given elevaztion


U10 = Wind_U10;
z0 = 10*exp(-2/(5*sqrt(k)));
Umean = U10 * 5/2 * sqrt(k) * log(z/z0);


% Wind Heaing Slow variation input
Wind_Head_mu = 0.001;
Wind_Head_noise_power = 0.001;
Wind_Head_Sample_time = 1;
Wind_Head_Seed = 30000;


% Wind Magnitue Slow variation input
Wind_U_mean = Umean;
Wind_U_mu = 0.001;
Wind_U_noise_power = 0.005;
Wind_U_Sample_time = 1;
Wind_U_Seed = 500000;

% Win Gust input
Wind_gust_nfreq = 1000;
Wind_gust_L = 1800;             % [m]
Wind_gust_k = 0.0026;

Wind_gust_phi = 2*pi * rand(Wind_gust_nfreq,1);



%% CONTROLLER INPUT
Kp_surge = (2*pi/30)^2*(6.4794^5+6.362*10^6);   % based on natural period of abot 30 seconds from RAOs
Ki_surge = 18000;
Kd_surge = 2*0.7*sqrt(6.4794^5+6.362*10^6)*sqrt(Kp_surge);
% Kp_surge = 0;   
% Ki_surge = 0;
% Kd_surge = 0;


Kp_sway = (2*pi/20)^2*(2.157*10^6+6.362*10^6);        % based on natural period of 25 seconds from RAOs
Ki_sway = 80000;
Kd_sway = 2*0.7*sqrt(2.157*10^6+6.362*10^6)*sqrt(Kp_sway);
% Kp_sway = 0;       
% Ki_sway = 0;
% Kd_sway = 0;

Kp_yaw = (2*pi/10)^2*(1.0711*10^9+2.724*10^9);    % based on natural period of 10 seconds from RAOs
Ki_yaw = 1.5*10^8;
Kd_yaw = 2*0.7*sqrt(1.0711*10^9+2.724*10^9)*sqrt(Kp_yaw);
% Kp_yaw = 0;  
% Ki_yaw = 0;
% Kd_yaw = 0;



%% SET-POINTS INPUT
typeOfSimulation = 1; % 0=constant set-point , 1 sequence of set-points, 2 OPTSIM

%if constant set-point
ConstSPSuge = 10;  %m
ConstSPSway = 10;  %m
ConstSPYaw = 3/2*pi;   %rad

% if sequence of set-points
% Used to tune the controller
n0 = [0 0 0];
n1 = [50 0 0];
n2 = [50 -50 0];
n3 = [50 -50 -pi/4];
n4 = [0 -50 -pi/4];
n5 = [0 0 0];

nEnd = [0 0 0];
TimeStep = 300;       % seconds , time for which the system has to stay  steady
                      % it cannot be smaller than 100 seconds
TimeSteady = 200;     % seconds , time for which the system has to stay  steady

if typeOfSimulation==1
    %%% LINSIM
    setPoints = [n0;n1;n2;n3;n4;n5;nEnd];
    timeVector = [0;TimeStep;TimeStep;TimeStep;TimeStep;TimeStep;TimeStep];

    TimeTansition = TimeStep-100;    % seconds , transidition time between one set-point to the next


    % do not change anything from now on
    setPoints = [n0;n0;n1;n1;n2;n2;n3;n3;n4;n4;n5;n5;nEnd];

    timeVector = [0;TimeSteady;TimeTansition;TimeSteady;TimeTansition;TimeSteady;TimeTansition;...
        TimeSteady;TimeTansition;TimeSteady;TimeTansition;TimeSteady;TimeTansition];

    for idx =1:length(timeVector)
        timeSetPoints(idx)=sum(timeVector(1:idx));
    end   
    % RateOfChangeSuge = 10;  % m/s
    % RateOfChangeSway = 10;  % m/s
    % RateOfChangeYaw = 1;  % rad/s

    SurgeSP = [timeSetPoints' setPoints(:,1)];
    SwaySP = [timeSetPoints' setPoints(:,2)];
    YawSP = [timeSetPoints' setPoints(:,3)];

elseif typeOfSimulation==2
    %%% OPTSIM
    % Definitions of general parameters
    TimeTransition = TimeStep-TimeSteady;
    t = 0:0.1:TimeTransition; % Vector of discrete time instants [s]

    epsilon_L = 0.05; % Min. frac. of L to be trav. at max. (min.) vel.
    theta_a = 0.90; % Function switching threshold (0.6 <= theta_a < 1) Example 0.90
    theta_d = 0.90; % Function switching threshold (0 < theta_d <= 0.4) Example 0.15
    theta_0 = exp(-10); %Fcn. switching thr.(exp(??13)<=theta_0<=exp(??7)) Example exp(-10)

    V_d_tuning_paameter = 1.8;
    T_a_tuning_paameter = 1/8;
    T_d_tuning_paameter = 1/8;

    %%% CONSTUCTION OF REFERENCE MODEL 

    setPoints = [n0;n1;n2;n3;n4;n5;nEnd];
    timeSetPoints = [0;TimeStep;TimeStep;TimeStep;TimeStep;TimeStep;TimeStep];
    for j=1:length(timeSetPoints)
        timeVector(j,1)=sum(timeSetPoints(1:j));
    end

    SP = [timeVector setPoints];
    SurgeSP = [timeVector setPoints(:,1)];
    SwaySP = [timeVector setPoints(:,2)];
    YawSP = [timeVector setPoints(:,3)];

    vesselMotionId = {'SetPointSurge';'SetPointSway';'SetPointYaw'};
    for jdx=1:1:3
        % loop on the vessel motions
        eval([vesselMotionId{jdx} '= [SP(1,[1,' num2str(jdx+1) ']) 0 0];']);
        for idx = 2:1:size(SP,1)-1
            % loop on set-points

            Direction = sign(SP(idx,1+jdx)-SP(idx-1,1+jdx));
            % Definitions of general parameters
            V_d = abs(SP(idx,1+jdx)-SP(idx-1,1+jdx))/TimeTransition*V_d_tuning_paameter; % Desired cruise velocity [m/s]
            T_a = TimeTransition*T_a_tuning_paameter; % Desired minimum time to reach V_d [s]
            T_d = TimeTransition*T_d_tuning_paameter; % Desired minimum time to stop moving from V_d [s]

            if Direction == 0
                eval([vesselMotionId{jdx} ' = [' vesselMotionId{jdx} '; SP(idx,1), SP(idx,1+jdx), 0, 0];']);
            else
                L = abs(SP(idx,1+jdx)-SP(idx-1,1+jdx));
                t_0 = 0;
                [p,v,a] = constructPath(L,V_d,T_a,T_d,epsilon_L,theta_a,theta_d,theta_0,t,t_0);
                eval([vesselMotionId{jdx} ' = [' vesselMotionId{jdx} ';timeVector(idx)+t'', ' vesselMotionId{jdx} '(end,2)+p*Direction, ' vesselMotionId{jdx} '(end,3)+v*Direction, ' vesselMotionId{jdx} '(end,4)+a*Direction];']);
            end
        end

    end

    SurgeSP = SetPointSurge(:,1:2);
    SwaySP = SetPointSway(:,1:2);
    YawSP = SetPointYaw(:,1:2);
    
else
    SurgeSP = [0 0];
    SwaySP = [0 0];
    YawSP = [0 0];


end

%% SIMULATION

S = sim('part2_2017a','StartTime',StartTime,'StopTime',StopTime,'SimulationMode','normal');
%sim('part1')
SetPointPos = S.get('SetPointPos');
Eta = S.get('Eta');
SetPointSpeed = S.get('SetPointSpeed');
Nu = S.get('Nu');
Tau_Surge = S.get('Tau_Surge');
Tau_Sway = S.get('Tau_Sway');
Tau_Yaw = S.get('Tau_Yaw');
Wind_Speed = S.get('Wind_Speed');
Wind_Headind_EF = S.get('Wind_Headind_EF');
Wind_Force = S.get('Wind_Force');

%% Plot results

figure()
linkx(1)=subplot(3,1,1)
plot(SetPointPos.Time,SetPointPos.Data(:,1))
hold on
plot(Eta.Time,Eta.Data(:,1))
hold off
grid on
xlabel('Time [s]')
ylabel('X_E [m]')
legend('Set-point','Vessel')
title('Vessel position - Earth reference frame')
linkx(2)=subplot(3,1,2)
plot(SetPointPos.Time,SetPointPos.Data(:,2))
hold on
plot(Eta.Time,Eta.Data(:,2))
hold off
grid on
xlabel('Time [s]')
ylabel('Y_E [m]')
legend('Set-point','Vessel')
linkx(3)=subplot(3,1,3)
plot(SetPointPos.Time,SetPointPos.Data(:,3))
hold on
plot(Eta.Time,Eta.Data(:,3))
hold off
grid on
legend('Set-point','Vessel')
xlabel('Time [s]')
ylabel('Heading [rad]')

figure()
linkx(4)=subplot(3,1,1)
plot(SetPointSpeed.Time,SetPointSpeed.Data(:,1))
hold on
plot(Nu.Time,Nu.Data(:,1))
hold off
grid on
xlabel('Time [s]')
ylabel('Surge velocity [m/s]')
legend('Set-point','Vessel')
title('Vessel velocities - Earth reference frame')
linkx(5)=subplot(3,1,2)
plot(SetPointSpeed.Time,SetPointSpeed.Data(:,2))
hold on
plot(Nu.Time,Nu.Data(:,2))
hold off
grid on
xlabel('Time [s]')
ylabel('Sway velocity [m/s]')
legend('Set-point','Vessel')
linkx(6)=subplot(3,1,3)
plot(SetPointSpeed.Time,SetPointSpeed.Data(:,3))
hold on
plot(Nu.Time,Nu.Data(:,3))
hold off
grid on
legend('Set-point','Vessel')
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')


figure()
plot(SetPointPos.Data(:,2),SetPointPos.Data(:,1));
hold on
plot(Eta.Data(:,2),Eta.Data(:,1))
%plot(Eta.Data(:,1),Eta.Data(:,2))
hold off
grid on
legend('Set-point','Vessel')
ylabel('X_E [m] Earth reference frame')
xlabel('Y_E [m] Earth reference frame')
% xlim([min(SetPointPos.Data(:,1))-abs(max(SetPointPos.Data(:,1))*0.3) max(SetPointPos.Data(:,1))+abs(max(SetPointPos.Data(:,1))*0.3)])
% ylim([min(SetPointPos.Data(:,2))-abs(max(SetPointPos.Data(:,2))*0.3) max(SetPointPos.Data(:,2))+abs(max(SetPointPos.Data(:,2))*0.3)])
axis equal

figure()
linkx(7)=subplot(3,1,1)
plot(Tau_Surge.Time,Tau_Surge.Data(:,1));
hold on
plot(Tau_Surge.Time,Tau_Surge.Data(:,2));
plot(Tau_Surge.Time,Tau_Surge.Data(:,3));
hold off
grid on
title('Surge')
legend('\tau_P','\tau_D','\tau_I')
xlabel('Time [s]')
ylabel('\tau ')
linkx(8)=subplot(3,1,2)
plot(Tau_Sway.Time,Tau_Sway.Data(:,1));
hold on
plot(Tau_Sway.Time,Tau_Sway.Data(:,2));
plot(Tau_Sway.Time,Tau_Sway.Data(:,3));
hold off
grid on
title('Sway')
legend('\tau_P','\tau_D','\tau_I')
xlabel('Time [s]')
ylabel('\tau ')
linkx(9)=subplot(3,1,3)
plot(Tau_Yaw.Time,Tau_Yaw.Data(:,1));
hold on
plot(Tau_Yaw.Time,Tau_Yaw.Data(:,2));
plot(Tau_Yaw.Time,Tau_Yaw.Data(:,3));
hold off
grid on
title('Yaw')
legend('\tau_P','\tau_D','\tau_I')
xlabel('Time [s]')
ylabel('\tau ')
% xlim([min(SetPointPos.Data(:,1))-abs(max(SetPointPos.Data(:,1))*0.3) max(SetPointPos.Data(:,1))+abs(max(SetPointPos.Data(:,1))*0.3)])
% ylim([min(SetPointPos.Data(:,2))-abs(max(SetPointPos.Data(:,2))*0.3) max(SetPointPos.Data(:,2))+abs(max(SetPointPos.Data(:,2))*0.3)])




figure()
linkx(10)=subplot(2,5,1:3)
plot(Wind_Speed.Time,Wind_Speed.Data(:,1));
title('Wind Magnitude ')
xlabel('Time [s]')
ylabel('U_w [m/s] ')
grid on
linkx(11)=subplot(2,5,6:8)
plot(Wind_Headind_EF.Time,Wind_Headind_EF.Data(:,1)*180/pi);
title('Angle \alpha')
xlabel('Time [s]')
ylabel('Angle \alpha [deg] ')
grid on
subplot(2,5,[4,5,9,10])
h=polar(Wind_Headind_EF.Data(:,1),Wind_Speed.Data(:,1));
title('Wind magnitude - angle \alpha [deg]');


figure()
subplot(6,1,1)
plot(Wind_Force.Time,Wind_Force.Data(:,1));
title('F_x_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('F_x_w_i_n_d (body fixed ref frame) ')
grid on
subplot(6,1,2)
plot(Wind_Force.Time,Wind_Force.Data(:,2));
title('F_y_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('F_y_w_i_n_d (body fixed ref frame) ')
grid on
subplot(6,1,3)
plot(Wind_Force.Time,Wind_Force.Data(:,3));
title('F_z_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('F_z_w_i_n_d (body fixed ref frame) ')
grid on
subplot(6,1,4)
plot(Wind_Force.Time,Wind_Force.Data(:,4));
title('M_x_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('M_x_w_i_n_d (body fixed ref frame) ')
grid on
subplot(6,1,5)
plot(Wind_Force.Time,Wind_Force.Data(:,5));
title('M_y_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('M_y_w_i_n_d (body fixed ref frame) ')
grid on
subplot(6,1,6)
plot(Wind_Force.Time,Wind_Force.Data(:,6));
title('M_z_w_i_n_d (body fixed ref frame)')
xlabel('Time [s]')
ylabel('M_z_w_i_n_d (body fixed ref frame) ')
grid on




linkaxes(linkx,'x');



%% FUNCTIONS


function [p,v,a] = constructPath(L,V_d,T_a,T_d,epsilon_L,theta_a,theta_d,theta_0,t,t_0)


% ====================== PRE-COMPUTATIONS =======================
r_T = T_a/T_d; % Time ratio

xi_a = 15; % Ratio T_1/tau_11 (10 <= xi_a <= 15)
kappa_a = (theta_a^2)*((xi_a - exp(-xi_a))^2)/(2*((xi_a - 1)^2)) ...
- (theta_a^2)/(xi_a -1) - ((1 - theta_a)^2)*(1 - theta_0)- ...
(1 - theta_a)*log(theta_0); % Auxiliary constant

xi_d = 15; % Ratio T_4/tau_31 (10 <= xi_d <= 15)
kappa_d = (2*(1 - theta_d)*(xi_d - 1)*(xi_d - exp(-xi_d)) - ...
((1 - theta_d)^2)*(((xi_d - exp(-xi_d))^2) - 2*(xi_d - 1)))/ ...
(2*((xi_d - 1)^2)) + (1 - theta_0)*(theta_d^2); % Aux. constant

auxVar = L*(1 - min([0.1 epsilon_L]))/(kappa_a*r_T + kappa_d);
v_c = sqrt(auxVar*abs(V_d)/T_d);%Cand. abs. val. for the cruise vel.
v_m = sign(V_d)*min([v_c abs(V_d)]); % Max. (min.), or cruise, vel.

t_d = T_d*v_m/V_d; % Adj. min. time to stop from the cruise velocity
t_a = t_d*r_T; % Adjusted min. time to reach the cruise velocity

a_m = v_m/t_a; % Maximum (minimum) acceleration
d_m = -v_m/t_d; % Maximum (minimum) deceleration

tau_11 = theta_a*t_a/(xi_a - 1); % Time constant tau_11
tau_12 = (1 - theta_a)*t_a; % Time constant tau_12
tau_31 = (1 - theta_d)*t_d/(xi_d - 1); % Time constant tau_31
tau_32 = theta_d*t_d; % Time constant tau_32

T_1 = tau_11*xi_a; % Auxiliary time instant T_1
T_2 = -tau_12*log(theta_0); % Auxiliary time instant T_2
T_3 = abs(L/v_m) - (kappa_a*t_a + kappa_d*t_d); %Aux. time inst. T_3
T_4 = tau_31*xi_d; % Auxiliary time instant T_4
T_5 = -tau_32*log(theta_0); % Auxiliary time instant T_5

t_1 = T_1 + t_0; % Function switching time instant t_1
t_2 = T_2 + t_1; % Function switching time instant t_2
t_3 = T_3 + t_2; % Function switching time instant t_3
t_4 = T_4 + t_3; % Function switching time instant t_4
t_5 = T_5 + t_4; % Function switching time instant t_5

P_1 = a_m*((T_1^2)/2 - tau_11*T_1 + (tau_11^2)*(1 - exp(-xi_a)));
P_2 = P_1 + v_m*(T_2 - (1 - theta_a)*tau_12*(1 - theta_0)); % p(T_2)
P_3 = P_2 + v_m*T_3; % Position p(T_3)
P_4 = P_3 + v_m*T_4 + d_m*((T_4^2)/2 - tau_31*T_4 + ...
(tau_31^2)*(1 - exp(-xi_d))); % Position p(T_4)

% ==================== REFERENCE GENERATION =====================
% Memory space pre??allocation (recommended for expedited runs)
a = zeros(numel(t), 1); % a = a(t) = acceleration reference
v = zeros(numel(t), 1); % v = v(t) = velocity reference
p = zeros(numel(t), 1); % p = p(t) = position reference

for i = 1:numel(t)
    if t(i) >= t_5 % 4th phase
        a(i) = 0;
        v(i) = 0;
        p(i) = sign(v_m)*L;
    elseif t(i) >= t_4 % 3rd phase ?? 2nd subphase
        curTime = t(i) - t_4;
        f_32 = exp(-curTime/tau_32);
        a(i) = d_m*f_32;
        v(i) = v_m*theta_d*f_32;
        p(i) = P_4 + v_m*theta_d*tau_32*(1 - f_32);
    elseif t(i) >= t_3 % 3rd phase ?? 1st subphase
        curTime = t(i) - t_3;
        f_31 = 1 - exp(-curTime/tau_31);
        a(i) = d_m*f_31;
        v(i) = v_m + d_m*(curTime - tau_31*f_31);
        p(i) = P_3 + v_m*curTime + d_m*(0.5*(curTime^2) - ...
        tau_31*curTime + (tau_31^2)*f_31);
    elseif t(i) >= t_2 % 2nd phase
        curTime = t(i) - t_2;
        a(i) = 0;
        v(i) = v_m;
        p(i) = P_2 + v_m*curTime;
    elseif t(i) >= t_1 % 1st phase ?? 2nd subphase
        curTime = t(i) - t_1;
        f_12 = exp(-curTime/tau_12);
        a(i) = a_m*f_12;
        v(i) = v_m*(1 - (1 - theta_a)*f_12);
        p(i) = P_1 + ...
        v_m*(curTime - (1 - theta_a)*tau_12*(1 - f_12));
    elseif t(i) >= t_0 % 1st phase ?? 1st subphase
        curTime = t(i) - t_0;
        f_11 = 1- exp(-curTime/tau_11);
        a(i) = a_m*f_11;
        v(i) = a_m*(curTime - tau_11*f_11);
        p(i) = a_m*(0.5*(curTime^2) - tau_11*curTime + ...
        (tau_11^2)*f_11);
    else % Before the 1st phase starts
        a(i) = 0;
        v(i) = 0;
        p(i) = 0;
    end
end     

end