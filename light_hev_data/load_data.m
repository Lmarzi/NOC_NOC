%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HEV (FORD FIESTA LIKE) PARAMETERS %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% last update: 21-Oct-2024
% authors: Stefano Radrizzani, Lorenzo Brecciaroli, Giulio Panzani
% last modification author: Stefano Radrizzani
% contact: stefano.radrizzani@polimi.it

% References
% [1] 
% Radrizzani,S., Brecciaroli,L., Panzani,G., Savaresi,S.M. (2022)
% An efficiency based approach for the Energy Management in HEVs
% In: IFAC PapersOnLine 55-24 (2022) 167–172.

% [2] 
% Lee,B.,Lee,S.,Cherry,J.,Neam,A.,Sanchez,J.,and Nam,E. (2013)
% Development of Advanced Light-Duty Powertrain and Hybrid Analysis Tool.
% In: SAE 2013 World Congress & Exhibition, SAEInternational.

%% egine and motor
load('motor_engine_data')
% engine
%   maximum torque values [Nm] and brakepoints [rad/s]
%       full_throttle_torque_Nm
%       full_throttle_speed_radps
%   fuel consumption [g/s] and efficiency [-] maps and brakepoints [rad/s],[Nm]
%       fuel_map_speed_radps
%       fuel_map_torque_Nm
%       fuel_map_gps
%       efficiency_map

% electric motor
%   maximum and torque values [Nm] and brakepoints [rad/s]
%       max_torque_speed_radps
%       max_torque_torque_Nm
%       max_torque_torque_Nm
%   power consumption [W] map and brakepoints [rad/s],[Nm]
%       battery_power_map_speed_radps
%       battery_power_map_torque_Nm
%       battery_power_map_W
%       efficiency to be computed accordigly

%% vehicle
% vehicle mass [kg]
vehicle.M_kg = 1200;
% wheel radius [m]
vehicle.Rw_m = 0.281154;

% gear box (from motor to main axle) 1st to 5th gear [-]
vehicle.tau_gb_ = [3.417 1.958 1.276 0.943 0.757];

% final drive ratio (from main axle to wheel) [-]
vehicle.tau0_ = 3.1725494;

% costing down force [N] coefficients
% as a function of the longiditudinal speed [m/s]
vehicle.A_cd_N = 93.086154;  % [N]
vehicle.B_cd_Nspm = 2.5373490;  % [N/(m/s)]
vehicle.C_cd_Ns2pm2 = 0.38382359; % [N/(m/s)^2]

%% fuel
% fuel density [MJ/kg]
fuel.lambda_MKpkg = 43.308;

%% battery
% battery capacity [kWh]
battery.capacity_kWh = 4.5; 
%%
% Creare una griglia di velocità e coppia ICE
[Vel, Copp] = meshgrid(engine.fuel_map_speed_radps*(60/(2*pi)),engine.fuel_map_torque_Nm);

figure;
vect_eff=[0.05,0.15,0.20,0.25,0.29,0.33,0.35,0.37];
contour(Vel, Copp, engine.efficiency_map, vect_eff,'ShowText', 'on'); % Con testo dei livelli
colorbar; % Barra dei colori per rappresentare i valori di efficienza
xlabel('Speed [rpm]');
ylabel('Torque [Nm]');
title('Efficiency Contour Map');
hold on;
plot(engine.full_throttle_speed_radps*(60/(2*pi)),engine.full_throttle_torque_Nm)
xlim([500,6000])
ylim([0,200])

%% Creare una griglia di velocità e coppia EM
[torque_grid, speed_grid]=meshgrid(motor.battery_power_map_torque_Nm,motor.battery_power_map_speed_radps.*(60/(2*pi)));
figure
contourf(speed_grid, torque_grid,motor.battery_power_map_W, 'ShowText', 'on'); % Con testo dei livelli
colorbar; % Barra dei colori per rappresentare i valori di efficienza
xlabel('Speed [rpm]');
ylabel('Torque [Nm]');
title('Power Contour Map');

mech_power_map = torque_grid.*speed_grid.*((2*pi)/60);
efficiency_map = motor.battery_power_map_W./mech_power_map;
corr_eff = efficiency_map(:,1:25);
corr_eff(:,26)=ones(73,1);
test = corr_eff;
for i = 27:51
    j = 52-i;
    corr_eff(:,i)=efficiency_map(:,j);
end
test2 = corr_eff(:,26:end);
[torque_2,speed_2]=meshgrid(motor.battery_power_map_torque_Nm, motor.battery_power_map_speed_radps.*(60/(2*pi)));
figure;
livelli_eff = [0.85,0.88,0.91,0.93,0.94];
contour(speed_2,torque_2,corr_eff,livelli_eff, 'ShowText', 'on'); % Con testo dei livelli
xlabel('Speed [rpm]');
ylabel('Torque [Nm]');
title('Efficiency Contour Map');
hold on
plot(speed_2,motor.max_torque_torque_Nm,"k","LineWidth",2);
hold on
plot(speed_2,motor.min_torque_torque_Nm,"k","LineWidth",2);
ylim([-50 100])
xlim([600 4000])


[torque_2,speed_2]=meshgrid(motor.battery_power_map_torque_Nm(:,26:end), motor.battery_power_map_speed_radps.*(60/(2*pi)));
figure;
livelli_eff = [0.85,0.88,0.91,0.94];
contour(speed_2,torque_2,test2,livelli_eff, 'ShowText', 'on'); % Con testo dei livelli
xlabel('Speed [rpm]');
ylabel('Torque [Nm]');
title('Efficiency Contour Map');
hold on
plot(speed_2,motor.max_torque_torque_Nm,"k","LineWidth",2);
hold on
plot(speed_2,motor.min_torque_torque_Nm,"k","LineWidth",2);
ylim([-70 120])
xlim([0 6000])
%%
eth1 = interp2(motor.battery_power_map_torque_Nm(:,26:end), motor.battery_power_map_speed_radps,test2,0,156)