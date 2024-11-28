clc
close all
clear all

% load driving cycle
load ARTEMIS.mat
%load ARTEMIS_road.mat
%load WLTC.mat

%Choose the driving cycle 
drive_cycle = ARTEMIS;

N=length(drive_cycle(1,:));
speed_vector=drive_cycle(1,1:N);
acceleration_vector=drive_cycle(2,1:N);
gearnumber_vector=drive_cycle(3,1:N);

for i=1:N
    if(speed_vector(i)<=5/3.6)
        speed_vector(i)=5/3.6;
    end
end

%Driving cycles are defined without any slope
%If you want to define a slope, change the vector below 
road_slope = zeros(1,N); %rad
%SOC constraints
SOC_sup = 0.7;
SOC_inf = 0.4;
SOC_cons = 0.55;
% create grid
clear grd
Path = 0.01;
Nx = floor((SOC_sup-SOC_inf)/Path+1);
grd.Nx{1}    = Nx; 
grd.Xn{1}.hi = SOC_sup; 
grd.Xn{1}.lo = SOC_inf;
% set initial condition on the state
grd.X0{1} = SOC_cons;

% final state constraints
grd.XN{1}.hi = SOC_cons+0.01;
grd.XN{1}.lo = SOC_cons;

Inp_max = 1;
Inp_min = -5;
Nu = floor((Inp_max-Inp_min)/0.01+1);
%Input 
grd.Nu{1}    = Nu; 
grd.Un{1}.hi = Inp_max; 
grd.Un{1}.lo = Inp_min;	% Att: Lower bound may vary with engine size.

% define problem
clear prb
%input sequence
prb.W{1} = speed_vector;
prb.W{2} = acceleration_vector; 
prb.W{3} = gearnumber_vector; 
prb.W{4}=  road_slope;
%Sampling time definition for discretization
prb.Ts = 1;
prb.N  = N/prb.Ts;

% set options
options = dpm();
options.SaveMap=1;
options.MyInf = 1000;
options.BoundaryMethod = 'Line'; % also possible: 'none' or 'LevelSet';
if strcmp(options.BoundaryMethod,'Line') 
    %these options are only needed if 'Line' is used
    options.Iter = 5;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end
[res, dyn] = dpm(@hev,[],grd,prb,options);
%%
%PLOT SOC
t = 0:1:N;
SOC_extr = res.X{1};
SOC = SOC_extr;
figure
plot(t,SOC)
hold on
plot(t,SOC_cons*ones(N+1),"--k")
title("SOC")
xlabel("Time[s]")
ylabel("SOC [%]")
grid on
ylim([SOC_inf SOC_sup])
xlim([0 N])

%Plot control variable
t2 = 0:1:N-1;
xpl = linspace(0,N,N);
ypl = zeros(size(xpl));
x_fill = [xpl, fliplr(xpl)]; % combina x con la sua versione invertita per chiudere la forma
y_fill = -[ypl, 5*ones(size(ypl))];
y_fill2 = [ypl, 1*ones(size(ypl))];
figure
subplot(3,1,1)
f=fill(x_fill, y_fill,"green","FaceAlpha",0.4);
hold on
fill(x_fill, y_fill2, "red","FaceAlpha",0.4);
hold on 
grid on
plot(res.u)
xlabel("Time[s]")
ylabel("Torque split factor")
xlim([0 N])
ylim([-5,2])
title("Torque Split Ratio")
legend("Battery charge","Battery Discharge","U0")
subplot(3,1,2)
stairs(t2,speed_vector*3.6)
xlabel("Time[s]")
ylabel("Speed [Km/h]")
title("Driving cycle")
subplot(3,1,3)
stairs(t,SOC)
hold on
plot(t,SOC_cons*ones(N+1),"--k")
title("SOC")
xlabel("Time[s]")
ylabel("SOC")
grid on
ylim([SOC_inf SOC_sup])
xlim([0 N])
%%
%Plot fuel consumption
C_extr2 = res.Pe2(1,:);
cons2 =zeros(1,N);
for i=1:N-1
 cons2(i+1) = cons2(i)+C_extr2(i);
end
C = res.C{1};
total = zeros(1,N);
for i=1:N-1
    total(i+1)=total(i)+C(i);
end

figure
plot(t2,(cons2*N));
hold on
plot(t2,total*N)
hold on
saved_fuel = [total(1,N)*N cons2(1,N)*N];
final_t = [t2(N) t2(N)];
line(final_t,saved_fuel,"LineWidth",1.5)
legend("Consumption ICE Only","Consumption ICE+EM","Saved Fuel")
xlabel("Time[s]")
ylabel("Fuel consumption [g]")
title("Consumption comparison")
Fuel_Saved = 100-(total(1,N))/(cons2(1,N))*100;
fprintf('Fuel saved %4.2f%% \n',Fuel_Saved)

%%
figure
plot(res.UU)