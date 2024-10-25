clc
close all
clear all

% load driving cycle
load Artemis.mat
load ARTEMIS_road.mat
load WLTC.mat

%Choose the driving cycle 
drive_cycle = ARTEMIS_road;
N=length(drive_cycle(1,:));
speed_vector=drive_cycle(1,1:N);
acceleration_vector=drive_cycle(2,1:N);
gearnumber_vector=drive_cycle(3,1:N);
SOC_sup = 0.7;
SOC_inf = 0.4;
SOC_cons = 0.55;
% create grid
clear grd
grd.Nx{1}    = 55; 
grd.Xn{1}.hi = SOC_sup; 
grd.Xn{1}.lo = SOC_inf;

grd.Nu{1}    = 25; 
grd.Un{1}.hi = 1; 
grd.Un{1}.lo = -1;	% Att: Lower bound may vary with engine size.

% set initial condition on the state
grd.X0{1} = SOC_cons;

% final state constraints
grd.XN{1}.hi = SOC_cons+0.01;
grd.XN{1}.lo = SOC_cons;

% define problem
clear prb
%input sequence
prb.W{1} = speed_vector;
prb.W{2} = acceleration_vector; 
prb.W{3} = gearnumber_vector; 
%Sampling time definition for discretization
prb.Ts = 1;
prb.N  = (N-1)*1/prb.Ts + 1;

% set options
options = dpm();
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
U0_extr = dyn.Uo(:,:);
U0 = cell2mat(U0_extr);
t2 = 0:1:N-1;
xpl = linspace(0,N,N);
ypl = zeros(size(xpl));
x_fill = [xpl, fliplr(xpl)]; % combina x con la sua versione invertita per chiudere la forma
y_fill = -[ypl, 1*ones(size(ypl))];
y_fill2 = [ypl, 1*ones(size(ypl))];
figure
subplot(2,1,1)
f=fill(x_fill, y_fill,"green","FaceAlpha",0.4);
hold on
fill(x_fill, y_fill2, "red","FaceAlpha",0.4);
hold on
plot(t2,U0(1,:),"k","LineWidth",1)
grid on
xlabel("Time[s]")
ylabel("U0")
xlim([0 N])
ylim([-2,2])
title("Torque Split Ratio")
legend("Regenerative Braking","Mix","U0")
subplot(2,1,2)
plot(t2,speed_vector*3.6)
xlabel("Time[s]")
ylabel("Speed [Km/h]")
title("Driving cycle:")

%Plot fuel consumption
C_extr = res.Pe(1,:);
cons =zeros(1,N);
for i=1:N-1
 cons(i+1) = cons(i)+C_extr(i);
end
C_extr2 = res.Pe2(1,:);
cons2 =zeros(1,N);
for i=1:N-1
 cons2(i+1) = cons2(i)+C_extr2(i);
end
figure
plot(t2,(cons2*N/43400000));
hold on
plot(t2,(cons*N/43400000));
legend("Consumption ICE Only","Consumption ICE+EM")
title("Consumption comparison")


