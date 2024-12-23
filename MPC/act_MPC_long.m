close all
clc
clear all
load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat

my_cycle(1,:)=linspace(30,0,1001);
my_cycle(2,:)=[my_cycle(1,1:end-1)-my_cycle(1,2:end),0];
my_cycle(3,:)=3;

start=0;%Definisci l'istante iniziale
N=5; %Time horizon 
SOC_START=0.55; %initial soc values

%Initialization
SOC=SOC_START*ones(N,1);
u_n=1*ones(N,1);
N_rounds=3; %determine how many times to iterate on the same cycle
driving_cycle=ARTEMIS; %choose the driving cycle
N_it=length(driving_cycle)-1;

eltime = 0; %evaluate elapsed time
maxdur = zeros(1,N_it/10); %evaluate longest iter
tic
x=zeros(N_rounds,1); 
C=[eye(N);-eye(N)]; %define inequality constraint matrix
d=[-ones(N,1);-ones(N,1)]; %define the d vector for u>-1, u<1

% Define the desired options for myfmincon
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-6;
myoptions.tolfun    	=	1e-8;
myoptions.tolX       	=	1e-8;
myoptions.ls_beta       =	0.2;
myoptions.ls_c          =	.01;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	5e2;
myoptions.tolconstr     =	1e-3;
myoptions.xsequence     =	'on';
myoptions.display       ='none';

full_driving_cycle=[0;0;0;0];

pos=1;
% MPC
for k=1:N_rounds
     %Se si vuole cambiare ciclo tra un round e l'altro
     x(k)=rand;
     driving_cycle=pick_cycle(0.2);
     full_driving_cycle=[full_driving_cycle,driving_cycle];
     N_it=length(driving_cycle)-1;
    for j=1:N_it-N
        
        % Driving cycle extraction
        speed=driving_cycle(1,start+j:start+N-1+j);
        acceleration=driving_cycle(2,start+j:start+N-1+j);
        gear=driving_cycle(3,start+j:start+N-1+j);
        dislivello=driving_cycle(4,start+j:start+N-1+j);
        % Definition of the function handles
        StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
        myFullStateUpdate=@(u)my_full_horizon(u,SOC(2),StateUpdate);
        FullStateUpdate=@(u)full_horizon(u,SOC(2),StateUpdate);
        
        % Solve the optimization problem
        [u_n,~,~,exitf(j),~,~]=myfmincon(myFullStateUpdate,[u_n(2:end);u_n(end)],[],[],C,d,0*N,7*N,myoptions);
        
            
        % See if the problem is not solvable
        if exitf(j)<=0
            fprintf("All'iterazione %d l'exitflag è %d ",j,exitf(j))
            [u_n,~,~,exitf(j),~,~]=myfmincon(myFullStateUpdate,[u_n(2:end);u_n(end)],[],[],C,d,0*N,7*N,myoptions);
            if exitf(j)<=0
              fprintf("All'iterazione %d l'exitflag è ancora %d ",j,exitf(j))
            end
        end
        maxdur(1,j)=toc;
        if j>1
            maxdur(1,j)=maxdur(1,j)-sum(maxdur(1,1:j-1));
        end
        % Save the first control input U
        u(pos)=u_n(1);
        pos=pos+1;
        %Time plot
        if rem(j,N_it/10)==0
            md = max(maxdur);
            eltime = eltime+toc;
            int = N_it/10;
            fprintf("Siamo al: %d %% del giro %d \n In totale impiega: %f s \n Mediamente impiega: %f s ad iterazione \n" + ...
                "La più lunga impiega: %f s \n",j/(N_it)*100,k,toc,eltime/int,md)
            eltime = 0;
            maxdur=zeros(1,N_it/10);
            tic
        end
        
        % Update the SOC based on the current computed input
        [~,SOC]=FullStateUpdate(u_n);
    end

end
toc

%Plot the risultats using the found optimal control input U
speed=full_driving_cycle(1,:);
acceleration=full_driving_cycle(2,:);
gear=full_driving_cycle(3,:);
dislivello=full_driving_cycle(4,:);


StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
FullStateUpdate=@(u)full_horizon(u,SOC_START,StateUpdate);
[cost,SOC2plot,T_req,T_giv,mf,I_c,V_c,out]=FullStateUpdate(u);
[c,ceq]=nonlinconstr(u,SOC_START,StateUpdate);
tot_c=sum(sum(c<=0))/(4*length(c))
tot_ceq=sum(abs(ceq)<=myoptions.tolconstr)/length(ceq)
tot_mf=sum(mf)*1000
tot_cost = sum(cost)*1000
tot_soc_var=SOC_START-SOC(end)
stairs(u)
ylabel('u')
figure
hold on
num_segments = length(x); % Number of regions
segment_size = length(SOC) / num_segments; % Elements per segment
colors = [1 1 0; 1 0 0; 0 1 0]; % Define colors (red, green, blue)

% Plot the SOC data on top
plot(SOC2plot, 'k', 'LineWidth', 1);

% Additional plot settings
xlim([1, length(SOC2plot)]);
ylim([min(SOC2plot), max(SOC2plot)]);
xlabel('Time[s]');
ylabel('SOC [%]');
title('SOC');
box on;
hold off

stairs(speed)
ylabel('speed')
stairs(acceleration)
ylabel('acceleration')
stairs(T_giv)
hold on
stairs(T_req)
hold off
ylabel('comparison between Treq and Tgiven')
legend("Tgive","Treq")

