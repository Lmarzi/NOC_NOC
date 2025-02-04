close all
clc
clear all

%load all the driving cycle data and the efficiency grids
load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat
load Results\LONG_FINAL.mat

%Define the SOC the car starts at
SOC_START=0.55;

%Decide the driving cycle, set to [] to pick a random one out of
%the three with a random downhill pattern each loop, to test for more
%complicated patterns. Otherwise set to ARTEMIS, ARTEMIS_road, or WLTC for
%standardised driving cycles.
driving_cycle_init=WLTC;%[tot_speed;tot_acceleration;tot_gear;tot_dislivello];

%Define how many cycles in a row to solve
N_rounds=1;

%Define which instant of the loop to start in 
start=0;

%Define the time horizon
N=1; 

%Initialise the first window of SOC
SOC=SOC_START*ones(N,1);

%Initialise the input of the first window
u_n=1*ones(N,1);

%Define the linear inequality constraints for myfmincon
C=[eye(N);-eye(N)];                     
d=[-ones(N,1);-ones(N,1)];          % -1 <= u <= 1

%Decide if you want the "global constraint" for MPC. 
%Set to 1 if wanted
globconstr = 1;


%Define the solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-6;
myoptions.tolfun    	=	1e-12;
myoptions.tolx       	=	1e-3;
myoptions.ls_beta       =	0.2;
myoptions.ls_c          =	.01;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e2;
myoptions.tolconstr     =	1e-3;
myoptions.xsequence     =	'on';
myoptions.display       =   'none';
myoptions.QPoptions.TolCon=myoptions.tolconstr;

%Initialise the vectors that change sides overtime
u=[];
exitf=[];
SOC_tot=[];
israndom=false;

%If it's empty set a flag value for later in the loop so we know we have to randomise it
if isempty(driving_cycle_init)
    israndom=true;

%If it's not standardise the driving cycle, specifying a zero downhill slope
else
    driving_cycle=driving_cycle_init;
    if length(driving_cycle(:,1))<4
        driving_cycle(4,1:length(driving_cycle))=zeros(1,length(driving_cycle));
    end
end

tic
eltime = 0;
failed=0;
pos_exitf=1;
pos_t=1;
%Starts the loop across the driving cycles, if N_rounds>=1
for k=1:N_rounds
    %To create the path it uses two numbers picked at random, that then get saved to allow 
    %for future simulations on the same exact path. To recreate it use recreate_path(x,y,N,0.25).
    % x picks the path, y picks the downhill pattern when present, 25% of
    % the time
    if israndom
        x(k)=rand;
        y(k)=rand;
        [driving_cycle,name]=pick_cycle(x(k),y(k),0.25);
        fprintf("The driving cycle %s has been picked, with a slope of %f %% \n",name,tan(min(driving_cycle(4,:)))*100)
    end
    %Start the MPC algorithm on the specific cycle
    N_it=length(driving_cycle)- start;
    for j=1:N_it-N
        
        %Extracts the vectors from the driving cycle matrix
        speed=driving_cycle(1,start+j:start+N-1+j);
        acceleration=driving_cycle(2,start+j:start+N-1+j);
        gear=driving_cycle(3,start+j:start+N-1+j);
        dislivello=driving_cycle(4,start+j:start+N-1+j);

        %Defines the handles to pass to myfmincon, myFullStateUpdate simulates 
        %along the entire horizon, StateUpdate simulates one step
        t(pos_t)=N_it*N_rounds-j-(k-1)*N_it;
        StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int,t(pos_t),globconstr);
        FullStateUpdate=@(u)my_full_horizon(u,SOC(1),StateUpdate);
        pos_t=pos_t+1;
        %Computes the optimal N seconds window. This is donegiving as an initial
        % guess what was computed the loop prior, this way the action will
        % be smoother and the optimisation will be quicker.
        % In case of abrupt changes it can fail in the first attempt, 
        %in that case it tries with u=0 in the last time step and later raises a warning.
        %This problem disappears with a low enough N.
        [u_n,~,~,exitf(pos_exitf)]=myfmincon(FullStateUpdate,[u_n(2:end);u_n(end)],[],[],C,d,0*N,8*N,myoptions);
        if exitf(pos_exitf)==-2
            fprintf("failed at iteration %d of cycle %d with exitflag %d\n",j,k,exitf(pos_exitf))
            [u_n,~,~,exitf_f]=myfmincon(FullStateUpdate,[u_n(1:end-1);0],[],[],C,d,0*N,8*N,myoptions);
            if exitf_f==-2
                failed=failed+1;
            end
        end        
        pos_exitf=pos_exitf+1;

        %Saves the u to simulate the results later
        u=[u,u_n(1)];
        
        %Every 10% it prints a loading message
        if rem(j,ceil(N_it/10))==0
            eltime = eltime+toc;
            int = ceil(N_it/10);
            fprintf("We are at: %d %% of round %d \n To compute this section in total it took: %f s \n On average it takes: %f s per iteration \n",round(j/(N_it)*100),k,toc,eltime/int)
            eltime = 0;
            tic
        end
        
        %Updates SOC for next optimisation window
        [~,SOC]=FullStateUpdate(u_n);      
    end
    eltime = eltime+toc;
    fprintf("We are at: %d %% of round %d \n To compute this section in total it took: %f s \n On average it takes: %f s per iteration \n",100,k,toc,eltime/int)
    
end

%%
%Simulate the input found, to compute overall performances.

%reload all the driving cycle data and the efficiency grids, sometimes it's
%useful to run this section on its own
load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat

%Standardise the driving cycle for the total input, useful if N_rounds>1
driving_cycle=[];

if isempty(driving_cycle_init)
    driving_cycle=recreate_path(x,y,N,start,0.25);
else
    for i=1:N_rounds
        driving_cycle=[driving_cycle,driving_cycle_init(:,start+1:end-N)];
    end
    driving_cycle(4,:)=zeros(1,length(driving_cycle));
end


%Prints if the first attempt ever failed to converge and had to retry with
%u=0, it can fail if u,x,y were reloaded
try
    fprintf("The first attempt failed %d times, it also failed with u=0 %d times\n",sum(exitf<=0),failed)
catch
    fprintf("Record of optimisation failure was not found")
end

%Extracts the total vectors
tot_speed=driving_cycle(1,:);
tot_acceleration=driving_cycle(2,:);
tot_gear=driving_cycle(3,:);
tot_dislivello=driving_cycle(4,:);

%Redefines the handles with the full vector
StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int,t(i),globconstr);
FullStateUpdate=@(u)my_full_horizon(u,SOC_START,StateUpdate);

%Simulates along the entire cycle
[vect,SOC2plot,mf]=FullStateUpdate(u);

%Writes total consumption on console, along with the total variation in SOC
tot_mf=sum(mf)*1000
tot_MPC = sum(vect(1,:))*1000
tot_soc_var=SOC_START-SOC2plot(end)

%Plots u
figure
stairs(u)
ylabel('u')
%Plots SOC overtime
figure
plot(SOC2plot, 'k', 'LineWidth', 1);
%compares with dp
hold on
%Computes dp result
res=dp_comp(driving_cycle,SOC_START,SOC2plot(end));
fprintf("Compared to dp, it consumes %f times as much\n",tot_mf/(sum(res.C{:}*1000))) 

%Computes SOC with dp and plots it
[~,SOC_dp]=FullStateUpdate(res.u(1:end-N));
plot(SOC_dp)
legend('SOC with MPC','SOC dp')
hold off
ylabel("SOC [ % ]")
xlabel("time [s]")

%Computes and plot total consumption overtime in the two cases
for i=1:length(tot_speed)-N*N_rounds
    tot_cons_MPC(i)=sum(mf(1:i));
    tot_cons_dp(i)=sum(res.C{1}(1:i));
end
figure
plot(tot_cons_MPC)
hold on
plot(tot_cons_dp)
hold off
legend('consumption with MPC','consumption with dp')
ylabel("mf [kg]")
xlabel("time [s]")
