close all
clc
clear all

%load all the driving cycle data and the efficiency grids
load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat

%Define the SOC the car starts at
SOC_START=0.55;

%Decide the driving cycle, set to [] to pick a random one out of
%the three with a random downhill pattern each loop, to test for more
%complicated patterns. Otherwise set to ARTEMIS, ARTEMIS_road, or WLTC for
%standardised driving cycles.
driving_cycle_init=[];

%Define which instant of the loop to start in 
start=0;

%Define the time horizon
N=1; 

%Initialise the input of the first window
u_n=1*ones(N,1);

%Define the linear constraints for myfmincon,
C=[eye(N);-eye(N)];                     
d=[-ones(N,1);-ones(N,1)];          % -1 <= u <= 1

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

%If it's empty set a flag value for later in the loop ro randomise it
if isempty(driving_cycle_init)
    israndom=true;

%If it's not standardise the driving cycle, specifying a zero downhill slope
else
    driving_cycle=driving_cycle_init;
    driving_cycle(4,1:length(driving_cycle))=zeros(1,length(driving_cycle));
end

eltime = 0;
failed=0;
pos_exitf=1;
%Starts the loop across the driving cycles, if N_rounds>=1
for h=1:100
    u=[];
    x=[];
    y=[];
    SOC=SOC_START*ones(N,1);
    N_rounds(h)=min(max(round(4*(randn+1)),1),10);
    N_rounds(h)=N_rounds(h)
    h=h
    disl_prob(h)=0.5*rand;
    tic
    for k=1:N_rounds(h)
        %To create the path it uses two numbers picked at random, that then get saved to allow 
        %for future simulations on the same exact path. To recreate it use recreate_path(x,y,N,0.25).
        % x picks the path, y picks the downhill pattern that will only be present 25% of the time
        if israndom
            x(k)=rand;
            y(k)=rand;
            [driving_cycle,name]=pick_cycle(x(k),y(k),disl_prob);
        end
        
        %Start the MPC algorithm on the specific cycle
        N_it=length(driving_cycle)- start- 1;
        for j=1:N_it-N
            
            %Extracts the vectors from the driving cycle matrix
            speed=driving_cycle(1,start+j:start+N-1+j);
            acceleration=driving_cycle(2,start+j:start+N-1+j);
            gear=driving_cycle(3,start+j:start+N-1+j);
            dislivello=driving_cycle(4,start+j:start+N-1+j);
    
            %Defines the handles to pass to myfmincon, myFullStateUpdate simulates 
            %along the entire horizon, StateUpdate simulates one step
            StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
            FullStateUpdate=@(u)my_full_horizon(u,SOC(1),StateUpdate);
            
            %Computes the optimal N seconds window. This is donegiving as an initial
            % guess what was computed the loop prior, this way the action will
            % be smoother and the optimisation will be quicker.
            % In case of abrupt changes it can fail in the first attempt, 
            %in that case it tries with u=0 in the last time step and later raises a warning.
            %This problem disappears with a low enough N.
            [u_n,~,~,exitf(pos_exitf),~,~]=myfmincon(FullStateUpdate,[u_n(2:end);u_n(end)],[],[],C,d,0*N,8*N,myoptions);
            if exitf(pos_exitf)<=0
                fprintf("failed at iteration %d of cycle %d with exitflag %d\n",j,k,exitf(pos_exitf))
                [u_n,~,~,exitf_f,~,~]=myfmincon(FullStateUpdate,[u_n(1:end-1);0],[],[],C,d,0*N,8*N,myoptions);
                if exitf_f<=0
                    failed=failed+1;
                end
            end        
            pos_exitf=pos_exitf+1;
    
            %Saves the u to simulate the results later
            u=[u,u_n(1)];

            %Updates SOC for next optimisation window
            [~,SOC]=FullStateUpdate(u_n);      
        end
    end
    toc
    %%
    %Simulate the input found, to compute overall performances. If the u has
    %already been computed this part can be ran on its own, defining the
    %driving cycle like before
    
    driving_cycle=[];
    
    if isempty(driving_cycle_init)
        driving_cycle=recreate_path(x,y,N,start,disl_prob(h));
    else
        for i=1:N_rounds
            driving_cycle=[driving_cycle,driving_cycle_init(:,start+1:end-N)];
        end
        driving_cycle(4,:)=zeros(1,length(driving_cycle));
    end
    
    %Extracts the total vectors
    tot_speed=driving_cycle(1,:);
    tot_acceleration=driving_cycle(2,:);
    tot_gear=driving_cycle(3,:);
    tot_dislivello=driving_cycle(4,:);
    
    %Redefines the handles with the full vector
    StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
    FullStateUpdate=@(u)my_full_horizon(u,SOC_START,StateUpdate);
    
    %Simulates along the entire cycle
    [~,SOC2plot,mf]=FullStateUpdate(u);
    
    %Writes total consumption on console, along with the total variation in SOC
    tot_mf(h)=sum(mf)*1000;
    res(h)=dp_comp(driving_cycle,SOC_START,SOC2plot(end));
    rapp(h)=tot_mf(h)/(sum(res(h).C{:}*1000));
    drive_cycle{h}=driving_cycle;
    u_back{h}=u;
end