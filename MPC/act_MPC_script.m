close all
clc
clear all
load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat


start=0;%Definisci l'istante iniziale
N=5; %Time horizon ->compare sol with 5
SOC_START=0.52;
N_rounds=100;

struct_eff=load("eff_interpol.mat");

etam_int=struct_eff.etam_int;
etaeng_int=struct_eff.etaeng_int;
Tmmax_int=struct_eff.Tmmax_int;
Tmmin_int=struct_eff.Tmmin_int;
Temax_int=struct_eff.Temax_int;


SOC=SOC_START*ones(N,1);
u_n=1*ones(N,1);

eltime = 0;
tic
x=zeros(N_rounds,1);
C=[eye(N);-eye(N)];
d=[-ones(N,1);-ones(N,1)];

myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-6;
myoptions.tolfun    	=	1e-12;
myoptions.tolX       	=	1e-12;
myoptions.ls_beta       =	0.2;
myoptions.ls_c          =	.01;
myoptions.ls_nitermax   =	1e2;
myoptions.nitermax      =	1e2;
myoptions.tolconstr     =	1e-3;
myoptions.xsequence     =	'on';
myoptions.display       =   'none';
        
tot_speed=[];
tot_acceleration=[];
tot_gear=[];
tot_dislivello=[];

pos=1;
m=1;
u=[];
exitf=[];
for k=1:N_rounds
    %Se si vuole cambiare ciclo tra un round e l'altro
    x(k)=rand;
    y(k)=rand;
    [driving_cycle,name]=pick_cycle(x(k),y(k));
    fprintf("È stato scelto il ciclo %s, il SOC è %f",name, SOC(end))
    fprintf("\n")
    tot_speed=[tot_speed,driving_cycle(1,1:end-N)];
    tot_acceleration=[tot_acceleration,driving_cycle(2,1:end-N)];
    tot_gear=[tot_gear,driving_cycle(3,1:end-N)];
    tot_dislivello=[tot_dislivello,driving_cycle(4,1:end-N)];
    N_it=length(driving_cycle)-1;
    for j=1:N_it-N
        %Estrai i sottovettori e ridefinisci gli handle per i sottovettori.
        %Stavolta la funzione di costo è calcolata tutta in una volta. Bisogna
        %rivedere la funzione di costo bene
       
        speed=driving_cycle(1,start+j:start+N-1+j);
        acceleration=driving_cycle(2,start+j:start+N-1+j);
        gear=driving_cycle(3,start+j:start+N-1+j);
        dislivello=driving_cycle(4,start+j:start+N-1+j);

        StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
        myFullStateUpdate=@(u)my_full_horizon(u,SOC(2),StateUpdate);
        FullStateUpdate=@(u)full_horizon(u,SOC(2),StateUpdate);
        
        %calcoli la u ottima sull'orizzonte dei 5 secondi
        [u_n,~,~,exitf_i,~,~]=myfmincon(myFullStateUpdate,[u_n(2:end);u_n(end)],[],[],C,d,0*N,8*N,myoptions);
        exitf=[exitf,exitf_i];
        if exitf(j)<=0
            [u_n,~,~,exitf(j),~,~]=myfmincon(myFullStateUpdate,[u_n(2:end);0.4],[],[],C,d,0*N,8*N,myoptions);
        end

        %salvi la u per poi vedere il risultato complessivo
        u=[u,u_n(1)];
        pos=pos+1;
        if exitf(j)<=0
            fprintf("All'iterazione %d l'exitflag è %d ",j,exitf(j))
        end
        if rem(j,ceil(N_it/10))==0
            eltime = eltime+toc;
            int = ceil(N_it/10);
            m=m+1;
            fprintf("Siamo al: %d %% del giro %d \n In totale impiega: %f s \n Mediamente impiega: %f s ad iterazione \n",j/(N_it)*100,k,toc,eltime/int)
            eltime = 0;
            tic
        end
    
        [~,SOC]=FullStateUpdate(u_n);
    end
end
toc
%%
%Vedi i risultati utilizzando la u trovata nell'intero orizzonte
tot_speed=[];
tot_acceleration=[];
tot_gear=[];
tot_dislivello=[];
for i=1:27
    driving_cycle=pick_cycle(x(i),y(i));
    tot_speed=[tot_speed,driving_cycle(1,1:end-N-1)];
    tot_acceleration=[tot_acceleration,driving_cycle(2,1:end-N-1)];
    tot_gear=[tot_gear,driving_cycle(3,1:end-N-1)];
    tot_dislivello=[tot_dislivello,driving_cycle(4,1:end-N-1)];
end

StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
FullStateUpdate=@(u)full_horizon(u,SOC_START,StateUpdate);

[cost,SOC2plot,T_req,T_giv,mf,I_c,V_c]=FullStateUpdate(u);
tot_mf=sum(mf)*1000
tot_soc_var=SOC_START-SOC2plot(end)
figure
stairs(u)
ylabel('u')

% Plot the SOC data on top
figure
plot(SOC2plot, 'k', 'LineWidth', 1);
ylabel('SOC')
figure
stairs(tot_dislivello)
ylabel('dislivello')
figure
stairs(tot_acceleration)
ylabel('acceleration')
figure
stairs(T_giv)
hold on
stairs(T_req)
hold off
ylabel('comparison between Treq and Tgiven')
legend("Tgive","Treq")