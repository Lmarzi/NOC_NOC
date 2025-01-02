function [u_n,exitf_i] = for_coder(u_n_old,speed,acceleration,gear,dislivello,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int)
StateUpdate=@(input,cur_SOC,i)my_hev(speed(i),acceleration(i),gear(i),dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
myFullStateUpdate=@(u)my_full_horizon(u,SOC(1),StateUpdate);

%calcoli la u ottima sull'orizzonte dei 5 secondi
[u_n,~,~,exitf_i,~,~]=myfmincon(myFullStateUpdate,[u_n_old(2:end);u_n_old(end)],[],[],C,d,0*N,8*N,myoptions);
end