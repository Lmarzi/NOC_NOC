load ARTEMIS.mat;
load ARTEMIS_road.mat
load WLTC.mat
load eff_interpol.mat
%load Results\LONG_FINAL.mat
load Results\different_windows_1_7.mat
load Results\dp_LONG_FINAL.mat
struct_eff=load("eff_interpol.mat");

etam_int=struct_eff.etam_int;
etaeng_int=struct_eff.etaeng_int;
Tmmax_int=struct_eff.Tmmax_int;
Tmmin_int=struct_eff.Tmmin_int;
Temax_int=struct_eff.Temax_int;
SOC_START=0.52;
StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
FullStateUpdate=@(u)full_horizon(u,SOC_START,StateUpdate);

for i=1:1
    [cost,SOC2plot,T_req,T_give,mf_dp,I_c,V_c]=FullStateUpdate(res.u);
    plot(SOC2plot)
end
