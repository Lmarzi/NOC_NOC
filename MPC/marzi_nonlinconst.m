function [c,ceq] = marzi_nonlinconst(u,SOC_0,speed,acc,gear)

[~,SOC,Treq,Tgiv,~,I_c,V_c,~]=full_horizon_MARZI(u,SOC_0,speed,acc,gear);

c=[SOC-0.7,0.4-SOC,I_c,V_c];
ceq=[Treq-Tgiv,SOC(end)-0.55];

end