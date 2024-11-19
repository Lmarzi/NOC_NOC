function [c,ceq] = nonlinconstr(u,SOC_0,StateUpdate)
[mf_eq,I,SOC,mf,Te_max,Tm_min,Tm_max,Te,Tm]=full_horizon(u,SOC_0,StateUpdate);

c=[SOC-0.7;0.3-SOC;Te-Te_max;Tm-Tm_max;Tm_min-Tm];
ceq=[];

end