function [c,ceq] = nonlinconstr(u,SOC_0,StateUpdate)
[~,SOC,Treq,Tgiv,~,I_c,V_c]=full_horizon(u,SOC_0,StateUpdate);

c=[SOC-0.7;0.4-SOC;I_c;V_c];
ceq=Treq-Tgiv;

end