function SOC_dot = SOC_lin(params,Pv,u)
Qb=params(1);
Vn=params(2);

SOC_dot=-1/(Qb*Vn)*Pv*u;
end