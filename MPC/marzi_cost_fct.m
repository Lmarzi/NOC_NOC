function [mf_eq]= marzi_cost_fct(u,SOC_0,speed,acc,gear)

    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1)=SOC_0;
    mf=zeros(1,N);
    for i=1:N
       [mf(i),SOC(i+1),~,~,~,~,~,~,~] = my_hev(speed(i),acc(i),gear(i),SOC(i),u(i));
    end
    %era così?
    mf_eq=sum(mf(1:end));%+mb(2:end).*seq(2:end));
end