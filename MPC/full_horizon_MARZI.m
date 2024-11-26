function [mf_eq,SOC,Treq,Tgive,mf,I_c,V_c,Tm]= full_horizon_MARZI(u,SOC_0,speed,acc,gear)

    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1)=SOC_0;
    mf=zeros(1,N);
    mb=zeros(1,N);
    seq=zeros(1,N);
    Treq=zeros(1,N);
    Tgive=zeros(1,N);
    I_c=zeros(1,N);
    V_c=zeros(1,N);
    Tm = zeros(1,N);
    for i=1:N
       [mf(i),SOC(i+1),seq(i),Treq(i),Tgive(i),mb(i),I_c(i),V_c(i),Tm(i)] = my_hev(speed(i),acc(i),gear(i),SOC(i),u(i));
    end
    %era così?
    mf_eq=sum(mf(1:end));%+mb(2:end).*seq(2:end));
    SOC=SOC(1:end);
    Treq=Treq(1:end);
    Tgive=Tgive(1:end);
    I_c=I_c(1:end);
    V_c=V_c(1:end);
    Tm=Tm(1:end);
end