function [mf_eq,SOC,Treqs,Tgiv,mf,I_c,V_c,Tm]= full_horizon(u,SOC_0,StateUpdate)
    N=length(u);
    %SOC=zeros(1,N+1);
    SOC=SOC_0;
    mf=zeros(1,N+1);
    mb=zeros(1,N+1);
    seq=zeros(1,N+1);
    Treq=zeros(1,N+1);
    Tgive=zeros(1,N+1);
    I_c=zeros(1,N+1);
    V_c=zeros(1,N+1);
    for i=2:N+1
        [mf(i),SOC(i),seq(i),Treq(i),Tgive(i),mb(i),I_c(i),V_c(i),Tm(i)]=StateUpdate(u(i-1),SOC(i-1),i-1);
    end

    %era così?
    mf_eq=sum(mf(2:end)+mb(2:end).*seq(2:end));
    SOC=SOC(2:end);
    Treqs=Treq(2:end);
    Tgiv=Tgive(2:end);
    I_c=I_c(2:end);
    V_c=V_c(2:end);
    Tm=Tm(2:end);
end