function [mf_eq,I,SOC,mf]= full_horizon(u,SOC_0,StateUpdate)
    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1)=SOC_0;
    mf=zeros(1,N+1);
    mb=zeros(1,N+1);
    seq=zeros(1,N+1);
    I=zeros(1,N+1);
    for i=2:N+1
        [mf(i),SOC(i),seq(i),mb(i),I(i)]=StateUpdate(u(i-1),SOC(i-1),i);
    end
    %Funzione di costo un po' tumore, ma funziona sembra :)
    mf_eq=sum(mf(2:end).*seq(2:end)+abs(0.55-SOC(end)./(seq(2:end)+(seq(2:end)==0))));
    SOC=SOC(2:end);
end