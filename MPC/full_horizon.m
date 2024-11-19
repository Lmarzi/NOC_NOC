function [mf_eq,I,SOC,mf,Te_max,Tm_min,Tm_max,Te,Tm]= full_horizon(u,SOC_0,StateUpdate)
    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1)=SOC_0;
    mf=zeros(1,N+1);
    mb=zeros(1,N+1);
    seq=zeros(1,N+1);
    I=zeros(1,N+1);
    Te_max=zeros(1,N+1);
    Tm_min=zeros(1,N+1);
    Tm_max=zeros(1,N+1);
    Te=zeros(1,N+1);
    Tm=zeros(1,N+1);

    for i=2:N+1
        [mf(i),SOC(i),seq(i),mb(i),I(i),Te_max(i),Tm_min(i),Tm_max(i),Te(i),Tm(i)]=StateUpdate(u(i-1),SOC(i-1),i-1);
    end

    %Funzione di costo un po' tumore, ma funziona sembra :)
    mf_eq=sum(mf(2:end).*seq(2:end)+abs(0.55-SOC(end)./(seq(2:end)+(seq(2:end)==0))));
    SOC=SOC(2:end);
    Te_max=Te_max(2:end);
    Tm_min=Tm_min(2:end);
    Tm_max=Tm_max(2:end);
    Te=Te(2:end);
    Tm=Tm(2:end);
end