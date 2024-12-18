function [mf_eq,SOC,Treqs,Tgiv,mf,I_c,V_c,Te]= full_horizon(u,SOC_0,StateUpdate)
    N=length(u);
    SOC=zeros(1,N+1);
    SOC(1,1)=SOC_0;
    mf=zeros(1,N);
    mb=zeros(1,N);
    seq=zeros(1,N);
    Treq=zeros(1,N);
    Tgive=zeros(1,N);
    I_c=zeros(1,N);
    V_c=zeros(1,N);
    for i=2:N+1
        [mf(i-1),SOC(i),seq(i-1),Treq(i-1),Tgive(i-1),mb(i-1),I_c(i-1),V_c(i-1),Te(i-1)]=StateUpdate(u(i-1),SOC(i-1),i-1);
    end

    %era così?
    mf_eq=sum(mf+mb.*seq);
    SOC=SOC(2:end);
    Treqs=Treq(1:end);
    Tgiv=Tgive(1:end);
    I_c=I_c(1:end);
    V_c=V_c(1:end);
    Te=Te(1:end);
end