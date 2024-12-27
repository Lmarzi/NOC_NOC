function [mf_eq,SOC,Treq,Tgive,mf,I_c,V_c]= full_horizon(u,SOC_0,StateUpdate)
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
        [mf(i-1),SOC(i),seq(i-1),Treq(i-1),Tgive(i-1),mb(i-1),I_c(i-1),V_c(i-1)]=StateUpdate(u(i-1),SOC(i-1),i-1);
    end

    %era così?
    mf_eq=sum(mf+mb.*seq);
    SOC=SOC(2:end);

end