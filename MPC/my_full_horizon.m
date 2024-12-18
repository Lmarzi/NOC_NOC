function toreturn= my_full_horizon(u,SOC_0,StateUpdate)
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
    % Model simulation
    for i=2:N+1
        [mf(i-1),SOC(i),seq(i-1),Treq(i-1),Tgive(i-1),mb(i-1),I_c(i-1),V_c(i-1),out(i-1)]=StateUpdate(u(i-1,1),SOC(1,i-1),i-1);
    end
    % Define the cost function, equality and inequality constraints to
    % return to myfmincon
    mf_eq=sum(mf+mb.*seq);
    c=[-SOC(2:end)+0.7,SOC(2:end)-0.4,I_c,V_c,[out.Tm]-[out.Tmmin],[out.Tmmax_cons],[out.Temax_cons]];
    ceq=[];
    toreturn=[mf_eq,ceq,c]';
end