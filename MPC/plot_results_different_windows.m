for i=1:7
    StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
    FullStateUpdate=@(u)my_full_horizon(u,SOC_START,StateUpdate);
    [~,SOC2plot,mf]=FullStateUpdate(u);
    mf2plot(i)=sum(mf);
end
stem(mf2plot)