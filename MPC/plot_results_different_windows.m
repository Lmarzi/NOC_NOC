for i=1:7
    StateUpdate=@(input,cur_SOC,i)my_hev(tot_speed(i),tot_acceleration(i),tot_gear(i),tot_dislivello(i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
    FullStateUpdate=@(u)my_full_horizon(u,SOC_START,StateUpdate);
    [cost,SOC2plot,mf]=FullStateUpdate(u_back{i});
    mf2plot(i)=sum(mf);
    cost2plot(i)=cost(1);
end
stem(mf2plot)

stem(cost2plot)