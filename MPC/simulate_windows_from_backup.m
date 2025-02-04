
driving_cycle=recreate_path(x,y,0,0,0.25);
driving_cycle(4,:)=driving_cycle(4,:)/0.5709998338118033;
SOC={};
mf={};
cost={};
for i=1:7
   StateUpdate=@(input,cur_SOC,i)my_hev(driving_cycle(1,i),driving_cycle(2,i),driving_cycle(3,i),driving_cycle(4,i),cur_SOC,input,etam_int,etaeng_int,Tmmax_int,Tmmin_int,Temax_int);
   FullStateUpdate=@(u)my_full_horizon(u,0.55,StateUpdate);
   [cost_tmp,SOC{i},mf{i}]=FullStateUpdate(u_back{i});
   cost{i}=cost_tmp(1);
   %plot(SOC{i})
end
%hold off

%%
clear tot_f
clear tot_cost
for i=1:7
    tot_cost(i)=cost{i}*1000;
    tot_mf(i)=sum(mf{i})*1000;
end
figure 
stem(tot_cost)
% xlim([0.9,5.1])
% ylim([min(tot_cost-100),max(tot_cost)+10]*1000)
figure
stem(tot_mf)
% xlim([0.9,5.1])
% ylim([min(tot_mf),max(tot_mf)+10])