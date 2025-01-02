function driving_cycle = recreate_path(x,y,N,start,slope_perc)
%To create the random path it uses two random numbers, that then get saved for
%future simulations on the same exact path. x picks the path, y picks
%the downhill pattern, that will only be present 25% of the time
tot_speed=[];
tot_acceleration=[];
tot_gear=[];
tot_dislivello=[];
for k=1:length(x)
    x(k)=rand;
    y(k)=rand;
    
    [driving_cycle]=pick_cycle(x(k),y(k),slope_perc);    
    tot_speed=[tot_speed,driving_cycle(1,start:start+end-N)];
    tot_acceleration=[tot_acceleration,driving_cycle(2,start:start+end-N)];
    tot_gear=[tot_gear,driving_cycle(3,start:start+end-N)];
    tot_dislivello=[tot_dislivello,driving_cycle(4,start:start+end-N)];
end
driving_cycle=[tot_speed;tot_acceleration;tot_gear;tot_dislivello];

end