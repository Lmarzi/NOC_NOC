function driving_cycle=pick_cycle(x)
   load ARTEMIS.mat;
   load ARTEMIS_road.mat
   load WLTC.mat
    if x<=0.33
       driving_cycle=ARTEMIS;
   elseif x<=0.66
       driving_cycle=WLTC;
    else
       driving_cycle=ARTEMIS_road;
   end
    
end