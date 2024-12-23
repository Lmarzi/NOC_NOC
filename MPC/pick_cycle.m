function driving_cycle=pick_cycle(x)
    if x<=0.33
       load ARTEMIS.mat;
       driving_cycle=ARTEMIS;
   elseif x<=0.66
       load ARTEMIS_road.mat
       driving_cycle=ARTEMIS_road;
    else
       load WLTC.mat
       driving_cycle=WLTC;
    end
    driving_cycle=[driving_cycle;zeros(1,length(driving_cycle))];
    if x>=0.9
        res=crea_dislivello(length(driving_cycle),length(driving_cycle)/10);
        driving_cycle(4,:)=res;
    end
end