function driving_cycle = recreate_path(x,y,N,start,slope_perc)
%Function to recreate a randomised path from the original seed. 

driving_cycle=[];
%For all elements of x and y it recreates the driving cycle the same way it was
%created the first time
for k=1:min(length(x),length(y))
    driving_cycle_t=pick_cycle(x(k),y(k),slope_perc); 
    driving_cycle=[driving_cycle,driving_cycle_t(:,start+1:end-N)];
end
end