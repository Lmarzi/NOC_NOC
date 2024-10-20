%Create ARTEMIS URBAN DRIVING CYCLE!!
t = 0:1:1000;
out=sim("Code\DRIVING_CYCLE_GEN_SIM");
speed_vector = out.speed(:,1);
acceleration_vector = out.acc(:,1);
gearnumber_vector = ones(length(t),1);
for i=1:length(t)
    % Determina la marcia in base alla velocità
    if speed_vector(i) <= 20/3.6
        gearnumber_vector(i) = 1;
    elseif speed_vector(i) <= 40/3.6
        gearnumber_vector(i) = 2;
    elseif speed_vector(i) <= 60/3.6
        gearnumber_vector(i) = 3;
    elseif speed_vector(i) <= 90/3.6
        gearnumber_vector(i) = 4;
    else
        gearnumber_vector(i) = 5;
    end
    
    % Aggiusta la marcia in base all'acceleration_vector(i)
    if acceleration_vector(i) > 2.0 && gearnumber_vector(i) > 1
        gearnumber_vector(i) = max(1, gearnumber_vector(i) - 1);  % Scendi di una marcia se l'acceleration_vector(i) è alta
    elseif acceleration_vector(i) < 0.5 && gearnumber_vector(i) > 1
        gearnumber_vector(i) = min(6, gearnumber_vector(i) + 1);  % Sali di una marcia se l'acceleration_vector(i) è molto bassa
    end
end
ARTEMIS_1 = [speed_vector,acceleration_vector,gearnumber_vector];
save("Artemis.mat","ARTEMIS")
