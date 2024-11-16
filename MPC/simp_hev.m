function [mf,SOC,seq,mb] = simp_hev(speed,acceleration,gear,cur_SOC,u,consts)  
    wv=speed./consts.wheel_radius;
    dwv=speed./consts.wheel_radius;
    % Crankshaft speed (rad/s)
    wg  = (gear>0) .* consts.r_gear(gear + (gear==0)) .* wv;
    % Crankshaft acceleration (rad/s^2)
    dwg = (gear>0) .* consts.r_gear(gear+ (gear==0)) .* dwv;

    Tv = (consts.a_cd+consts.b_cd.*speed+ consts.c_cd.*speed.^2 + consts.vehicle_mass.*acceleration) .* consts.wheel_radius;
    Tg  = (gear>0) .* (Tv>0)  .* Tv ./ consts.r_gear(gear + (gear==0)) ./ consts.gearbox_efficiency...
    + (gear>0) .* (Tv<=0) .* Tv ./ consts.r_gear(gear + (gear==0)) .* consts.gearbox_efficiency;
    %Engine drag torque
    Te0  = dwg * consts.engine_inertia;
    % Electric motor drag torque (Nm)
    Tm0  = dwg * consts.motor_inertia;
    % Total required torque (Nm)
    Ttot = Te0.*(u~=1) + Tm0 + Tg;
    % Torque provided by engine
    Te  = (wg>0) .* (Ttot>0)  .* (1-u).*Ttot;
    Tb  = (wg>0) .* (Ttot<=0) .* (1-u).*Ttot;
    
    % Engine efficiency (function of speed)
    e_th = (wg~=0) .*interp2(consts.we_list,consts.Te0_list,consts.eta,wg.*ones(size(Te)),Te)+0.070694.*(Te==0);
    
    % Battery voltage
    v = interp1(consts.soc_list, consts.V_oc, speed,'linear','extrap');
    % Torque provided by electric motor
    Tm  = (wg>0) .*    u .*       Ttot;
    % Battery efficiency
    % Electric motor efficiency
    e = (wg~=0) .* interp2(consts.Tm_list,consts.wm_list,consts.etam,abs(Tm),wg.*ones(size(Tm))) + (wg==0);
    % Calculate electric power consumption
    Pm =  (Tm<0) .* wg.*Tm.*e + (Tm>=0) .* wg.*Tm./e;
    
    SOC=-1/(consts.battery_capacity*3600)*Pm./v+cur_SOC;
    mf=Te.*wg./e_th./consts.gasoline_lower_heating_value;
    mb=Pm./e./consts.gasoline_lower_heating_value;
    seq = (Tm>0).*1/(0.35*0.9)+(Tm<0).*0.9/0.35;

end