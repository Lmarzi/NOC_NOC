function [mf,SOC_new,seq,Treq,Tgiv,mb,I_c,V_c,out] = my_hev(speed,acceleration,gear,cur_SOC,u,F1,F2)

% VEHICLE PARAMETERS
wheel_radius = 0.281154; %m
vehicle_mass = 1200; %kg

% Coasting down test parameters
a_cd = 93.086154;  %N
b_cd = 2.5373490;  %N/(m/s))
c_cd = 0.38382359; %N/(m/s)^2)
g=9.81;


speed=max(speed,5/3.6);

inp.W{1}=speed;
inp.W{2}=acceleration;
inp.W{3}=gear;
inp.W{4}=0;
inp.X{1}=cur_SOC;
inp.U{1}=u;

% Wheel speed (rad/s)
wv  = inp.W{1} ./ wheel_radius;

Tv = (vehicle_mass*g*sin(inp.W{4})+a_cd*cos(inp.W{4})*(inp.W{1}>0)+b_cd.*inp.W{1}*cos(inp.W{4}) + c_cd.*inp.W{1}.^2 + vehicle_mass.*inp.W{2}) .* wheel_radius;

% TRANSMISSION
gearbox_efficiency = 0.96;
% final drive ratio (from main axle to wheel)
t0 = 3.1725494;
% gear box (from motor to main axle) 1st to 5th gear
tprime = [3.417 1.958 1.276 0.943 0.757];
% gear ratios
r_gear = t0.*tprime;
% Crankshaft speed (rad/s)
wg  = (inp.W{3}>0) .* r_gear(inp.W{3} + (inp.W{3}==0)) .* wv; 
% Crankshaft torque (Nm)
Tg  = (inp.W{3}>0) .* (Tv>0)  .* Tv ./ r_gear(inp.W{3} + (inp.W{3}==0)) ./ gearbox_efficiency...
    + (inp.W{3}>0) .* (Tv<=0) .* Tv ./ r_gear(inp.W{3} + (inp.W{3}==0)) .* gearbox_efficiency;
Treq=Tg;

% motor speed list
wm_list = [62.8318530717959	68.0678408277789	73.3038285837618	78.5398163397448	83.7758040957278	89.0117918517108	94.2477796076938	99.4837673636768	104.719755119660	109.955742875643	115.191730631626	120.427718387609	125.663706143592	130.899693899575	136.135681655558	141.371669411541	146.607657167524	151.843644923507	157.079632679490	162.315620435473	167.551608191456	172.787595947439	178.023583703422	183.259571459405	188.495559215388	193.731546971371	198.967534727354	204.203522483337	209.439510239320	214.675497995303	219.911485751286	225.147473507269	230.383461263252	235.619449019235	240.855436775217	246.091424531200	251.327412287183	256.563400043166	261.799387799149	267.035375555132	272.271363311115	277.507351067098	282.743338823081	287.979326579064	293.215314335047	298.451302091030	303.687289847013	308.923277602996	314.159265358979	319.395253114962	324.631240870945	329.867228626928	335.103216382911	340.339204138894	345.575191894877	350.811179650860	356.047167406843	361.283155162826	366.519142918809	371.755130674792	376.991118430775	382.227106186758	387.463093942741	392.699081698724	397.935069454707	403.171057210690	408.407044966673	413.643032722656	418.879020478639	424.115008234622	429.350995990605	434.586983746588];
% motor maximum torque (indexed by speed list)
Tmmax   = [	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	100	98.7858267466937	97.1114907001395	95.4929658551372	93.9275073984956	92.4125476017457	90.9456817667973	89.5246554891911	88.1473530970497	86.8117871410338	85.5160888254960	84.2584992839446	83.0373616131628	81.8511135901176	80.6982810043413	79.5774715459477	78.4873691960032	77.4267290717329	76.3943726841098	75.3891835698452	74.4101032637433	73.4561275808748	72.5263031811169	71.6197243913529	70.7355302630646	69.8729018452223	69.0310596543161	68.2092613250980];
% motor minimum torque (indexed by speed list)
Tmmin   = [	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-50	-49.3929133733468	-48.5557453500698	-47.7464829275686	-46.9637536992478	-46.2062738008728	-45.4728408833987	-44.7623277445956	-44.0736765485249	-43.4058935705169	-42.7580444127480	-42.1292496419723	-41.5186808065814	-40.9255567950588	-40.3491405021707	-39.7887357729738	-39.2436845980016	-38.7133645358664	-38.1971863420549	-37.6945917849226	-37.2050516318716	-36.7280637904374	-36.2631515905584	-35.8098621956765	-35.3677651315323	-34.9364509226112	-34.5155298271580	-34.1046306625490];
we_list  = [0	52.3598780000000	78.5398160000000	104.719760000000	130.899690000000	157.079630000000	183.259570000000	209.439510000000	235.619450000000	261.799390000000	287.979330000000	314.159270000000	340.339200000000	366.519140000000	418.879020000000	471.238900000000	523.598780000000	575.958650000000	628.318530000000	680.678410000000	733.038290000000];
%Max eng torque
Tmax = [0 50 80 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 153.012 150.15 128.6901];


Tm_min=interp1(wm_list,Tmmin,wg,'linear','extrap');
Tm_max=interp1(wm_list,Tmmax,wg,'linear','extrap');
% Maximum engine torque
Te_max = interp1(we_list,Tmax,wg,'linear','extrap');


% Total required torque (Nm)
Ttot = Tg;
% Torque provided by engine
%Te  = max(min(Ttot>0)  .* (1-inp.U{1}).*Ttot,Te_max),0);
Te = (Ttot>0)  .* (1-inp.U{1}).*Ttot;
Tb  = (Ttot<=0) .* (1-inp.U{1}).*Ttot;
% Torque provided by electric motor
%Tm  = min(max(inp.U{1} .* Ttot,Tm_min),Tm_max);
Tm = inp.U{1} .* Ttot;
Tm=Tm*(1-(Tm>=0 && Treq<=0));
Tgiv=Te+Tb+Tm;

%compute mf
gasoline_lower_heating_value = 43.308*10^6; %J/kg
e_th = F2(wg.*ones(size(Te)),Te);
e_th(isnan(e_th)) = 0.5;

mf=Te.*wg./e_th./gasoline_lower_heating_value;

%SOC update

% Calculate electric power consumption
% motor efficiency map (indexed by speed list and torque list)

e=F1(abs(Tm),wg.*ones(size(Tm)));
e(isnan(e))=1;
Pm =  (Tm<0) .* wg.*Tm.*e + (Tm>=0) .* wg.*Tm./e;

% BATTERY SIMPLIFIED MODEL-Only a constant voltage generator
Vn = 100;
% Battery current limitations
battery_capacity =45; %Ah 

SOC_new = -1/(battery_capacity.*3600).*Pm./Vn+inp.X{1};

% Battery power consumption
% columbic efficiency (0.9 when charging)
% Battery current
r = 39e-3; %ohm
eff = (Pm>0) + (Pm<=0) .* 0.9;
Ib  =   eff .* (Vn-sqrt(Vn.^2 - 4.*r.*Pm))./(2.*r);
max_disch_curr = 100;   %A
max_char_curr    = 125; %A
Im = (Pm>0) .* max_disch_curr + (Pm<=0) .* max_char_curr;
I_c=(Pm>0).*(Im-Ib)./Im-(Pm<=0).*(Ib-Im)./Im;
V_c=(Vn.^2 - 4.*r.*Pm)./Vn^2;

Pb =   Ib .* Vn;
mb=Pb./gasoline_lower_heating_value;
seq = ((Tm>0).*1/(0.2757*0.8879)+(Tm<0).*0.8879/0.2757)*20*(-(2*(SOC_new-0.55)).^3+(2*(0.7-0.55)).^3);
out.Tmmax = Tm_max;
out.Tmmin = Tm_min;
out.Temax = Te_max;
out.Te = Te;
out.Tm = Tm;

end