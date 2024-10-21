a = 21.57;
a_SI = 95.98;
b = 0.2548;
b_SI=2.534;
c = 0.01839;
c_SI=0.409;
m = 1200;
R = 28;

speed_ms = 0:1/3.6:150/3.6;
speed = 0:1:150;
Fcd = zeros(length(speed));
for i=1:length(speed)
    Fcd(i)= a_SI+b_SI*speed_ms(i)+c_SI*speed_ms(i)^2;
end
plot(speed,Fcd)