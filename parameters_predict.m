function [tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, v_control, v_min, Co2, xi_ramp, xi_speed, xi_theta, off_rate] = parameters_predict(weather)
% tao=18+9*weather; % unit: s
% kai=40+20*weather;
% yita_high=65; % km^2/h
% yita_low=30-weather*5;
% rou_max=180-weather*30; % veh/km/lane
% sigma=0.1+weather*0.2;
% am=1.867-weather*0.2;
% v_free=102-10*weather; % km/h
% rou_crit=33.5-6*weather; % veh/km/lane
% alpha=0.1-0.1*weather;
T=10; % unit: s
lambda=2;
% Lm=1000-50*weather; % unit: m
v_control=200; % km/h
v_min=7;
% ro=1;
% Co1=4000; % veh/h
Co2=2000; % veh/h
xi_ramp=0.4;
xi_speed=0.4;
xi_theta=1;
off_rate=0.95;
switch weather
    case 1
        tao=18*1.05;
        kai=40*1.05;
        yita_high=65*1.05;
        yita_low=30*1.05;
        rou_max=180*1.05;
        sigma=0.1*1.05;
        am=1.867*1.05;
        v_free=102*1.05;
        rou_crit=33.5*1.05;
        alpha=0.1*1.05;
        Lm=1000*1.05;
    case 2
        tao=27*1.05;
        kai=60*1.05;
        yita_high=55*1.05;
        yita_low=25*1.05;
        rou_max=150*1.05;
        sigma=0.3*1.05;
        am=1.667*1.05;
        v_free=92*1.05;
        rou_crit=26.5*1.05;
        alpha=0*1.05;
        Lm=900*1.05;
    case 3
        tao=36*1.05;
        kai=80*1.05;
        yita_high=45*1.05;
        yita_low=20*1.05;
        rou_max=120*1.05;
        sigma=0.5*1.05;
        am=1.467*1.05;
        v_free=82*1.05;
        rou_crit=19.5*1.05;
        alpha=-0.1*1.05;
        Lm=800*1.05;
end

end


