clear; 
% clc
% close all
%% Initial state
x=[zeros(62,1);0];
v_free=102;
u=[v_free,v_free,v_free,v_free,v_free,v_free,v_free,v_free,v_free,1,1,1];Weather=[];
% noise_o0=random('Normal',0,150,1,151); % normal distributed noise on the demand
% noise_o1=random('Normal',0,60,1,151);
% noise_o2=random('Normal',0,60,1,151);
% noise_o3=random('Normal',0,60,1,151);
initial_weather=2;
end_weather=3;
weather=initial_weather;  % weather condition: 1 is sunny, 2 is rainy, 3 is storming
for i=1:60
    x=Freeway_model(x,u, 1);
end
xx=[];
% u=[102,102,102,102,102,102,1,1,1];
% load('weather_01.mat');
% u=[100*ones(2,900);u_ramp];
% u=repelem(U,1,6);
for i=1:720
    
%     weather=3;
%     u=U(:,i);
%     weather=(end_weather-initial_weather)/720*i+initial_weather;
    x=Freeway_model(x,u, weather);
    xx=[xx x];
    if i>=360
        weather=end_weather;
    end
    Weather=[Weather weather];
end
rou_11=xx(1,:);
v_11=xx(2,:);
q_11=xx(3,:);
rou_12=xx(4,:);
v_12=xx(5,:);
q_12=xx(6,:);
rou_13=xx(7,:);
v_13=xx(8,:);
q_13=xx(9,:);
rou_14=xx(10,:);
v_14=xx(11,:);
q_14=xx(12,:);
rou_15=xx(13,:);
v_15=xx(14,:);
q_15=xx(15,:);
rou_16=xx(16,:);
v_16=xx(17,:);
q_16=xx(18,:);
rou_21=xx(19,:);
v_21=xx(20,:);
q_21=xx(21,:);
rou_22=xx(22,:);
v_22=xx(23,:);
q_22=xx(24,:);
rou_23=xx(25,:);
v_23=xx(26,:);
q_23=xx(27,:);
rou_24=xx(28,:);
v_24=xx(29,:);
q_24=xx(30,:);
rou_25=xx(31,:);
v_25=xx(32,:);
q_25=xx(33,:);
rou_26=xx(34,:);
v_26=xx(35,:);
q_26=xx(36,:);
rou_31=xx(37,:);
v_31=xx(38,:);
q_31=xx(39,:);
rou_32=xx(40,:);
v_32=xx(41,:);
q_32=xx(42,:);
rou_33=xx(43,:);
v_33=xx(44,:);
q_33=xx(45,:);
rou_34=xx(46,:);
v_34=xx(47,:);
q_34=xx(48,:);
rou_35=xx(49,:);
v_35=xx(50,:);
q_35=xx(51,:);
rou_36=xx(52,:);
v_36=xx(53,:);
q_36=xx(54,:);
q_o0=xx(55,:);
w_o0=xx(56,:);
q_o1=xx(57,:);
w_o1=xx(58,:);
q_o2=xx(59,:);
w_o2=xx(60,:);
q_o3=xx(61,:);
w_o3=xx(62,:);
Total_veh=(rou_11+rou_12+rou_13+rou_14+rou_15+rou_16+rou_21+rou_22+rou_23+rou_24+rou_25+rou_26+...
    rou_31+rou_32+rou_33+rou_34+rou_35+rou_36).*1000./1000.*2+w_o0+w_o1+w_o2+w_o3;
TTS=10/3600.*Total_veh;
% Flow_ave=sum(q_22)/360;
%%
fprintf('TTS is %.3f veh*h \n', sum(TTS))
% fprintf('Average outflow is %.3f veh*h \n', sum(Flow_ave))
figure();
t=1/360:1/360:2.0;
subplot(4,2,1)
plot(t, v_11, '-', 'linewidth', 1.5);
hold on;
plot(t, v_15, '--', 'linewidth', 1.5);
hold on;
plot(t, v_21, ':', 'linewidth', 1.5);
hold on;
plot(t, v_25, '-', 'linewidth', 1.5);
hold on;
plot(t, v_31, '--', 'linewidth', 1.5);
hold on;
plot(t, v_35, ':', 'linewidth', 1.5);
legend('Segmeng 1-1','Segmeng 1-5','Segmeng 2-1','Segmeng 2-5','Segmeng 3-1','Segmeng 3-5')
xlabel('Time [h]');
ylabel('Speed [km/h]')

subplot(4,2,2)
plot(t, q_11, '-', 'linewidth', 1.5);
hold on;
plot(t, q_15, '--', 'linewidth', 1.5);
hold on;
plot(t, q_21, ':', 'linewidth', 1.5);
hold on;
plot(t, q_25, '-', 'linewidth', 1.5);
hold on;
plot(t, q_31, '--', 'linewidth', 1.5);
hold on;
plot(t, q_35, ':', 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Flow [veh/h]')

subplot(4,2,3)
plot(t, rou_11, '-', 'linewidth', 1.5);
hold on;
plot(t, rou_15, '--', 'linewidth', 1.5);
hold on;
plot(t, rou_21, ':', 'linewidth', 1.5);
hold on;
plot(t, rou_25, '-', 'linewidth', 1.5);
hold on;
plot(t, rou_31, '--', 'linewidth', 1.5);
hold on;
plot(t, rou_35, ':', 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Density [veh/km]')

subplot(4,2,4)
plot(t, demando0(91:810), '-', 'linewidth', 1.5);
hold on;
plot(t, demando1(91:810), '-.', 'linewidth', 1.5);
hold on;
plot(t, demando2(91:810), ':', 'linewidth', 1.5);
hold on;
plot(t, demando3(91:810), '--', 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Demand [veh/h]');
legend('O_0','O_1','O_2','O_3')
% ylim([0 4000])

subplot(4,2,5)
plot(t, q_o0, '-', 'linewidth', 1.5);
hold on;
plot(t, q_o1, '--', 'linewidth', 1.5);
hold on;
plot(t, q_o2, '-.', 'linewidth', 1.5);
hold on;
plot(t, q_o3, ':', 'linewidth', 1.5);
legend('O_0','O_1','O_2','O_3')
xlabel('Time [h]');
ylabel('Outflow [veh/h]')
% ylim([0 4000])

subplot(4,2,6)
plot(t,w_o0,'-', 'linewidth', 1.5);
hold on;
plot(t,w_o1,'--', 'linewidth', 1.5);
hold on;
plot(t,w_o2,'-.', 'linewidth', 1.5);
hold on;
plot(t,w_o3,':', 'linewidth', 1.5);
legend('O_0','O_1','O_2','O_3')
xlabel('Time [h]');
ylabel('Queue length [veh]')

subplot(4,2,7)
plot(t,Total_veh,'-', 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Total vehicles [veh]')

subplot(4,2,8)
plot(t,Weather,'-','linewidth', 1.5);
xlabel('Time [h]');
ylabel('Weather')

figure
plot(t, demando0(61:780), '-', 'linewidth', 1.5);
hold on;
plot(t, demando1(61:780), '-.', 'linewidth', 1.5);
hold on;
plot(t, demando2(61:780), ':', 'linewidth', 1.5);
hold on;
plot(t, demando3(61:780), '--', 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Demand [veh/h]');
ylim([0 3500]);
legend('O_0','O_1','O_2','O_3')