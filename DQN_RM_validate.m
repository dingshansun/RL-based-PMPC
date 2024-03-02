% clear; 
% clc
% close all
%% Initialize
Exit=[];
x=[zeros(62,1);0];
% ramp_only=0;
u=[102,102,102,102,102,102,102,102,102,1,1,1];
initial_weather=2;
weather=initial_weather;  % taken as the nominal weather
end_weather=1;
nominal_model=1;
for i=1:60
    x=Freeway_model(x,u,1); % real model use the real weather
end

%% Parameters
Np=3; % Np times M is the prediction horizon
Nc=3;  % Nc is the variables of theta during the prediction
M=30; % MPC operation time: 30 simulation steps
Mc=6; % control sampling time: 6 simulation steps
N=720;
% N_max=15; % multi-start point
xx=[];
% [tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, ~, v_min, Co2, xi_r, xi_s,xi_theta,~] = parameters_real(weather);
% dim_speed=6;
% dim_ramp=3;
% para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s, weather, rou_crit, Mc, xi_theta];
% options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
% 
% % v_control=repmat(100*ones(dim_speed,1),1,Nc);
% % v_control_pre=v_control(:,1);
% 
dim_theta=1;
U=[];
Theta=zeros(dim_theta,N);
u_pre=u'; %
rm_pre=1;
% % theta_lb=repmat([0;-100;-100;0],1,Nc);
% % theta_ub=repmat([2;100;100;10],1,Nc);
% % theta_pre=repmat(ones(dim_theta,1), 1, Nc);
% theta_lb=repmat(-10,dim_theta,Nc);
% theta_ub=repmat(10,dim_theta,Nc);
theta_pre=repmat(ones(dim_theta,1), 1, Nc);
% Rou=repelem([35 15], 1, 12);
%%
for i=1:N/Mc
    if x(63)>=420
        weather=end_weather;
    end
    %% DQN action
    rou_13=x(7);
    v_13=x(8);
    rou_14=x(10);
    v_14=x(11);
    rou_15=x(13);
    v_15=x(14);
    rou_23=x(25);
    v_23=x(26);
    rou_24=x(28);
    v_24=x(29);
    rou_25=x(31);
    v_25=x(32);
    rou_33=x(43);
    v_33=x(44);
    rou_34=x(46);
    v_34=x(47);
    rou_35=x(49);
    v_35=x(50);
    
    w_o0=x(56,:);
    w_o1=x(58);
    w_o2=x(60);
    w_o3=x(62);
    k=x(63);
    norm_state=[rou_13 v_13 rou_14 v_14 rou_15 v_15 rou_23 v_23 rou_24 v_24 rou_25 v_25 rou_33 v_33 rou_34 v_34 rou_35 v_35]'/100;
    norm_queue=[w_o0;w_o1;w_o2;w_o3]/100;
    norm_demand=[demando0(k)/1000; demando1(k)/1000; demando2(k)/1000; demando3(k)/1000];
    Observation=[norm_state; norm_demand;norm_queue;rm_pre;rou_36_down(k)/100;weather];
    act = getAction(agent, Observation);
    u_rm=act{1}/10;
    u=[102; 102; 102; 102; 102; 102; 102; 102; 102; u_rm; u_rm; u_rm];
    for k=1:Mc
        x=Freeway_model(x,u,weather);
        xx=[xx x];
    end
    u_pre=u;
    rm_pre=u_rm;
    U=[U repmat(u,1,Mc)];
end
%% figure
u_speed=U(1:9,:);
u_ramp=U(10:12,:);
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
%%
fprintf('TTS is %.3f veh*h \n', sum(TTS))
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
% subplot(3,1,1)
plot(t,Theta, 'linewidth', 1.5);
xlabel('Time [h]');
ylabel('Theta');
legend('0','1','2','3')
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
% subplot(3,1,2)
plot(t,u_speed,'-', 'linewidth', 1.5);
legend('v-11','v-12','v-21','v-22','v-31','v-32');
xlabel('Time [h]');
ylabel('Speed limits')
% 
subplot(4,2,8)
% subplot(3,1,3)
plot(t,u_ramp,'-','linewidth',1.0);
legend('O_1','O_2','O_3');
ylim([0 1]);
xlabel('Time [h]');
ylabel('Ramp metering')







