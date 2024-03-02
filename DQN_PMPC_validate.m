% clear; 
% clc
% close all
%% Initialize
Exit=[];
x=[zeros(62,1);0];
% ramp_only=0;
u=[102,102,102,102,102,102,102,102,102,1,1,1];
initial_weather=3;
% weather=initial_weather;  % taken as the nominal weather
end_weather=1;
Weather_real=[ones(1,60) initial_weather*ones(1,360) end_weather*ones(1,510)];
Weather_predict=[ones(1,60) initial_weather*ones(1,870)];
Weather=Weather_predict;
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
% % theta_lb=repmat([0;-100;-100;0],1,Nc);
% % theta_ub=repmat([2;100;100;10],1,Nc);
% % theta_pre=repmat(ones(dim_theta,1), 1, Nc);
% theta_lb=repmat(-10,dim_theta,Nc);
% theta_ub=repmat(10,dim_theta,Nc);
theta_pre=repmat(ones(dim_theta,1), 1, Nc);
% Rou=repelem([35 15], 1, 12);
%%
for i=1:N/M
    weather=Weather(x(63)+1);
    [tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, ~, alpha, T, lambda, Lm, ~, v_min, Co2, xi_r, xi_s,xi_theta,~] ...
        = parameters_predict(weather);
%     rou_crit=36.5;
    %% DQN action
    if rem(i,6)==1
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
        Observation=[norm_state; norm_demand;norm_queue;u_pre(10:12);theta_pre(:,1);rou_36_down(k)/100;Weather_real(k+1)];
        act = getAction(saved_agent, Observation);
        Rou=[15 17.5 20.0 22.5 25.0 27.5 30.0 32.5 35.0 37.5 40.0];
        rou_crit=Rou(act{1}+1);
%         rou_crit=20.4750;
%         rou_crit=Rou(i);
    end
    theta_opt=MPC_imp_RM(x,u_pre,theta_pre, Weather_predict,rou_crit); % give the wrong weather, but rou_crit can be adjusted
    for j=1:M/Mc % every control step
        u=u_para_RM(theta_opt(:,1), x, u_pre, rou_crit, v_free);
        for k=1:Mc
            x=Freeway_model(x,u,Weather_real(x(63)+1));
            xx=[xx x];
%             step=x(63)-60;
%             weather=(end_weather-initial_weather)/720*step+initial_weather;
        end
        u_pre=u;
        U=[U repmat(u,1,Mc)];
    end
    
    Theta(:,M*(i-1)+1:M*i)=repmat(theta_opt(:,1),1,M);
%     theta_pre=theta_t{index};
    theta_pre=theta_opt;
end
%% figure
u_speed=U(1:6,:);
u_ramp=U(7:9,:);
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
% figure();
% t=1/360:1/360:2.0;
% subplot(4,2,1)
% plot(t, v_11, '-', 'linewidth', 1.5);
% hold on;
% plot(t, v_15, '--', 'linewidth', 1.5);
% hold on;
% plot(t, v_21, ':', 'linewidth', 1.5);
% hold on;
% plot(t, v_25, '-', 'linewidth', 1.5);
% hold on;
% plot(t, v_31, '--', 'linewidth', 1.5);
% hold on;
% plot(t, v_35, ':', 'linewidth', 1.5);
% legend('Segmeng 1-1','Segmeng 1-5','Segmeng 2-1','Segmeng 2-5','Segmeng 3-1','Segmeng 3-5')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(4,2,2)
% plot(t, q_11, '-', 'linewidth', 1.5);
% hold on;
% plot(t, q_15, '--', 'linewidth', 1.5);
% hold on;
% plot(t, q_21, ':', 'linewidth', 1.5);
% hold on;
% plot(t, q_25, '-', 'linewidth', 1.5);
% hold on;
% plot(t, q_31, '--', 'linewidth', 1.5);
% hold on;
% plot(t, q_35, ':', 'linewidth', 1.5);
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(4,2,3)
% plot(t, rou_11, '-', 'linewidth', 1.5);
% hold on;
% plot(t, rou_15, '--', 'linewidth', 1.5);
% hold on;
% plot(t, rou_21, ':', 'linewidth', 1.5);
% hold on;
% plot(t, rou_25, '-', 'linewidth', 1.5);
% hold on;
% plot(t, rou_31, '--', 'linewidth', 1.5);
% hold on;
% plot(t, rou_35, ':', 'linewidth', 1.5);
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% subplot(4,2,4)
% subplot(3,1,1)
% plot(t,Theta, 'linewidth', 1.5);
% xlabel('Time [h]');
% ylabel('Theta');
% legend('0','1','2','3')
% % ylim([0 4000])
% 
% subplot(4,2,5)
% plot(t, q_o0, '-', 'linewidth', 1.5);
% hold on;
% plot(t, q_o1, '--', 'linewidth', 1.5);
% hold on;
% plot(t, q_o2, '-.', 'linewidth', 1.5);
% hold on;
% plot(t, q_o3, ':', 'linewidth', 1.5);
% legend('O_0','O_1','O_2','O_3')
% xlabel('Time [h]');
% ylabel('Outflow [veh/h]')
% % ylim([0 4000])
% 
% subplot(4,2,6)
% plot(t,w_o0,'-', 'linewidth', 1.5);
% hold on;
% plot(t,w_o1,'--', 'linewidth', 1.5);
% hold on;
% plot(t,w_o2,'-.', 'linewidth', 1.5);
% hold on;
% plot(t,w_o3,':', 'linewidth', 1.5);
% legend('O_0','O_1','O_2','O_3')
% xlabel('Time [h]');
% ylabel('Queue length [veh]')
% 
% subplot(4,2,7)
% subplot(3,1,2)
% plot(t,u_speed,'-', 'linewidth', 1.5);
% legend('v-11','v-12','v-21','v-22','v-31','v-32');
% xlabel('Time [h]');
% ylabel('Speed limits')
% % 
% % subplot(4,2,8)
% subplot(3,1,3)
% plot(t,u_ramp,'-','linewidth',1.0);
% legend('O_1','O_2','O_3');
% ylim([0 1]);
% xlabel('Time [h]');
% ylabel('Ramp metering')
%%
function J=ModelState(x, theta, Nc, Np, para, u_pre, theta_pre)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    M=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
    weather=para(9);
    rou_crit=para(10);
    Mc=para(11);
    xi_theta=para(12);
%     uc=[u repelem(u(:, end),1,Np-Nc)];
%     uc_full=repelem(uc,1,M);
    theta_full=repelem(theta,1,Np/Nc);
    yy=[];
    Diff=zeros(size(u_pre,1),Np); % the erros between the successive control inputs
    Diff_theta=zeros(size(theta_pre,1),Nc);
    uc_full=zeros(size(u_pre,1),Np);
    for i=1:Np
        for j=1:M/Mc
            u=u_para(theta_full(:,i), x, u_pre, rou_crit);
            uc_full(:,i)=u;
            for k=1:Mc
%                 step=x(63)-60;
%                 initial_weather=0;
%                 end_weather=2;
%                 weather=(end_weather-initial_weather)/720*step+initial_weather;
                x=Freeway_model(x,u,weather,0);
                yy=[yy x];
            end
            u_pre=u;
        end
    end
    u_diff=[u_pre uc_full];
    theta_diff=[theta_pre theta];
%     for i=1:Np
%         Diff(:,i)=u_diff(:,i+1)-u_diff(:,i);
%     end
    for i=1:Nc
        Diff_theta(:,i)=theta_diff(:,i+1)-theta_diff(:,i);
    end
    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);rou_15=yy(13,:);rou_16=yy(16,:);
    rou_21=yy(19,:);rou_22=yy(22,:);rou_23=yy(25,:);rou_24=yy(28,:);rou_25=yy(31,:);rou_26=yy(34,:);
    rou_31=yy(37,:);rou_32=yy(40,:);rou_33=yy(43,:);rou_34=yy(46,:);rou_35=yy(49,:);rou_36=yy(52,:);
    w_o0=yy(56,:);w_o1=yy(58,:);w_o2=yy(60,:);w_o3=yy(62,:);
    TTS=(sum((rou_11+rou_12+rou_13+rou_14+rou_15+rou_16+rou_21+rou_22+rou_23+rou_24+rou_25+rou_26...
        +rou_31+rou_32+rou_33+rou_34+rou_35+rou_36))*Lm/1000*Lambda+sum(w_o0+w_o1+w_o2+w_o3))*T/3600;
%     Diff_speed=Diff(1:dim_speed,:);
%     Diff_r=Diff(dim_speed+1:end,:);
%     penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    penalty=xi_theta*sum(sum(Diff_theta.^2));
    J=TTS+penalty;
end
function [c, ceq]=ModelCons(x, theta, Nc, Np, para,u_pre)
    ceq=[];
    M=para(4);
    weather=para(9);
    rou_crit=para(10);
%     uc=[theta repelem(theta(:, end),1,Np-Nc)];
%     uc_full=repelem(uc,1,M);
    yy=zeros(size(x,1),Np*M);
    theta_full=repelem(theta,1,Np/Nc);
    u_vsl=zeros(6, Np);
    u_rm=zeros(3,Np);
    for i=1:Np
        u=u_para(theta_full(:,i), x, u_pre, rou_crit);
        u_vsl(:,i)=u(1:6);
        u_rm(:,i)=u(7:9);
        for j=1:M
            x=Freeway_model(x,u,weather);
            yy(:,j+M*(i-1))=x;
        end
    end
    w_o0=yy(56,:);w_o1=yy(58,:);
    w_o2=yy(60,:);w_o3=yy(62,:);
    c=[w_o0'-100;w_o1'-100;w_o2'-100;w_o3'-100];
%     u_vsl_full=u_vsl(:);
%     u_rm_full=u_rm(:);
%     c=[u_vsl_full-102;20-u_vsl_full;u_rm_full-1;-u_rm_full];
end









