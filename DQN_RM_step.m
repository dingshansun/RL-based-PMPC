function [Observation,Reward,IsDone,LoggedSignals] = DQN_RM_step(u_rou,LoggedSignals)
k=LoggedSignals.k;
x=LoggedSignals.x;
Weather_real=LoggedSignals.Weather_real;
Weather_predict=LoggedSignals.Weather_predict;
u_pre=LoggedSignals.u_pre;
theta_mpc=LoggedSignals.theta_mpc;
% rou_crit=33.5;
xx=zeros(63,180);
% u_theta=sat_theta(u_theta); % define sat_theta
% step=k-60;
% weather=(end_weather-initial_weather)/720*step+initial_weather;
% implememt u_theta
% weather=initial_weather;
Rou=[15 17.5 20.0 22.5 25.0 27.5 30.0 32.5 35.0 37.5 40.0];
rou_crit=Rou(u_rou+1);
% rou_crit=33.5;
for m=1:6
    theta_mpc=MPC_imp_RM(x, u_pre, theta_mpc, Weather_predict, rou_crit); % new mpc input; fake weather condition
    u_theta=theta_mpc(:,1);
    for i=1:5 % MPC operation time: 5 min
        u_vslrm=u_para_RM(u_theta, x, u_pre, rou_crit);
        % implement u_vslrm
        for j=1:6 % control sampling time: 1min
            x=Freeway_model_predict(x,u_vslrm, Weather_real(k+1)); % use the true weather condition
            k=k+1;
    %         step=k-60;
    %         weather=(end_weather-initial_weather)/720*step+initial_weather;
            xx(:,j+6*(i-1)+30*(m-1))=x;
        end
        u_pre=u_vslrm;
    end
end
% calculate the mpc input for next step

% calculate the reward: TTS
rou_11=xx(1,:);rou_12=xx(4,:);rou_13=xx(7,:);rou_14=xx(10,:);rou_15=xx(13,:);rou_16=xx(16,:);
rou_21=xx(19,:);rou_22=xx(22,:);rou_23=xx(25,:);rou_24=xx(28,:);rou_25=xx(31,:);rou_26=xx(34,:);
rou_31=xx(37,:);rou_32=xx(40,:);rou_33=xx(43,:);rou_34=xx(46,:);rou_35=xx(49,:);rou_36=xx(52,:);
w_o0=xx(56,:);w_o1=xx(58,:);w_o2=xx(60,:);w_o3=xx(62,:);
TTS=(sum((rou_11+rou_12+rou_13+rou_14+rou_15+rou_16+rou_21+rou_22+rou_23+rou_24+rou_25+rou_26...
    +rou_31+rou_32+rou_33+rou_34+rou_35+rou_36))*1000/1000*2+sum(w_o0+w_o1+w_o2+w_o3))*10/3600;
Reward=-TTS;

rou_11=x(1);
v_11=x(2);
q_11=x(3);
rou_12=x(4);
v_12=x(5);
q_12=x(6);
rou_13=x(7);
v_13=x(8);
q_13=x(9);
rou_14=x(10);
v_14=x(11);
q_14=x(12);
rou_15=x(13);
v_15=x(14);
q_15=x(15);
rou_16=x(16);
v_16=x(17);
q_16=x(18);
rou_21=x(19);
v_21=x(20);
q_21=x(21);
rou_22=x(22);
v_22=x(23);
q_22=x(24);
rou_23=x(25);
v_23=x(26);
q_23=x(27);
rou_24=x(28);
v_24=x(29);
q_24=x(30);
rou_25=x(31);
v_25=x(32);
q_25=x(33);
rou_26=x(34);
v_26=x(35);
q_26=x(36);
rou_31=x(37);
v_31=x(38);
q_31=x(39);
rou_32=x(40);
v_32=x(41);
q_32=x(42);
rou_33=x(43);
v_33=x(44);
q_33=x(45);
rou_34=x(46);
v_34=x(47);
q_34=x(48);
rou_35=x(49);
v_35=x(50);
q_35=x(51);
rou_36=x(52);
v_36=x(53);
q_36=x(54);
w_o0=x(56,:);
w_o1=x(58);
w_o2=x(60);
w_o3=x(62);

norm_state=[rou_13 v_13 rou_14 v_14 rou_15 v_15 rou_23 v_23 rou_24 v_24 rou_25 v_25 rou_33 v_33 rou_34 v_34 rou_35 v_35]'/100;
norm_queue=[w_o0;w_o1;w_o2;w_o3]/100;
norm_demand=[demando0(k)/1000; demando1(k)/1000; demando2(k)/1000; demando3(k)/1000];
Observation=[norm_state; norm_demand;norm_queue;u_vslrm(10:12);theta_mpc(:,1);rou_36_down(k)/100;Weather_real(k+1)];

if k>779
    IsDone=true;
else
    IsDone=false;
end

LoggedSignals.k=k;
LoggedSignals.x=x;
LoggedSignals.theta_mpc=theta_mpc;
LoggedSignals.u_pre=u_pre;
LoggedSignals.Weather_real=Weather_real;
LoggedSignals.Weather_predict=Weather_predict;
end

