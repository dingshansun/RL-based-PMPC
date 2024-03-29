function [InitialObservation,LoggedSignals] = DQN_RM_reset
initial_weather=1;
end_weather=3;
Weather_real=[ones(1,60) initial_weather*ones(1,360) end_weather*ones(1,510)];
Weather_predict=[ones(1,60) initial_weather*ones(1,870)];
dim_theta=1;
Nc=3;
x=[zeros(62,1);0];
u_rm_initial=[1 1 1]';
u_initial=[102; 102; 102; 102; 102; 102; 102; 102; 102; u_rm_initial];
[~, ~, ~, ~, ~, ~, ~, ~, rou_crit, ~, ~, ~, ~, ~, ~, ~, ~, ~,~,~] = parameters_real(initial_weather);
for i=1:60
    x=Freeway_model(x, u_initial, 1);
end
theta_pre=repmat(ones(dim_theta,1), 1, Nc);
u_pre=u_initial;
theta_mpc=MPC_imp_RM(x, u_pre, theta_pre, Weather_real, rou_crit);

k=x(63);
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

LoggedSignals.k=k;
LoggedSignals.x=x;
LoggedSignals.theta_mpc=theta_mpc;
LoggedSignals.u_pre=u_pre;
LoggedSignals.Weather_real=Weather_real;
LoggedSignals.Weather_predict=Weather_predict;
InitialObservation=[norm_state; norm_demand;norm_queue;u_rm_initial;theta_mpc(:,1);rou_36_down(k)/100;Weather_real(k+1)];
end

