function x_=Freeway_model_predict(x, u, weather)
%% Initial conditions
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
q_o0=x(55);
w_o0=x(56);
q_o1=x(57);
w_o1=x(58);
q_o2=x(59);
w_o2=x(60);
q_o3=x(61);
w_o3=x(62);
k=x(63);

%% Noise
noise_15=0;
noise_25=0;
noise_35=0;
% if rou_15<=30
%     noise_15=random('Normal',-1,1);
% elseif rou_15>30 && rou_15<40
%     noise_15=random('Normal',-3,2);
% elseif rou_15>=40
%     noise_15=random('Normal',-5,3);
% end
% if rou_25<=30
%     noise_25=random('Normal',-1,1);
% elseif rou_25>30 && rou_25<40
%     noise_25=random('Normal',-3,2);
% elseif rou_25>=40
%     noise_25=random('Normal',-5,3);
% end
% if rou_35<=30
%     noise_35=random('Normal',-1,1);
% elseif rou_35>30 && rou_35<40
%     noise_35=random('Normal',-3,2);
% elseif rou_35>=40
%     noise_35=random('Normal',-5,3);
% end
%% Parameters
% if nominal ==1
%     [tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, v_control, v_min, Co, ~, ~, ~,off_rate] = parameters;
% else
[tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, v_control, v_min, Co, ~, ~, ~,off_rate] = parameters_predict(weather);
% end
para_speed=[T,tao,yita_high, yita_low, kai,Lm,sigma,lambda];
para_density=[T,Lm,lambda];
para_desire=[v_free, am, rou_crit, alpha];
para_outmain=[T, lambda, rou_crit, am, v_free, alpha];
para_outramp=[T, rou_max, rou_crit];
% Initial state of the source
% q_o1(1)=Outflow_main(demando1(1), v_free, w_o1(1), v_control, para_outmain);
% q_o2(1)=Outflow_ramp(demando2(1), w_o2(1), Co2, ro, rou_21(1), para_outramp);
% v_11(1)=v_free;
% v_12(1)=v_free;
% v_13(1)=v_free;
% v_14(1)=v_free;
% v_21(1)=v_free;
% v_22(1)=v_free;
%% Control input
u_v1=u(1);
u_v2=u(2);
u_v3=u(3);
u_v4=u(4);
u_v5=u(5);
u_v6=u(6);
u_v7=u(7);
u_v8=u(8);
u_v9=u(9);
r_o1=u(10);
r_o2=u(11);
r_o3=u(12);
% r_o1=1;
% r_o2=1;
% r_o3=1;
%% State evolution
    % Segment 1,1
    rou_11_=Density(rou_11, q_11, q_o0, para_density);
    V_desire_11=Desired_speed(rou_11, para_desire, v_control);
    v_11_=Speed(v_11, V_desire_11, v_11, rou_11, rou_12, para_speed, 0, v_min);
    q_11_=rou_11_*v_11_*lambda;
    % Segment 1,2
    rou_12_=Density(rou_12, q_12, q_11, para_density);
    V_desire_12=Desired_speed(rou_12, para_desire, u_v1);
    v_12_=Speed(v_12, V_desire_12, v_11, rou_12, rou_13, para_speed, 0, v_min);
    q_12_=rou_12_*v_12_*lambda;
    % Segment 1,3
    rou_13_=Density(rou_13, q_13, q_12, para_density);
    V_desire_13=Desired_speed(rou_13, para_desire, u_v2);
    v_13_=Speed(v_13, V_desire_13, v_12, rou_13, rou_14, para_speed, 0, v_min);
    q_13_=rou_13_*v_13_*lambda;
    % Segment 1,4
    rou_14_=Density(rou_14, q_14, q_13*off_rate, para_density);
    V_desire_14=Desired_speed(rou_14, para_desire, u_v3);
    v_14_=Speed(v_14, V_desire_14, v_13, rou_14, rou_15, para_speed, 0, v_min);
    q_14_=rou_14_*v_14_*lambda;
    % Segment 1,5
    rou_15_=Density(rou_15, q_15, q_14+q_o1, para_density);
    V_desire_15=Desired_speed(rou_15, para_desire, v_control);
    v_15_=Speed(v_15, V_desire_15, v_14, rou_15, rou_16, para_speed, q_o1, v_min)+noise_15;
    q_15_=rou_15_*v_15_*lambda;
    % Segment 1,6
    rou_16_=Density(rou_16, q_16, q_15, para_density);
    V_desire_16=Desired_speed(rou_16, para_desire, v_control);
    v_16_=Speed(v_16, V_desire_16, v_15, rou_16, rou_21, para_speed, 0, v_min);
    q_16_=rou_16_*v_16_*lambda;
    % Segment 2,1
    rou_21_=Density(rou_21, q_21, q_16, para_density);
    V_desire_21=Desired_speed(rou_21, para_desire, v_control);
    v_21_=Speed(v_21, V_desire_21, v_16, rou_21, rou_22, para_speed, 0, v_min);
    q_21_=rou_21_*v_21_*lambda;
    % Segment 2,2
    rou_22_=Density(rou_22, q_22, q_21, para_density);
    V_desire_22=Desired_speed(rou_22, para_desire, u_v4);
    v_22_=Speed(v_22, V_desire_22, v_21, rou_22, rou_23, para_speed, 0, v_min);
    q_22_=rou_22_*v_22_*lambda;
    % Segment 2,3
    rou_23_=Density(rou_23, q_23, q_22, para_density);
    V_desire_23=Desired_speed(rou_23, para_desire, u_v5);
    v_23_=Speed(v_23, V_desire_23, v_22, rou_23, rou_24, para_speed, 0, v_min);
    q_23_=rou_23_*v_23_*lambda;
    % Segment 2,4
    rou_24_=Density(rou_24, q_24, q_23*off_rate, para_density);
    V_desire_24=Desired_speed(rou_24, para_desire, u_v6);
    v_24_=Speed(v_24, V_desire_24, v_23, rou_24, rou_25, para_speed, 0, v_min);
    q_24_=rou_24_*v_24_*lambda;
    % Segment 2,5
    rou_25_=Density(rou_25, q_25, q_24+q_o2, para_density);
    V_desire_25=Desired_speed(rou_25, para_desire, v_control);
    v_25_=Speed(v_25, V_desire_25, v_24, rou_25, rou_26, para_speed, q_o2, v_min)+noise_25;
    q_25_=rou_25_*v_25_*lambda;
    % Segment 2,6
    rou_26_=Density(rou_26, q_26, q_25, para_density);
    V_desire_26=Desired_speed(rou_26, para_desire, v_control);
    v_26_=Speed(v_26, V_desire_26, v_25, rou_26, rou_31, para_speed, 0, v_min);
    q_26_=rou_26_*v_26_*lambda;
    % Segment 3,1
    rou_31_=Density(rou_31, q_31, q_26, para_density);
    V_desire_31=Desired_speed(rou_31, para_desire, v_control);
    v_31_=Speed(v_31, V_desire_31, v_26, rou_31, rou_32, para_speed, 0, v_min);
    q_31_=rou_31_*v_31_*lambda;
    % Segment 3,2
    rou_32_=Density(rou_32, q_32, q_31, para_density);
    V_desire_32=Desired_speed(rou_32, para_desire, u_v7);
    v_32_=Speed(v_32, V_desire_32, v_31, rou_32, rou_33, para_speed, 0, v_min);
    q_32_=rou_32_*v_32_*lambda;
    % Segment 3,3
    rou_33_=Density(rou_33, q_33, q_32, para_density);
    V_desire_33=Desired_speed(rou_33, para_desire, u_v8);
    v_33_=Speed(v_33, V_desire_33, v_32, rou_33, rou_34, para_speed, 0, v_min);
    q_33_=rou_33_*v_33_*lambda;
    % Segment 3,4
    rou_34_=Density(rou_34, q_34, q_33*off_rate, para_density);
    V_desire_34=Desired_speed(rou_34, para_desire, u_v9);
    v_34_=Speed(v_34, V_desire_34, v_33, rou_34, rou_35, para_speed, 0, v_min);
    q_34_=rou_34_*v_34_*lambda;
    % Segment 3,5
    rou_35_=Density(rou_35, q_35, q_34+q_o3, para_density);
    V_desire_35=Desired_speed(rou_35, para_desire, v_control);
    v_35_=Speed(v_35, V_desire_35, v_34, rou_35, rou_36, para_speed, q_o3, v_min)+noise_35;
    q_35_=rou_35_*v_35_*lambda;
    % Segment 3,6
    rou_36_=Density(rou_36, q_36, q_35, para_density);
    V_desire_36=Desired_speed(rou_36, para_desire, v_control);
    v_36_=Speed(v_36, V_desire_36, v_35, rou_36, rou_36_down(k), para_speed, 0, v_min);
    q_36_=rou_36_*v_36_*lambda;
    % Source nodes
    % o0
    w_o0_=w_o0+T/3600*(demando0(k-1)-q_o0);
    q_o0_=Outflow_main(demando0(k), v_11_, w_o0_, v_control, para_outmain);
    % o1
    w_o1_=w_o1+T/3600*(demando1(k-1)-q_o1);
    q_o1_=Outflow_ramp(demando1(k), w_o1_, Co, r_o1, rou_15_, para_outramp);
    % o2
    w_o2_=w_o2+T/3600*(demando2(k-1)-q_o2);
    q_o2_=Outflow_ramp(demando2(k), w_o2_, Co, r_o2, rou_25_, para_outramp);
    % o3
    w_o3_=w_o3+T/3600*(demando3(k-1)-q_o3);
    q_o3_=Outflow_ramp(demando3(k), w_o3_, Co, r_o3, rou_35_, para_outramp);

x_=[rou_11_;
v_11_;
q_11_;
rou_12_;
v_12_;
q_12_;
rou_13_;
v_13_;
q_13_;
rou_14_;
v_14_;
q_14_;
rou_15_;
v_15_;
q_15_;
rou_16_;
v_16_;
q_16_;
rou_21_;
v_21_;
q_21_;
rou_22_;
v_22_;
q_22_;
rou_23_;
v_23_;
q_23_;
rou_24_;
v_24_;
q_24_;
rou_25_;
v_25_;
q_25_;
rou_26_;
v_26_;
q_26_;
rou_31_;
v_31_;
q_31_;
rou_32_;
v_32_;
q_32_;
rou_33_;
v_33_;
q_33_;
rou_34_;
v_34_;
q_34_;
rou_35_;
v_35_;
q_35_;
rou_36_;
v_36_;
q_36_;
q_o0_;
w_o0_;
q_o1_;
w_o1_;
q_o2_;
w_o2_;
q_o3_;
w_o3_;
k+1];
end
% fprintf('TTS is %d veh*h \n', sum(TTS(60:960)))
% figure();
% t=0:1/360:2.5;
% subplot(3,2,1)
% plot(t, v_11(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, v_12(61:961), '--', 'linewidth', 1.0);
% hold on;
% plot(t, v_13(61:961), ':', 'linewidth', 1.0);
% hold on;
% plot(t, v_14(61:961), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, v_21(61:961), '-o', 'MarkerIndices',1:30:length(v_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, v_22(61:961), '-x',  'MarkerIndices',1:30:length(v_22(61:961)), 'linewidth', 1.0);
% legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Speed [km/h]')
% 
% subplot(3,2,2)
% plot(t, q_11(60:960), '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_12(60:960), '--', 'linewidth', 1.0);
% hold on;
% plot(t, q_13(60:960), ':', 'linewidth', 1.0);
% hold on;
% plot(t, q_14(60:960), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, q_21(60:960), '-o', 'MarkerIndices',1:30:length(q_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, q_22(60:960), '-x',  'MarkerIndices',1:30:length(q_22(61:961)), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Flow [veh/h]')
% 
% subplot(3,2,3)
% t=0:1/360:2.5;
% plot(t, rou_11(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, rou_12(61:961), '--', 'linewidth', 1.0);
% hold on;
% plot(t, rou_13(61:961), ':', 'linewidth', 1.0);
% hold on;
% plot(t, rou_14(61:961), '-.', 'linewidth', 1.0);
% hold on;
% plot(t, rou_21(61:961), '-o', 'MarkerIndices',1:30:length(rou_21(61:961)), 'linewidth', 1.0);
% hold on;
% plot(t, rou_22(61:961), '-x',  'MarkerIndices',1:30:length(rou_22(61:961)), 'linewidth', 1.0);
% % legend('Segmeng 1-1','Segmeng 1-2','Segmeng 1-3','Segmeng 1-4','Segmeng 2-1','Segmeng 2-2')
% xlabel('Time [h]');
% ylabel('Density [veh/km]')
% 
% subplot(3,2,5)
% plot(t, q_o1(61:961), '-', 'linewidth', 1.0);
% hold on;
% plot(t, q_o2(61:961), '--', 'linewidth', 1.0);
% legend('O_1','O_2')
% 
% subplot(3,2,6)
% plot(t,w_o1(61:961),'-', 'linewidth', 1.0);
% hold on;
% plot(t,w_o2(61:961),'--', 'linewidth', 1.0);
% legend('O_1','O_2')

