clear; 
% clc
% close all
%% Initialize
Exit=[];Time=[];
x=[zeros(62,1);0];
ramp_only=1;
u=[102,102,102,102,102,102,102,102,102,1,1,1];
initial_weather=2;
% weather=initial_weather;  % taken as the nominal weather
end_weather=3;

Weather_real=[ones(1,60) initial_weather*ones(1,360) end_weather*ones(1,810)];
Weather_predict=[ones(1,60) initial_weather*ones(1,1170)];
Weather=Weather_predict;
for i=1:60
    weather=Weather(x(63)+1);
    x=Freeway_model(x,u,1);
end

%% Parameters
Mc=6; % control sampling time: 60s
M=6; % operation sampling time: 300s
Ns=M/Mc;
Np=15*Ns; % 
Nc=15*Ns;  % 
N=720;
N_max=40; % multi-start point
xx=zeros(size(x,1),N);
% use nominal parameters to calculate objective values
[tao, kai, yita_high, yita_low, rou_max, sigma, am, v_free, rou_crit, alpha, T, lambda, Lm, ~, v_min, Co2, xi_r, xi_s, ~, ~] = parameters_predict(weather);
dim_speed=9;
dim_ramp=3;
U=zeros(dim_speed+dim_ramp,N);

para=[Lm, lambda, v_free, Mc, dim_speed, T, xi_r, xi_s]; % pass the weather information through this
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);

% v_control=repmat(100*ones(dim_speed,1),1,Nc);
% v_control_pre=v_control(:,1);
if ramp_only
    dim_control=dim_ramp;
    v_control=repmat(102*ones(dim_speed,1),1,Nc);
    v_control_pre=v_control(:,1);
    u_pre=ones(dim_control,Nc); % 
    u_lb=repmat(zeros(dim_control,1),1,Nc);
    u_ub=repmat(ones(dim_control,1),1,Nc);
else
    dim_control=dim_speed+dim_ramp;
    v_control=[];
    v_control_pre=[];
    u_pre=[102*ones(dim_speed,Nc);ones(dim_ramp,Nc)]; % 
    u_lb=repmat([20*ones(dim_speed,1);0*ones(dim_ramp,1)],1,Nc);
    u_ub=repmat([v_free*ones(dim_speed,1);ones(dim_ramp,1)],1,Nc);
end


%%
for i=1:N/M
%     if x(63)>=420
%         weather=end_weather;
%     end
%     para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s, weather];
    objfun=@(co) ModelState(x, [v_control; co], Nc, Np, para, [v_control_pre;u_pre(:,1)], Weather);
    confun=@(co) ModelCons(x, [v_control; co], Nc, Np, Mc, Weather); % Incorrect
    u0=cell(1,N_max);u_t=cell(1,N_max);Fval=nan(1,N_max);Exitflag=nan(1,N_max);
    tic
    for l=1:N_max
        if l==1
            u0{l}=u_pre;
        elseif l==2
            u0{l}=u_lb;
        elseif l==3
            u0{l}=u_ub;
        else
            if ramp_only
                u0{l}=repmat(rand,dim_control, Nc);
            else
                u0{l}=repmat([rand(dim_speed,1)*80+20;rand(dim_ramp,1)],1, Nc);
            end
        end
        [u_t{l}, Fval(l), exitflag] = fmincon(objfun, u0{l}, [], [], [], [], u_lb, u_ub, [], options);
        if exitflag == 0
            warning('the solver does not succeed');
            Fval(l)=Fval(l)+1e6;
        elseif exitflag < 0
            warning('the solver does not succeed');
            Fval(l)=Fval(l)+1e6;
        end
        Exitflag(l)=exitflag;
    end
    time=toc;
    Time=[Time time];
    [fval, index]=min(Fval);
    Exit=[Exit Exitflag'];
    u_opt=[v_control;u_t{index}];
%     u_opt=[v_control;[1 1 1]];
    U(:,M*(i-1)+1:M*i)=repelem(u_opt(:,1:Ns),1,Mc);
    u_imp=repelem(u_opt, 1, Mc);
    for j=1:M
        weather=Weather_real(x(63)+1);
        x=Freeway_model(x,u_imp(:,j),weather); % use real parameters
        xx(:,M*(i-1)+j)=x;
    end
    u_pre=u_t{index};
end
%% figure
u_speed=U(1:dim_speed,:);
u_ramp=U(dim_speed+1:dim_speed+dim_ramp,:);
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
plot(t, demando0(61:780), '-', 'linewidth', 1.5);
hold on;
plot(t, demando1(61:780), '-.', 'linewidth', 1.5);
hold on;
plot(t, demando2(61:780), ':', 'linewidth', 1.5);
hold on;
plot(t, demando3(61:780), '--', 'linewidth', 1.5);
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
plot(t,u_speed,'-', 'linewidth', 1.5);
legend('v-11','v-12','v-13','v-21','v-22','v-23','v-31','v-32','v-33');
xlabel('Time [h]');
ylabel('Speed limits')

subplot(4,2,8)
plot(t,u_ramp,'-','linewidth',1.0);
legend('O_1','O_2','O_3');
ylim([0 1]);
xlabel('Time [h]');
ylabel('Ramp metering')
%%
function J=ModelState(x, u, Nc, Np, para, u_pre, Weather)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    Mc=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,Mc);
    u_diff=[u_pre u];
    yy=nan(size(x,1),Np*Mc);
    Diff=zeros(size(u,1),Nc); % the erros between the successive control inputs
    for i=1:Nc
        Diff(:,i)=u_diff(:,i+1)-u_diff(:,i);
    end
    for i=1:Np*Mc
        weather=Weather(x(63)+1);
        x=Freeway_model_predict(x,uc_full(:,i),weather); % use real parameters
        yy(:,i)=x;
    end
    rou_11=yy(1,:);rou_12=yy(4,:);rou_13=yy(7,:);rou_14=yy(10,:);rou_15=yy(13,:);rou_16=yy(16,:);
    rou_21=yy(19,:);rou_22=yy(22,:);rou_23=yy(25,:);rou_24=yy(28,:);rou_25=yy(31,:);rou_26=yy(34,:);
    rou_31=yy(37,:);rou_32=yy(40,:);rou_33=yy(43,:);rou_34=yy(46,:);rou_35=yy(49,:);rou_36=yy(52,:);
    w_o0=yy(56,:);w_o1=yy(58,:);w_o2=yy(60,:);w_o3=yy(62,:);
    TTS=(sum((rou_11+rou_12+rou_13+rou_14+rou_15+rou_16+rou_21+rou_22+rou_23+rou_24+rou_25+rou_26...
        +rou_31+rou_32+rou_33+rou_34+rou_35+rou_36))*1000/1000*2+sum(w_o0+w_o1+w_o2+w_o3))*10/3600;
    Diff_speed=Diff(1:dim_speed,:);
    Diff_r=Diff(dim_speed+1:end,:);
    penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
    J=TTS;
end
function [c, ceq]=ModelCons(x, u, Nc, Np, Mc, Weather)
    ceq=[];
    uc=[u repelem(u(:, end),1,Np-Nc)];
    uc_full=repelem(uc,1,Mc);
    yy=zeros(size(x,1),Np*Mc);
    for i=1:Np*Mc
        weather=Weather(x(63)+1);
        x=Freeway_model(x,uc_full(:,i), weather);
        yy(:,i)=x;
    end
    w_o0=yy(56,:);w_o1=yy(58,:);
    w_o2=yy(60,:);w_o3=yy(62,:);
    c=[w_o1'-100;w_o2'-100;w_o3'-100];
end









