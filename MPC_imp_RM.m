function u_theta = MPC_imp_RM(x, u_pre, theta_pre, Weather, rou_crit)
    Np=3;
    Nc=3;
    M=30;
    Mc=6;
    N_max=40; % multi-start point
    dim_speed=9;
    % dim_ramp=3;
    dim_theta=1;
    % fake weather condition; rou_crit can be adjusted
    weather=Weather(x(63)+1);
    [~, ~, ~, ~, ~, ~, ~, v_free, ~, ~, T, lambda, Lm, ~, ~, ~, xi_r, xi_s,xi_theta,~] = parameters_predict(weather);
    para=[Lm, lambda, v_free, M, dim_speed, T, xi_r, xi_s, weather, rou_crit, Mc, xi_theta];
    options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2);
    theta_lb=repmat(-5,dim_theta,Nc);
    theta_ub=repmat(5,dim_theta,Nc);
    objfun=@(theta) ModelState(x, theta, Nc, Np, para, u_pre(:,1), theta_pre(:,1), Weather);
    confun=@(theta) ModelCons(x, theta, Nc, Np, para, u_pre(:,1), Weather);
    theta0=cell(1,N_max);theta_t=cell(1,N_max);Fval=nan(1,N_max);
    parfor l=1:N_max
        if l==1
            theta0{l}=theta_pre;
        elseif l==2
            theta0{l}=theta_lb;
        elseif l==3
            theta0{l}=theta_ub;
        elseif l==4
            theta0{l}=(theta_lb+theta_ub)/2;
        else
    %             theta0{l}=repmat([2*rand;200*rand-100;200*rand-100;200*rand],1, Nc);
            theta0{l}=repmat(10*rand-5,dim_theta, Nc);
        end
        [theta_t{l}, Fval(l), exitflag] = fmincon(objfun, theta0{l}, [], [], [], [], theta_lb, theta_ub, [], options);
        if exitflag == 0
            Fval(l)=Fval(l)+1e6;
        elseif exitflag < 0
            Fval(l)=Fval(l)+1e6;
        end
    end
    [~, index]=min(Fval);
    u_theta=theta_t{index};
end

function J=ModelState(x, theta, Nc, Np, para, u_pre, theta_pre, Weather)
    Lm=para(1);
    Lambda=para(2);
    v_free=para(3);
    M=para(4);
    dim_speed=para(5);
    T=para(6);
    xi_r=para(7);
    xi_s=para(8);
%     weather=para(9);
    rou_crit=para(10);
    Mc=para(11);
    xi_theta=para(12);
%     uc=[u repelem(u(:, end),1,Np-Nc)];
%     uc_full=repelem(uc,1,M);
    theta_full=repelem(theta,1,Np/Nc);
    yy=[];
%     Diff=zeros(size(u_pre,1),Np); % the erros between the successive control inputs
    Diff_theta=zeros(size(theta_pre,1),Nc);
    uc_full=zeros(size(u_pre,1),Np);
    for i=1:Np
        for j=1:M/Mc
            u=u_para_RM(theta_full(:,i), x, u_pre, rou_crit);
            uc_full(:,i)=u;
            for k=1:Mc
                weather=Weather(x(63)+1);
                x=Freeway_model_predict(x,u,weather);
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
        +rou_31+rou_32+rou_33+rou_34+rou_35+rou_36))*1000/1000*2+sum(w_o0+w_o1+w_o2+w_o3))*T/3600;
%     Diff_speed=Diff(1:dim_speed,:);
%     Diff_r=Diff(dim_speed+1:end,:);
%     penalty=xi_r*sum(sum(Diff_r.^2))+xi_s*sum(sum((Diff_speed/v_free).^2));
%     penalty=xi_theta*sum(sum(Diff_theta.^2));
    J=TTS;
end

function [c, ceq]=ModelCons(x, theta, Nc, Np, para,u_pre, Weather)
    ceq=[];
    M=para(4);
%     weather=para(9);
    rou_crit=para(10);
    Mc=para(11);
%     uc=[u repelem(u(:, end),1,Np-Nc)];
%     uc_full=repelem(uc,1,M);
    theta_full=repelem(theta,1,Np/Nc);
    yy=[];
    uc_full=zeros(size(u_pre,1),Np);
    for i=1:Np
        for j=1:M/Mc
            u=u_para_RM(theta_full(:,i), x, u_pre, rou_crit);
            uc_full(:,i)=u;
            for k=1:Mc
                weather=Weather(x(63)+1);
                x=Freeway_model(x,u,weather);
                yy=[yy x];
            end
            u_pre=u;
        end
    end
    w_o0=yy(56,:);w_o1=yy(58,:);
    w_o2=yy(60,:);w_o3=yy(62,:);
    c=[w_o1'-100;w_o2'-100;w_o3'-100];
%     u_vsl_full=u_vsl(:);
%     u_rm_full=u_rm(:);
%     c=[u_vsl_full-102;20-u_vsl_full;u_rm_full-1;-u_rm_full];
end