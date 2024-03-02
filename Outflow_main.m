function outflow = Outflow_main(demand,V,wo,v_control,para)
%OUTFLOW_MAIN para=[T, lambda, rou_crit, am, v_free, alpha];
%   
T=para(1)/3600;
lambda=para(2);
rou_crit=para(3);
am=para(4);
v_free=para(5);
alpha=para(6);
V_desire=Desired_speed(rou_crit, [v_free, am, rou_crit, alpha], v_control);
q_cap=lambda*V_desire*rou_crit;
v_lim=min([v_control, V]);
q_speed=lambda*v_lim*rou_crit*(-am*log(v_lim/v_free))^(1/am);
q_lim=q_speed*(v_lim<V_desire) + q_cap*(v_lim>=V_desire);
outflow=min([demand+wo/T, q_lim]);
end

