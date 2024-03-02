function u_rm = Para_RM_control_law(theta, u_rm_pre, rou_crit, rou,w)
%PARAMETERIZED_CONTROL_LAW Summary of this function goes here
%   Detailed explanation goes here
u_rm=u_rm_pre+theta(1)*(rou_crit-rou);
end

