function outflow = Outflow_ramp(demand,wo,Co,ro,rou,para)
%OUTFLOW para=[T,rou_max,rou_crit];
%   Detailed explanation goes here
T=para(1)/3600;
rou_max=para(2);
rou_crit=para(3);
outflow=min([demand+wo/T, Co*ro, Co*(rou_max-rou)/(rou_max-rou_crit)]);
end

