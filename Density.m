function rou_ = Density(rou, flow_i,flow_up,para)
%DENSITY para=[T,Lm,lambda];
%   
T=para(1)/3600;
Lm=para(2)/1000;
lambda=para(3);
rou_=max(0, rou+T/(Lm*lambda)*(flow_up-flow_i));
end

