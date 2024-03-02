function V = Desired_speed(rou,para,v_control)
%DESIRED_SPEED input: density of current segment, speed control, alpha, am,v_free, rou_crit; output: the desired speed; para=[v_free, am, rou_crit, alpha];
v_free=para(1);
am=para(2);
rou_crit=para(3);
alpha=para(4);
V=min([v_free*exp(-1/am*(rou/rou_crit)^am), (1+alpha)*v_control]);
end

