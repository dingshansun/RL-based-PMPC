function u = u_para_RM(theta,x,u_pre,rou_crit, v_free)
%Map the states to control input according to theta
%   Detailed explanation goes here
% u_vsl_pre_11=u_pre(1);
% u_vsl_pre_12=u_pre(2);
% u_vsl_pre_21=u_pre(3);
% u_vsl_pre_22=u_pre(4);
% u_vsl_pre_31=u_pre(5);
% u_vsl_pre_32=u_pre(6);
u_rm_pre_1=u_pre(10);
u_rm_pre_2=u_pre(11);
u_rm_pre_3=u_pre(12);

rou_12=x(4);
v_12=x(5);
rou_13=x(7);
v_13=x(8);
rou_14=x(10);
v_14=x(11);
rou_15=x(13);
v_15=x(14);
rou_22=x(22);
v_22=x(23);
rou_23=x(25);
v_23=x(26);
rou_24=x(28);
v_24=x(29);
rou_25=x(31);
v_25=x(32);
rou_32=x(40);
v_32=x(41);
rou_33=x(43);
v_33=x(44);
rou_34=x(46);
v_34=x(47);
rou_35=x(49);
v_35=x(50);
w_o1=x(58);
w_o2=x(60);
w_o3=x(62);

% u_vsl_11=Para_VSL_control_law(theta(1:3), u_vsl_pre_11, rou_13, v_13, rou_14, v_14);
% u_vsl_12=Para_VSL_control_law(theta(1:3), u_vsl_pre_12, rou_14, v_14, rou_15, v_15);
% u_vsl_21=Para_VSL_control_law(theta(1:3), u_vsl_pre_21, rou_23, v_23, rou_24, v_24);
% u_vsl_22=Para_VSL_control_law(theta(1:3), u_vsl_pre_22, rou_24, v_24, rou_25, v_25);
% u_vsl_31=Para_VSL_control_law(theta(1:3), u_vsl_pre_31, rou_33, v_33, rou_34, v_34);
% u_vsl_32=Para_VSL_control_law(theta(1:3), u_vsl_pre_32, rou_34, v_34, rou_35, v_35);
u_rm_1=Para_RM_control_law(theta(1), u_rm_pre_1, rou_crit, rou_15, w_o1);
u_rm_2=Para_RM_control_law(theta(1), u_rm_pre_2, rou_crit, rou_25, w_o2);
u_rm_3=Para_RM_control_law(theta(1), u_rm_pre_3, rou_crit, rou_35, w_o3);

% u_vsl_11=sat_vsl(u_vsl_11);
% u_vsl_12=sat_vsl(u_vsl_12);
% u_vsl_21=sat_vsl(u_vsl_21);
% u_vsl_22=sat_vsl(u_vsl_22);
% u_vsl_31=sat_vsl(u_vsl_31);
% u_vsl_32=sat_vsl(u_vsl_32);
u_vsl_11=102;
u_vsl_12=102;
u_vsl_13=102;
u_vsl_21=102;
u_vsl_22=102;
u_vsl_23=102;
u_vsl_31=102;
u_vsl_32=102;
u_vsl_33=102;
u_rm_1=sat_rm(u_rm_1);
u_rm_2=sat_rm(u_rm_2);
u_rm_3=sat_rm(u_rm_3);
% u_rm_1=1;
% u_rm_2=1;
% u_rm_3=1;

u=[u_vsl_11 u_vsl_12 u_vsl_13 u_vsl_21 u_vsl_22 u_vsl_23 u_vsl_31 u_vsl_32 u_vsl_33 u_rm_1 u_rm_2 u_rm_3]';

end

