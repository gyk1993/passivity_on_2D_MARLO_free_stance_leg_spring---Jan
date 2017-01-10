function [ LfLfh0 LgLfh0 ] = get_LfLfh0_LgLfh0( q,dq,alpha,theta_begin,theta_end,robot )
dh0dq=get_dh0dq(q,dq,alpha,theta_begin,theta_end);
d_dh0dq=get_d_dh0dq(q,dq,alpha,theta_begin,theta_end);

[D,C,G,B,damping]= robot.Dynamic_model(q,dq);
[ER,dER]=robot.get_ER([q;dq]);
M=[D -ER'; ER zeros(2,2)];
H=[C*dq+G+damping;dER*dq];
temp=dh0dq*[eye(11) zeros(11,2)]*M^-1;

LfLfh0=d_dh0dq*dq-temp*H;
LgLfh0=temp*[B;zeros(2,4)];

end