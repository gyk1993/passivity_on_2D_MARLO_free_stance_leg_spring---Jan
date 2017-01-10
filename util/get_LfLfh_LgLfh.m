function [ LfLfh LgLfh ] = get_LfLfh_LgLfh( q,dq,alpha,theta_begin,theta_end,robot )


dhdq=get_dhdq(q,dq,alpha,theta_begin,theta_end);
d_dhdq=get_d_dhdq(q,dq,alpha,theta_begin,theta_end);

[D,C,G,B,damping]= robot.Dynamic_model(q,dq);
[ER,dER]=robot.get_ER([q;dq]);
M=[D -ER'; ER zeros(2,2)];
H=[C*dq+G+damping;dER*dq];
temp=dhdq*[eye(11) zeros(11,2)]*M^-1;

LfLfh=d_dhdq*dq-temp*H;
LgLfh=temp*[B;zeros(2,4)];

end

