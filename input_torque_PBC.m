function u= input_torque_PBC( x )
global robot
global alpha
global theta_begin theta_end
global Kd Kp
q=x(1:11);
dq=x(12:22);

[s ds] =get_s_and_ds(q,dq,theta_begin,theta_end);

h=get_h(q,dq,s,ds,alpha,theta_begin,theta_end);
dh=get_dh(q,dq,s,ds,alpha,theta_begin,theta_end);


ddh=-Kd*dh-Kp*h;
[ LfLfh LgLfh ] = get_LfLfh_LgLfh( q,dq,alpha,theta_begin,theta_end,robot );
[D_hat,C_hat,Omega_hat,B2]=robot.get_PBC_part(q,dq);

u=-(D_hat*LgLfh)^-1*((C_hat+Kd)*dh+Kp*h+D_hat*LfLfh);

 end