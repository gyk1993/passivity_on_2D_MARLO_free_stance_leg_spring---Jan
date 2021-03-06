function u= input_torque_IOL( x )
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
u=LgLfh^-1*(-LfLfh+ddh);

% for i=1:4
%     if u(i)>40
%         u(i)=40;
%     end
%     if u(i)<-40
%         u(i)=-40;
%     end
% end
end

