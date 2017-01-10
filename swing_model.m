function dx = swing_model( t,x )
global robot
global input_torque

dx=zeros(22,1);

q=x(1:11);
dq=x(12:22);

[D,C,G,B,damping]= robot.Dynamic_model(q,dq);

% u=zeros(4,1);
u=input_torque(x);
Fg=ground_force_swing(robot,x,u);
[ER,dER]=robot.get_ER(x);

dx(1:11)=x(12:22);
dx(12:22)=D^-1*(-C*dq-G-damping+B*u+ER'*Fg);
end



