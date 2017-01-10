function Fg = ground_force_swing( robot,x,u )
            q=x(1:11);
            dq=x(12:22);
[ER,dER]=robot.get_ER(x);
[D,C,G,B,damping]= robot.Dynamic_model(q,dq);
Fg=(ER*D^-1*ER')^-1*(-dER*dq+ER*D^-1*(C*dq+G+damping-B*u));

end