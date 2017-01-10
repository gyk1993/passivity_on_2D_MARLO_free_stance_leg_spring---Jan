function [value,isterminal,direction] = impactevent( t,x )
global robot
dx=zeros(14,1);

q=x(1:7);
dq=x(8:14);
q1=x(1); dq1=x(8);
q2=x(2); dq2=x(9);
q3=x(3); dq3=x(10);
q4=x(4); dq4=x(11);
q5=x(5); dq5=x(12);
q6=x(6); dq6=x(13);
q7=x(7); dq7=x(14);

yH=x(1);
zH=x(2);
qT=x(3);
q1R=x(4);
q2R=x(5);
q1L=x(6);
q2L=x(7);

L1=robot.L1; L2=robot.L2; L3=robot.L3; L4=robot.L4;

[pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(x);

if p4L(2)<0.001 && p4L(1)>p4R(1)
    value=0;
else
    value=1;
end
isterminal=1;
direction=0;
end

