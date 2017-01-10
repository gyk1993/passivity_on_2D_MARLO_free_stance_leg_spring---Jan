function x_a = impact_model(x)
global robot

x_a=zeros(22,1);
q=x(1:11);
dq=x(12:22);
% q1=x(1); dq1=x(8);
% q2=x(2); dq2=x(9);
% q3=x(3); dq3=x(10);
% q4=x(4); dq4=x(11);
% q5=x(5); dq5=x(12);
% q6=x(6); dq6=x(13);
% q7=x(7); dq7=x(14);
% 
% qT=x(3);
% q1R=x(4);
% q2R=x(5);
% q1L=x(6);
% q2L=x(7);

L1=robot.L1; L2=robot.L2; L3=robot.L3; L4=robot.L4;

[EL,~]=robot.get_EL(x);

[D,C,G,B,~]= robot.Dynamic_model(q,dq);
[pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(x);

x_a(1:11)=x(1:11);

M=[D -EL'; EL zeros(2,2)];
temp=M^-1*[D*dq;zeros(2,1)]; % the first seven element in temp is dq, last 2 is Fg]
x_a(12:22)=temp(1:11);

%switch leg
x_a([4,5,6,7])=x_a([6,7,4,5]);
x_a([15,16,17,18])=x_a([17,18,15,16]);

%switch leg spring
x_a([8,9,10,11])=x_a([10,11,8,9]);
x_a([19,20,21,22])=x_a([21,22,19,20]);


x_a([1 2])=p4L;
x_a([12 13])=[0;0];
end

