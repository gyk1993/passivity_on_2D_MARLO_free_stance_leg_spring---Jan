function gen_q = world2gen( q )
global robot
L1=robot.L1; L2=robot.L2; L3=robot.L3; L4=robot.L4;
%q=[sy,sz,qT,LR,KR,LL,KL]; L is the leg angle wrt to vertical
q=num2cell(q);
[qT,LR,KR,LL,KL]=q{:};
qT=-qT;

q1R=pi-LR-qT-1/2*KR;
q2R=pi-LR-qT+1/2*KR;
q1L=pi-LL-qT-1/2*KL;
q2L=pi-LL-qT+1/2*KL;
gen_q=[qT;q1R;q2R;q1L;q2L];
end

