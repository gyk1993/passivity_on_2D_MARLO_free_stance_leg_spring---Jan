clear all
syms qT yLeg zLeg dqT dyLeg dzLeg real
syms q1R  q2R  dq1R dq2R real
syms q1L q2L dq1L dq2L real
syms qgr1R  qgr2R  dqgr1R  dqgr2R real    
syms qgr1L qgr2L dqgr1L dqgr2L real


syms L1 L2 L3 L4 real


q=[yLeg;zLeg;qT;q1R;q2R;q1L;q2L;qgr1R;qgr2R;qgr1L;qgr2L];
dq=[dyLeg;dzLeg;dqT;dq1R;dq2R;dq1L;dq2L;dqgr1R;dqgr2R;dqgr1L;dqgr2L];
ER=[ 1, 0
    0, 1
    0, 0
    0, 0
    0, 0
    0, 0
    0, 0
    0, 0
    0, 0
    0, 0
    0, 0];


EL=[                                                                  1,                                                                       0
    0,                                                                       1
    L2*cos(q2R + qT) + L4*cos(q1R + qT) - L2*cos(q2L + qT) - L4*cos(q1L + qT), L2*sin(q2R + qT) + L4*sin(q1R + qT) - L2*sin(q2L + qT) - L4*sin(q1L + qT)
    L4*cos(q1R + qT),                                                         L4*sin(q1R + qT)
    L2*cos(q2R + qT),                                                         L2*sin(q2R + qT)
    -L4*cos(q1L + qT),                                                       -L4*sin(q1L + qT)
    -L2*cos(q2L + qT),                                                       -L2*sin(q2L + qT)
    0,                                                                       0
    0,                                                                       0
    0,                                                                       0
    0,                                                                       0];
EL=EL';
ER=ER';

v_stance=ER*dq;
v_swing=EL*dq;
dER=jacobian(v_stance,q);
dEL=jacobian(v_swing,q);

% dER=[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
%      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% dEL=[ 0, 0, dqT*(L2*sin(q2L + qT) + L4*sin(q1L + qT) - L2*sin(q2R + qT) - L4*sin(q1R + qT)) + L2*dq2L*sin(q2L + qT) + L4*dq1L*sin(q1L + qT) - L2*dq2R*sin(q2R + qT) - L4*dq1R*sin(q1R + qT), - L4*dq1R*sin(q1R + qT) - L4*dqT*sin(q1R + qT), - L2*dq2R*sin(q2R + qT) - L2*dqT*sin(q2R + qT),   L4*dq1L*sin(q1L + qT) + L4*dqT*sin(q1L + qT),   L2*dq2L*sin(q2L + qT) + L2*dqT*sin(q2L + qT), 0, 0, 0, 0
%      0, 0, L2*dq2R*cos(q2R + qT) - L2*dq2L*cos(q2L + qT) - L4*dq1L*cos(q1L + qT) - dqT*(L2*cos(q2L + qT) + L4*cos(q1L + qT) - L2*cos(q2R + qT) - L4*cos(q1R + qT)) + L4*dq1R*cos(q1R + qT),   L4*dq1R*cos(q1R + qT) + L4*dqT*cos(q1R + qT),   L2*dq2R*cos(q2R + qT) + L2*dqT*cos(q2R + qT), - L4*dq1L*cos(q1L + qT) - L4*dqT*cos(q1L + qT), - L2*dq2L*cos(q2L + qT) - L2*dqT*cos(q2L + qT), 0, 0, 0, 0];
