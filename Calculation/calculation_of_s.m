clear all
q=sym('q',[11,1],'real');
dq=sym('dq',[11,1],'real');
syms theta_begin theta_end real

c =[0, 0, -1, -.5, -.5, 0, 0, 0, 0, 0, 0];
theta=c*q;
dtheta=c*dq;
s=(theta-theta_begin)/(theta_end-theta_begin);
ds=dtheta/(theta_end-theta_begin);

matlabFunction(s,'file',['../util/get_s.m'],'vars',{q,dq,theta_begin,theta_end});
matlabFunction(ds,'file',['../util/get_ds.m'],'vars',{q,dq,theta_begin,theta_end});
