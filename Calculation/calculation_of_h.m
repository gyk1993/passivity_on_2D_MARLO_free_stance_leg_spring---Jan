clear all
M=5;
N=4;
alpha=sym('alpha',[N,M+1],'real');
q=sym('q',[11,1],'real');
dq=sym('dq',[11,1],'real');
syms theta_begin theta_end real
syms s ds real

qs=[q;s];
dqs=[dq;ds];

H0=[0, 0, 0, 0, 0, 0, 0, .5, .5,  0,  0;...
    0, 0, 0, 0, 0, 0, 0,  0,  0, .5, .5;...
    0, 0, 0, 0, 0, 0, 0, -1,  1,  0,  0;...
    0, 0, 0, 0, 0, 0, 0,  0,  0, -1,  1];
c =[0, 0, -1, -.5, -.5, 0, 0, 0, 0, 0, 0];
% c =[0, 0, -1,0, 0, 0, 0, -.5, -.5, 0,  0];

h0=H0*q;
theta=c*q;

hd=sym(zeros(4,1));
for i=1:N
    hd(i)=0;
    for k=0:M
        hd(i)= hd(i) + alpha(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * s^k * (1-s)^(M-k);
    end
end


h=h0-hd;
dh=jacobian(h,qs)*dqs;
dh0=jacobian(h0,qs)*dqs;
dhd=jacobian(hd,qs)*dqs;

dhdq=jacobian(h,qs);
d_dhdq=jacobian(dhdq*dqs,qs); %d_dhdq=d/dq(dh/dq*qdot)


mkdir('../util/h_functions')
matlabFunction(h, 'file', ['../util/h_functions/get_h.m'], 'vars', {q,dq,s,ds,alpha,theta_begin,theta_end});
matlabFunction(h0, 'file', ['../util/h_functions/get_h0.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(hd, 'file', ['../util/h_functions/get_hd.m'], 'vars', {q,dq,s,ds,alpha,theta_begin,theta_end});

matlabFunction(dh, 'file', ['../util/h_functions/get_dh.m'], 'vars', {q,dq,s,ds,alpha,theta_begin,theta_end});
matlabFunction(dh0, 'file', ['../util/h_functions/get_dh0.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(dhd, 'file', ['../util/h_functions/get_dhd.m'], 'vars', {q,dq,s,ds,alpha,theta_begin,theta_end});

