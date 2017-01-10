clear all
M=5;
N=4;
alpha=sym('alpha',[N,M+1],'real');
q=sym('q',[11,1],'real');
dq=sym('dq',[11,1],'real');
syms theta_begin theta_end real

H0=[0, 0, 0, 0, 0, 0, 0, .5, .5,  0,  0;...
    0, 0, 0, 0, 0, 0, 0,  0,  0, .5, .5;...
    0, 0, 0, 0, 0, 0, 0, -1,  1,  0,  0;...
    0, 0, 0, 0, 0, 0, 0,  0,  0, -1,  1];
c =[0, 0, -1, -.5, -.5, 0, 0, 0, 0, 0, 0];
% c =[0, 0, -1,0, 0, 0, 0, -.5, -.5, 0,  0];

h0=H0*q;
theta=c*q;
s=(theta-theta_begin)/(theta_end-theta_begin);
hd=sym(zeros(4,1));
for i=1:N
    hd(i)=0;
    for k=0:M
        hd(i)= hd(i) + alpha(i,k+1) * (factorial(M)/(factorial(k)*factorial(M-k))) * s^k * (1-s)^(M-k);
    end
end

h=h0-hd;
dh=jacobian(h,q)*dq;
dh0=jacobian(h0,q)*dq;
dhd=jacobian(hd,q)*dq;

dhdq=jacobian(h,q);
d_dhdq=jacobian(dhdq*dq,q); %d_dhdq=d/dq(dh/dq*qdot)

dh0dq=jacobian(h0,q);
d_dh0dq=jacobian(dh0dq*dq,q);

dhddq=jacobian(hd,q);
d_dhddq=jacobian(dhddq*dq,q);

mkdir('h_functions')
matlabFunction(d_dhdq, 'file', ['../util/h_functions/get_d_dhdq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(dhdq, 'file', ['../util/h_functions/get_dhdq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(d_dh0dq, 'file', ['../util/h_functions/get_d_dh0dq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(dh0dq, 'file', ['../util/h_functions/get_dh0dq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(d_dhddq, 'file', ['../util/h_functions/get_d_dhddq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});
matlabFunction(dhddq, 'file', ['../util/h_functions/get_dhddq.m'], 'vars', {q,dq,alpha,theta_begin,theta_end});



