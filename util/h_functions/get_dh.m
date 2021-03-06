function dh = get_dh(in1,in2,s,ds,in5,theta_begin,theta_end)
%GET_DH
%    DH = GET_DH(IN1,IN2,S,DS,IN5,THETA_BEGIN,THETA_END)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    10-Jan-2017 16:39:23

alpha1_1 = in5(1);
alpha1_2 = in5(5);
alpha1_3 = in5(9);
alpha1_4 = in5(13);
alpha1_5 = in5(17);
alpha1_6 = in5(21);
alpha1_7 = in5(25);
alpha1_8 = in5(29);
alpha1_9 = in5(33);
alpha2_1 = in5(2);
alpha2_2 = in5(6);
alpha2_3 = in5(10);
alpha2_4 = in5(14);
alpha2_5 = in5(18);
alpha2_6 = in5(22);
alpha2_7 = in5(26);
alpha2_8 = in5(30);
alpha2_9 = in5(34);
alpha3_1 = in5(3);
alpha3_2 = in5(7);
alpha3_3 = in5(11);
alpha3_4 = in5(15);
alpha3_5 = in5(19);
alpha3_6 = in5(23);
alpha3_7 = in5(27);
alpha3_8 = in5(31);
alpha3_9 = in5(35);
alpha4_1 = in5(4);
alpha4_2 = in5(8);
alpha4_3 = in5(12);
alpha4_4 = in5(16);
alpha4_5 = in5(20);
alpha4_6 = in5(24);
alpha4_7 = in5(28);
alpha4_8 = in5(32);
alpha4_9 = in5(36);
alpha1_10 = in5(37);
alpha1_11 = in5(41);
alpha2_10 = in5(38);
alpha2_11 = in5(42);
alpha3_10 = in5(39);
alpha3_11 = in5(43);
alpha4_10 = in5(40);
alpha4_11 = in5(44);
dq8 = in2(8,:);
dq9 = in2(9,:);
dq10 = in2(10,:);
dq11 = in2(11,:);
t2 = s.^2;
t3 = t2.^2;
t4 = t3.^2;
t5 = s-1.0;
t6 = t5.^2;
t7 = t6.^2;
t8 = t7.^2;
t9 = s.*2.0;
t10 = t9-2.0;
dh = [dq8.*(1.0./2.0)+dq9.*(1.0./2.0)+ds.*(alpha1_2.*s.*t8.*9.0e1-alpha1_3.*s.*t8.*9.0e1+alpha1_10.*s.*t4.*1.0e1-alpha1_11.*s.*t4.*1.0e1-alpha1_1.*t5.*t8.*1.0e1+alpha1_2.*t5.*t8.*1.0e1-alpha1_9.*t4.*t10.*4.5e1+alpha1_10.*t4.*t5.*9.0e1+alpha1_6.*s.*t3.*t7.*1.26e3-alpha1_7.*s.*t3.*t7.*1.26e3-alpha1_5.*t3.*t5.*t7.*1.26e3+alpha1_6.*t3.*t5.*t7.*1.26e3+alpha1_4.*s.*t2.*t6.*t7.*8.4e2+alpha1_8.*s.*t2.*t3.*t6.*3.6e2-alpha1_5.*s.*t2.*t6.*t7.*8.4e2-alpha1_9.*s.*t2.*t3.*t6.*3.6e2-alpha1_3.*t2.*t5.*t6.*t7.*3.6e2-alpha1_7.*t2.*t3.*t5.*t6.*8.4e2+alpha1_4.*t2.*t5.*t6.*t7.*3.6e2+alpha1_8.*t2.*t3.*t5.*t6.*8.4e2);dq10.*(1.0./2.0)+dq11.*(1.0./2.0)+ds.*(alpha2_2.*s.*t8.*9.0e1-alpha2_3.*s.*t8.*9.0e1+alpha2_10.*s.*t4.*1.0e1-alpha2_11.*s.*t4.*1.0e1-alpha2_1.*t5.*t8.*1.0e1+alpha2_2.*t5.*t8.*1.0e1-alpha2_9.*t4.*t10.*4.5e1+alpha2_10.*t4.*t5.*9.0e1+alpha2_6.*s.*t3.*t7.*1.26e3-alpha2_7.*s.*t3.*t7.*1.26e3-alpha2_5.*t3.*t5.*t7.*1.26e3+alpha2_6.*t3.*t5.*t7.*1.26e3+alpha2_4.*s.*t2.*t6.*t7.*8.4e2+alpha2_8.*s.*t2.*t3.*t6.*3.6e2-alpha2_5.*s.*t2.*t6.*t7.*8.4e2-alpha2_9.*s.*t2.*t3.*t6.*3.6e2-alpha2_3.*t2.*t5.*t6.*t7.*3.6e2-alpha2_7.*t2.*t3.*t5.*t6.*8.4e2+alpha2_4.*t2.*t5.*t6.*t7.*3.6e2+alpha2_8.*t2.*t3.*t5.*t6.*8.4e2);-dq8+dq9+ds.*(alpha3_2.*s.*t8.*9.0e1-alpha3_3.*s.*t8.*9.0e1+alpha3_10.*s.*t4.*1.0e1-alpha3_11.*s.*t4.*1.0e1-alpha3_1.*t5.*t8.*1.0e1+alpha3_2.*t5.*t8.*1.0e1-alpha3_9.*t4.*t10.*4.5e1+alpha3_10.*t4.*t5.*9.0e1+alpha3_6.*s.*t3.*t7.*1.26e3-alpha3_7.*s.*t3.*t7.*1.26e3-alpha3_5.*t3.*t5.*t7.*1.26e3+alpha3_6.*t3.*t5.*t7.*1.26e3+alpha3_4.*s.*t2.*t6.*t7.*8.4e2+alpha3_8.*s.*t2.*t3.*t6.*3.6e2-alpha3_5.*s.*t2.*t6.*t7.*8.4e2-alpha3_9.*s.*t2.*t3.*t6.*3.6e2-alpha3_3.*t2.*t5.*t6.*t7.*3.6e2-alpha3_7.*t2.*t3.*t5.*t6.*8.4e2+alpha3_4.*t2.*t5.*t6.*t7.*3.6e2+alpha3_8.*t2.*t3.*t5.*t6.*8.4e2);-dq10+dq11+ds.*(alpha4_2.*s.*t8.*9.0e1-alpha4_3.*s.*t8.*9.0e1+alpha4_10.*s.*t4.*1.0e1-alpha4_11.*s.*t4.*1.0e1-alpha4_1.*t5.*t8.*1.0e1+alpha4_2.*t5.*t8.*1.0e1-alpha4_9.*t4.*t10.*4.5e1+alpha4_10.*t4.*t5.*9.0e1+alpha4_6.*s.*t3.*t7.*1.26e3-alpha4_7.*s.*t3.*t7.*1.26e3-alpha4_5.*t3.*t5.*t7.*1.26e3+alpha4_6.*t3.*t5.*t7.*1.26e3+alpha4_4.*s.*t2.*t6.*t7.*8.4e2+alpha4_8.*s.*t2.*t3.*t6.*3.6e2-alpha4_5.*s.*t2.*t6.*t7.*8.4e2-alpha4_9.*s.*t2.*t3.*t6.*3.6e2-alpha4_3.*t2.*t5.*t6.*t7.*3.6e2-alpha4_7.*t2.*t3.*t5.*t6.*8.4e2+alpha4_4.*t2.*t5.*t6.*t7.*3.6e2+alpha4_8.*t2.*t3.*t5.*t6.*8.4e2)];
