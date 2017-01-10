close all
clear all
addpath('./plot_functions');
addpath('./util');
addpath('./util/h_functions_M=10');
global robot
global Kd Kp
global alpha
global theta_begin theta_end
global input_torque

input_torque=@input_torque_IOL;

% Kd=200*eye(4);
% Kp=(Kd^2/5)*eye(4);

Kd=150*eye(4);
Kp=(Kd^2/5)*eye(4);


%% First set
% theta_begin=-3.217813187444452;
% theta_end=-2.994025326903608;
% world_IC=[1/24*pi;0;1/16*pi;-1/12*pi;1/12*pi];
% alpha=[3.098215357	3.143666885	2.884815149	3.088121611	2.766973163	2.874927356
% 2.874927624	2.955995185	2.971755178	3.027283653	3.104731441	3.098715267
% 0.91895479	0.790295577	0.897006944	0.966188695	0.790971235	0.885910591
% 0.886909585	0.953977949	1.540193613	0.703512705	2.091021156	0.918954931];

%%  Second Set
% theta_begin=-3.345596906275392;
% theta_end=-2.941398872178075;
% world_IC=[1/24*pi;0;1/20*pi;-1/8*pi;1/12*pi];
% alpha=[3.355280463	3.376066645	3.11027674	3.177991671	2.877022502	2.95101363
% 2.951068016	3.011428266	3.061022422	3.133721381	3.448629073	3.355764823
% 0.877602685	0.846157425	0.920368249	0.86724146	0.858515382	0.881425515
% 0.882204726	0.89596098	1.240101326	1.376545195	1.563014117	0.877597497];

%%  Third Set
% theta_begin=-3.312576582646859;
% theta_end=-3.065299652762040;
% world_IC=[1/24*pi;0;1/20*pi;-1/8*pi;1/12*pi];
% alpha=[3.449256411	3.431052439	3.341424859	3.308497057	3.152231298	3.202479478
% 3.202479954	3.23095169	3.363880499	3.517835299	3.619078486	3.449756355
% 0.874609764	0.86144129	0.900767619	0.868837293	0.885236728	0.923424111
% 0.924422801	0.946028098	1.358151483	1.267402083	1.318094727	0.874609731];

%% Fourth Set

theta_begin=-3.31134774444439;
theta_end=-3.171914312678380;
world_IC=[1/16*pi;0;1/6*pi;-1/8*pi;1/8*pi];
alpha=[3.543776419	3.429974709	3.577527192	3.181766067	3.807936733	3.068046962	3.657494544	3.340984059	3.429544533	3.446868727	3.417296754
3.417296759	3.384764443	3.576721889	3.199988257	4.158174724	2.705946578	4.474561206	3.051819115	3.914272069	3.647077926	3.543776438
0.894224182	0.43122351	1.413256449	-0.630303707	2.447565818	-0.747178065	1.521373486	0.465315942	0.772828935	0.657046162	0.722246781
0.723246734	0.795150917	1.334135242	0.410428556	3.001292898	-1.15764397	4.137290641	-0.418089552	2.411699761	1.315299926	0.89522355];



robot=Marlo_2D_class;

temp=world2gen(world_IC);

IC=[0;0;temp;temp(2:5);0;0;-1/6*pi;0;0;0;0;0;0;0;0];
dt=0.02;
timespan=0:dt:5;
opts = odeset('AbsTol',1e-12,'MaxStep',1e-2,'Events',@impactevent);
X=[];
T=[];
tstart=0;
step_index=[];
% [T,X]=ode45(@swing_model,timespan,IC);
% X=X';
for j=1:5
    [TT,XX]=ode45(@swing_model,timespan,IC,opts);
    XX=XX';
    IC=impact_model(XX(:,end)); % leg switch is considered in the impact model
    X=[X XX];
    T=[T;tstart+TT];
    tstart=tstart+TT(end);
    step_index(j)=length(T);
end

for i=1:length(T)
    [s(i) ds(i)] =get_s_and_ds(X(1:11,i),X(12:22,i),theta_begin,theta_end);
end

for i=1:length(T)
    h0(:,i)=get_h0(X(1:11,i),X(12:22,i),alpha,theta_begin,theta_end);
    hd(:,i)=get_hd(X(1:11,i),X(12:22,i),s(i),ds(i),alpha,theta_begin,theta_end);
end

for i=1:length(T)
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(X(:,i));
    [p1Rm,p2Rm,p3Rm,p4Rm,p1Lm,p2Lm,p3Lm,p4Lm]=robot.get_motor_joint_position(X(:,i));
    swing_foot_height(i)=p4L(2);
    swing_foot_height_motor(i)=p4Lm(2);
end

fig=figure(1);
clf(fig); 
color=['r','g','b','m'];

for j=1:4
    subplot(2,2,j)
    hold on
    plot(T,hd(j,:),'b');
    plot(T,h0(j,:),'r');
    legend(['hd' num2str(j)],['h0' num2str(j)])
    hold off
end
figure(2)
plot(T,s)

figure(3)
angle_name=['1R','2R','1L','2L'];
for j=1:4
    subplot(2,2,j)
    plot(T,X(3+j,:),'-')
    hold on
    plot(T,X(7+j,:),'--')
    for i=1:length(step_index)
        plot([T(step_index(i)) T(step_index(i))],[2 5],'k','LineWidth',0.1)
    end
    hold off
    title(angle_name([2*j-1,2*j]))
    legend('leg','shaft')
    
    
end




figure(4)
plot(T,swing_foot_height)
hold on
plot(T,swing_foot_height_motor,'r-')
for i=1:length(step_index)
    plot([T(step_index(i)) T(step_index(i))],[-0.5 0.5],'k','LineWidth',0.1)
end
hold off
title('swing foot height')

%%%%%make video
    vi = VideoWriter('video', 'MPEG-4');
    set(vi, 'FrameRate', 1/dt);
%%%%%%%%%%%%%%

figure
for i=1:length(T)
    plot_robot(robot,X(:,i))
    hold on
    walk_range=-5:0.1:30;
    plot(walk_range,zeros(1,length(walk_range)),'LineWidth',2)
    hold off
    axis equal
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(X(:,i));
    axis([-3+pHip(1) +3+pHip(1) -1 3])
    drawnow;
    %%%%%%get video%%%%%
    F(i) = getframe(gcf);
    %%%%%%%%%
    pause(dt);
end

%%%%%make video
vi = VideoWriter('video', 'MPEG-4');
set(vi, 'FrameRate', 1/dt);
open(vi);
writeVideo(vi, F);
close(vi);
%%%%%%%%%%%%%%
