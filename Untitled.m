robot=Marlo_2D_class;

X=[q,dq]';
X([1 2],:)=0;
for i=1:length(X(1,:))
    plot_robot(robot,X(:,i))
    hold on
    walk_range=-5:0.1:30;
    plot(walk_range,zeros(1,length(walk_range)),'LineWidth',2)
    hold off
    axis equal
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(X(:,i));
    axis([-3+pHip(1) +3+pHip(1) -1 3])
    drawnow;
    pause(0.05);
end
