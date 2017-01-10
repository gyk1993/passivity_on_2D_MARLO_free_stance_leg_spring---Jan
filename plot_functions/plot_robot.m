function plot_robot( robot,x )


[pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(x);
[p1Rm,p2Rm,p3Rm,p4Rm,p1Lm,p2Lm,p3Lm,p4Lm]=robot.get_motor_joint_position(x);
lw=0.1

plot([pT(1),pHip(1)],[pT(2),pHip(2)],'k','LineWidth',lw);
hold on
plot([p3R(1),p1R(1),pHip(1),p2R(1),p4R(1)],[p3R(2),p1R(2),pHip(2),p2R(2),p4R(2)],'r','LineWidth',lw)
plot([p3L(1),p1L(1),pHip(1),p2L(1),p4L(1)],[p3L(2),p1L(2),pHip(2),p2L(2),p4L(2)],'b','LineWidth',lw)
plot([p3Rm(1),p1Rm(1),pHip(1),p2Rm(1),p4Rm(1)],[p3Rm(2),p1Rm(2),pHip(2),p2Rm(2),p4Rm(2)],'r--','LineWidth',lw)
plot([p3Lm(1),p1Lm(1),pHip(1),p2Lm(1),p4Lm(1)],[p3Lm(2),p1Lm(2),pHip(2),p2Lm(2),p4Lm(2)],'b--','LineWidth',lw)
hold off
end

