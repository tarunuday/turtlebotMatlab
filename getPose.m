function [x,y,theta]=getPose(odomSub)
    MsgOdom=odomSub.LatestMessage;
    x=MsgOdom.Pose.Pose.Position.X;
    y=MsgOdom.Pose.Pose.Position.Y;
    th=MsgOdom.Twist.Twist.Angular.Z;
    q3=MsgOdom.Pose.Pose.Orientation.Z;
    q4=MsgOdom.Pose.Pose.Orientation.W;
    theta=quat2angle([q4 0 0 q3]);
end