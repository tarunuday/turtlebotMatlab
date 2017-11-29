function [isUpdated,estimatedPose,covariance]=localize(trueorfalse,subscan,subodom)
    if trueorfalse==false
        isUpdated=false;
        estimatedPose=0;covariance=0;
    else
        mcl = robotics.MonteCarloLocalization;
        
        sm = robotics.LikelihoodFieldSensorModel;
        p = zeros(200,200);
        sm.Map = robotics.OccupancyGrid(p,20);
        mcl.SensorModel = sm;
        mcl.UseLidarScan = true;
        
        %sample laser scan
        scan=receive(subscan);
        lscan=lidarScan(scan);
        plot(lscan)
        %sample odometry
        MsgOdom=receive(subodom);
        x=MsgOdom.Pose.Pose.Position.X;
        y=MsgOdom.Pose.Pose.Position.Y;
        th=MsgOdom.Twist.Twist.Angular.Z;
        odomPose=[x,y,th];
        
        [isUpdated,estimatedPose,covariance] = mcl(odomPose,lscan)
        
    end
end