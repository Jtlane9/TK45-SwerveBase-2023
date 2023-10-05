package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories 
{
    private PathPlannerTrajectory defaultAuto, justLeave, scoreAndLeave, doNothing;

    private final PathConstraints constraints, slowConstraints;
    
    public AutoTrajectories() 
    {
        constraints = new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);
        slowConstraints = new PathConstraints(Constants.AutoConstants.slowVel, Constants.AutoConstants.slowAccel);
    }

    public PathPlannerTrajectory defaultAuto() 
    {
        defaultAuto = PathPlanner.loadPath("justLeave", constraints, false);
        return defaultAuto;
    }

    public PathPlannerTrajectory doNothing()
    {
        doNothing = PathPlanner.loadPath("doNothing", constraints, false);
        return doNothing;
    }

    public PathPlannerTrajectory justLeave() 
    {
        justLeave = PathPlanner.loadPath("justLeave", constraints, false);
        return justLeave;
    }

    public PathPlannerTrajectory scoreAndLeave()
    {
        scoreAndLeave = PathPlanner.loadPath("scoreAndLeave", constraints, false);
        return scoreAndLeave;
    }
}