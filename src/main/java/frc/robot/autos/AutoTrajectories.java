package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories 
{

    private PathPlannerTrajectory defaultAuto, justLeave;


    private final PathConstraints constraints, slowConstraints;

    public AutoTrajectories() 
    {
        constraints = new PathConstraints(Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel);
        slowConstraints = new PathConstraints(Constants.AutoConstants.slowVel, Constants.AutoConstants.slowAccel);
    }

    public PathPlannerTrajectory defaultAuto() 
    {
        defaultAuto = PathPlanner.loadPath("defaultAuto", constraints);
        return defaultAuto;
    }

    public PathPlannerTrajectory justLeave() 
    {
        justLeave = PathPlanner.loadPath("justLeave", constraints);
        return justLeave;
    }
}