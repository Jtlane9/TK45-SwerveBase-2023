package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class AutoTrajectories 
{
    private PathPlannerTrajectory defaultAuto, justLeave, scoreAndLeave, doNothing, twoPiece, threePiece, scoreAndLeaveRed, scoreLeaveBumpBlue;

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

    public PathPlannerTrajectory doNothing()
    {
        doNothing = PathPlanner.loadPath("doNothing", constraints);
        return doNothing;
    }

    public PathPlannerTrajectory justLeave() 
    {
        justLeave = PathPlanner.loadPath("justLeave", constraints);
        return justLeave;
    }

    public PathPlannerTrajectory scoreAndLeave()
    {
        scoreAndLeave = PathPlanner.loadPath("scoreAndLeave", constraints);
        return scoreAndLeave;
    }

    public PathPlannerTrajectory twoPiece()
    {
        twoPiece = PathPlanner.loadPath("twoPiece", constraints);
        return twoPiece;
    }

    public PathPlannerTrajectory threePiece()
    {
        threePiece = PathPlanner.loadPath("threePiece", constraints);
        return threePiece;
    }

    public PathPlannerTrajectory scoreAndLeaveRed()
    {
        scoreAndLeaveRed = PathPlanner.loadPath("scoreAndLeaveRed", constraints);
        return scoreAndLeaveRed;
    }

    public PathPlannerTrajectory scoreLeaveBumpBlue()
    {
        scoreLeaveBumpBlue = PathPlanner.loadPath("scoreLeaveBumpBlue", constraints);
        return scoreLeaveBumpBlue;
    }
}