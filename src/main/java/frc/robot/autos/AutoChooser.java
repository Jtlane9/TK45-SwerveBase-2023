
package frc.robot.autos;

import java.time.chrono.ThaiBuddhistEra;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoChooser 
{
    private final AutoTrajectories trajectories;
    private final Swerve s_Swerve;
    
    private final SendableChooser<AutonomousMode> m_chooser = new SendableChooser<>();
    private HashMap<String, Command> eventMap;
    private PIDController thetaController = new PIDController(0.05, 0.005, 0.009);

    public AutoChooser(AutoTrajectories trajectories, HashMap<String, Command> eventMap, Swerve s_Swerve, Intake s_Intake, Arm s_Arm) 
    {
        this.s_Swerve = s_Swerve;
        this.eventMap = eventMap;
        this.trajectories = trajectories;
        
        m_chooser.setDefaultOption("Default Auto", AutonomousMode.kDefaultAuto);
        m_chooser.addOption("doNothing", AutonomousMode.kDoNothing);
        m_chooser.addOption("justLeave", AutonomousMode.kJustLeave);
        m_chooser.addOption("scoreAndLeave", AutonomousMode.kScoreAndLeave);
        m_chooser.addOption("twoPiece", AutonomousMode.kTwoPiece);
        m_chooser.addOption("threePiece", AutonomousMode.kThreePiece);
        m_chooser.addOption("scoreAndLeaveRed", AutonomousMode.kScoreAndLeaveRed);
        m_chooser.addOption("scoreLeaveBumpBlue", AutonomousMode.kScoreLeaveBumpBlue);
    }

    public SendableChooser<AutonomousMode> getAutoChooser()
    {
        return m_chooser;
    }

    public PIDController getPIDController() 
    {
        return thetaController;
    }

    public PPSwerveControllerCommand createControllerCommand(PathPlannerTrajectory trajectory) 
    {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new PPSwerveControllerCommand
        (trajectory, 
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics , 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        thetaController,
        s_Swerve::setModuleStates,
        true,
        s_Swerve
        );
    }

    public Command defaultAuto() // works?
    {
        var swerveCommand = createControllerCommand(trajectories.defaultAuto());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.defaultAuto().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.defaultAuto().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }

    public Command doNothing() // works
    {
        var swerveCommand = createControllerCommand(trajectories.doNothing());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand,
        trajectories.doNothing().getMarkers(),
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands
        (       
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.doNothing().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );

        return command;
    }

    public Command justLeave()  // works
    {
        var swerveCommand = createControllerCommand(trajectories.justLeave());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.justLeave().getMarkers(),
            eventMap);

        
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.justLeave().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }

    public Command scoreAndLeave()  // FOR BLUE - works
    {
        var swerveCommand = createControllerCommand(trajectories.scoreAndLeave());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.scoreAndLeave().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.scoreAndLeave().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand),
            new SequentialCommandGroup(eventMap.get("autoCorrect"))
        );
        return command;
    }

    public Command scoreAndLeaveRed()  // FOR RED - never tested 
    {
        var swerveCommand = createControllerCommand(trajectories.scoreAndLeave());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.scoreAndLeave().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.scoreAndLeave().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand),
            new SequentialCommandGroup(eventMap.get("autoCorrect"))
        );
        return command;
    }

    public Command scoreLeaveBumpBlue()  // FOR BLUE - never tested
    {
        var swerveCommand = createControllerCommand(trajectories.scoreLeaveBumpBlue());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.scoreAndLeave().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.scoreAndLeave().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand),
            new SequentialCommandGroup(eventMap.get("autoCorrect"))
        );
        return command;
    }


    public Command twoPiece() // never tested
    {
        var swerveCommand = createControllerCommand(trajectories.twoPiece());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.twoPiece().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid")),
            new InstantCommand(() -> {
                PathPlannerTrajectory.PathPlannerState initialState = trajectories.twoPiece().getInitialState();
                  initialState =
                      PathPlannerTrajectory.transformStateForAlliance(
                          initialState, DriverStation.getAlliance());
    
                s_Swerve.resetOdometry(
                    new Pose2d(
                        initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
              }
              ),

              new SequentialCommandGroup(followCommand),
              new SequentialCommandGroup(eventMap.get("scoreCubeMid"))
        );
        return command;
    }

    public Command threePiece() // NOT Cable Tray Side - // never tested
    {
        var swerveCommand = createControllerCommand(trajectories.threePiece());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.threePiece().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid")),
            new InstantCommand(() -> {
                PathPlannerTrajectory.PathPlannerState initialState = trajectories.threePiece().getInitialState();
                  initialState =
                      PathPlannerTrajectory.transformStateForAlliance(
                          initialState, DriverStation.getAlliance());
    
                s_Swerve.resetOdometry(
                    new Pose2d(
                        initialState.poseMeters.getTranslation(), initialState.holonomicRotation));
              }),
              new SequentialCommandGroup(followCommand),
              new SequentialCommandGroup(eventMap.get("scoreCubeLow")),
              new SequentialCommandGroup(followCommand),
              new SequentialCommandGroup(eventMap.get("scoreCubeLow"))
        );
        return command;
    }

    
    public Command getCommand() 
    {
        switch (m_chooser.getSelected()) {
            case kDefaultAuto :
            return defaultAuto();

            case kDoNothing :
            return doNothing();

            case kJustLeave :
            return justLeave();

            case kScoreAndLeave :
            return scoreAndLeave();

            case kTwoPiece :
            return twoPiece();

            case kThreePiece :
            return threePiece();

            case kScoreAndLeaveRed :
            return scoreAndLeave();
        }
        return defaultAuto();
    }

    
    private enum AutonomousMode 
    {
        kDefaultAuto, kJustLeave, kScoreAndLeave, kDoNothing, kTwoPiece, kThreePiece, kScoreAndLeaveRed, kScoreLeaveBumpBlue
    }

}