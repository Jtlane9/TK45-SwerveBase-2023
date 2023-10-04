
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
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoChooser 
{
    private final AutoTrajectories trajectories;
    private final Swerve s_Swerve;
    private final Intake s_Intake;
    private final Arm s_Arm;
    

    private final SendableChooser<AutonomousMode> m_chooser= new SendableChooser<>();
    private HashMap<String, Command> eventMap;
    private PIDController thetaController = new PIDController(0.05, 0.005, 0.009);


    public AutoChooser(AutoTrajectories trajectories, HashMap<String, Command> eventMap, Swerve s_Swerve, Intake s_Intake, Arm s_Arm) 
    {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        this.s_Intake = s_Intake;
        this.eventMap = eventMap;
        this.trajectories = trajectories;
        

        m_chooser.setDefaultOption("Default Auto", AutonomousMode.kDefaultAuto);
        m_chooser.addOption("justLeave", AutonomousMode.kJustLeave);
    }

    public SendableChooser<AutonomousMode> getAutoChooser() 
    {
        return m_chooser;
    }

    public PIDController getPIDController() 
    {
        return thetaController;
    }

    public Command defaultAuto() 
    {
        var swerveCommand = createControllerCommand(trajectories.defaultAuto());
        
        FollowPathWithEvents followCommand = new FollowPathWithEvents(
        swerveCommand, 
        trajectories.defaultAuto().getMarkers(), 
        eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        //command.addCommands( eventMap.get("scoreCubeHigh"));
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("scoreCubeMid, Leave")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.defaultAuto().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }

    public Command justLeave()
    {
        var swerveCommand = createControllerCommand(trajectories.defaultAuto());

        FollowPathWithEvents followCommand = new FollowPathWithEvents(
            swerveCommand,
            trajectories.justLeave().getMarkers(),
            eventMap);

        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(
            new SequentialCommandGroup(eventMap.get("justLeave")),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectories.justLeave().getInitialHolonomicPose())),
            new SequentialCommandGroup(followCommand)
        );
        return command;
    }



    public PPSwerveControllerCommand createControllerCommand(PathPlannerTrajectory trajectory) {
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

    public Command getCommand() {
        switch (m_chooser.getSelected()) {
            case kDefaultAuto :
            return defaultAuto();

            case kJustLeave :
            return justLeave();
        }
        return defaultAuto();
    }


    private enum AutonomousMode {
        kDefaultAuto, kJustLeave
    }

    

}