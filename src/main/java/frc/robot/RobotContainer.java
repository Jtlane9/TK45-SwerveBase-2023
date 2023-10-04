package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.reduxrobotics.canand.CANandEventLoop;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;
import com.reduxrobotics.sensors.canandcoder.CANandcoderStatus;

import frc.robot.autos.AutoChooser;
import frc.robot.autos.AutoTrajectories;
import frc.robot.autos.eventMap;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Arm s_Arm = new Arm();

    private final eventMap map = new eventMap(s_Swerve, s_Intake, s_Arm);
    private final AutoTrajectories trajectories = new AutoTrajectories();
    private final AutoChooser chooser = new AutoChooser(trajectories, map.getMap(), s_Swerve, s_Intake, s_Arm);


    //private final Arm s_Arm = new Arm(s_Arm.getEncoder());  // TK 45 - FIX THIS JTL 9-12-23

    private String pPlan = null;
    public double intakeVec = 0;

    public CommandBase autoCode = Commands.sequence(new PrintCommand("no auto selected"));

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    //op controls
    private final int wristAxis = XboxController.Axis.kLeftY.value; // Left Joystick
    private final int ArmAxis = XboxController.Axis.kRightY.value;

    private final JoystickButton y_button_op = new JoystickButton(operator, XboxController.Button.kY.value); // Y Button
    private final JoystickButton x_button_op = new JoystickButton(operator, XboxController.Button.kX.value); // X Button
    private final JoystickButton a_button_op = new JoystickButton(operator, XboxController.Button.kA.value); // A BUtton
    private final JoystickButton b_button_op = new JoystickButton(operator, XboxController.Button.kB.value); // B Button


    private final POVButton w_preset_0 = new POVButton(operator, 0);
    private final POVButton w_preset_1 = new POVButton(operator, 90);
    private final POVButton w_preset_2 = new POVButton(operator, 180);


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    /* Variables */
    boolean driveStatus = false;

    /* PathPlanner Setup */
    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      s_Swerve.setDefaultCommand(
        new TeleopSwerve(
          s_Swerve, 
          () -> -driver.getRawAxis(translationAxis), 
          () -> -driver.getRawAxis(strafeAxis), 
          () -> -driver.getRawAxis(rotationAxis), 
          () -> robotCentric.getAsBoolean()
        )
      );

      s_Intake.setDefaultCommand(
        new TeleopIntake(
          s_Intake,
          operator
        )      
      );
        
      // Configure the button bindings
      configureButtonBindings();

      SmartDashboard.putData("Auto Choices", chooser.getAutoChooser());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() 
    {
        /* Driver Buttons (and op buttons) */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        a_button_op.onTrue(   // Preset to go to front LOW Pickup / Score
          Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_FRONT_SCORE))
          )
        );

        b_button_op.onTrue(  // Preset to go to front MID Pickup / Score
          Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_MID_FRONT_SCORE)) // TK 45 - FIX THIS JTL 9-12-23
          )
        );

        x_button_op.onTrue(  // Preset to go to back LOW Pickup / Score
          Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_LOW_BACK_SCORE))  // TK 45 - FIX THIS JTL 9-12-23
          )
        );

        y_button_op.onTrue(  // Preset to go to back MID Pickup / Score
          Commands.sequence(
            new InstantCommand(() -> s_Arm.setAngle(Constants.ARM_MID_BACK_SCORE))  // TK 45 - FIX THIS JTL 9-12-23
          )
        );
    }


    public void printValues()
    {
        SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
        SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
        SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     
    public Command getAutonomousCommand() 
    {
      Constants.gyroOffset = s_Swerve.gyro.getPitch();
      //s_Swerve.zeroGyro();
      s_Swerve.gyro.setYaw(180);
      return chooser.getCommand();
    }
    
}
