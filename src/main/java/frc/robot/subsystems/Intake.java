package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase 
{
    private TalonSRX intakeLeft;
    private TalonSRX intakeRight;
    //Servo gripper = new Servo(0);    // TK 45 - NEED TO UPDATE PWM PORT NUMBER
    //public boolean intakeCube = true;

    public Intake() 
    {
        intakeLeft = new TalonSRX(Constants.IntakeLeftID);
        intakeLeft.configPeakCurrentLimit(20, 10);          // 20A      // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeLeft.configPeakCurrentDuration(200, 10);      // 200ms    // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeLeft.configContinuousCurrentLimit(10, 10);    // 10A      // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeLeft.enableCurrentLimit(true);
        intakeLeft.setInverted(true);     

        intakeRight = new TalonSRX(Constants.IntakeRightID);
        intakeRight.configPeakCurrentLimit(20, 10);          // 20A      // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeRight.configPeakCurrentDuration(200, 10);      // 200ms    // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeRight.configContinuousCurrentLimit(10, 10);    // 10A      // JTL 10-10-23 NEED TO CHANGE TO NOT POP CUBES
        intakeRight.enableCurrentLimit(true);
        intakeRight.setInverted(false); // JTL 10-10-23 CAN CHANGE TO TRUE AND REMOVE "-" from values below
    }

    public void setSpeed(double speed) 
    {
        intakeLeft.set(ControlMode.PercentOutput, speed); 
        intakeRight.set(ControlMode.PercentOutput, speed);
    }

    public void runIntake(Joystick joystick)    // Controls on Operator Controller
    {
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value))    // Cube / Cone Intake - Right Front Bumper
        {
            // NEED TO ADD: IF HIGH CURRENT, STOP / HOLD    -   Added current limit above, maybe this will work?
            intakeLeft.set(ControlMode.PercentOutput, 0.25); 
            intakeRight.set(ControlMode.PercentOutput, -0.25); 
        } 
        else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Cube / Cone Outtake - Left Front Bumper
        {
            intakeLeft.set(ControlMode.PercentOutput, -1); 
            intakeRight.set(ControlMode.PercentOutput, 1); 
        } 
        else 
        {
            // Might need to add a line to set intake to cube by default when no buttons pressed
            intakeLeft.set(ControlMode.PercentOutput, 0); // TK45 - Probs need to adjust speed  //  .1
            intakeRight.set(ControlMode.PercentOutput, 0); // TK45 - Probs need to adjust speed // -.1
            // TK 45 - 10-3-23 Probably need to set to slow constant IN to hold cube (PID won't cut it)(Also, no encoder);
        }



        // For adjustable Cone / Cube Intake:
        /*
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value))    // Cube Intake - Right Front Bumper
        {
            intakeLeft.set(ControlMode.PercentOutput, -1); 
            intakeCube = true;
            gripper.setAngle(90);  // TK 45 - CHANGE VALUE // Set to Open (Cube) Mode
        } 
        else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Cube Outtake - Left Front Bumper
        {
            intakeLeft.set(ControlMode.PercentOutput, 0.7); // TK45 - Probs need to adjust speed
            intakeCube = true;
            gripper.setAngle(90);  // TK 45 - CHANGE VALUE // Set to Open (Cube) Mode
        } 
        else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)    // Cone Intake - Right Back Trigger
        {
            intakeLeft.set(ControlMode.PercentOutput, 0.65); // TK45 - Probs need to adjust speed
            intakeCube = false;
            gripper.setAngle(0);  // TK 45 - CHANGE VALUE // Set to Closed (Cone) Mode
        } 
        else if (joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2)     // Cone Outtake - Left Back Trigger
        {
            intakeLeft.set(ControlMode.PercentOutput, -0.7); // TK45 - Probs need to adjust speed
            intakeCube = false;
            gripper.setAngle(0);  // TK 45 - CHANGE VALUE // Set to Closed (Cone) Mode
        } 
        else 
        {
            // Might need to add a line to set intake to cube by default when no buttons pressed
            intakeLeft.set(ControlMode.PercentOutput, 0); // TK45 - Probs need to adjust speed  
            // TK 45 - 10-3-23 Probably need to set to slow constant IN to hold cube (PID won't cut it); (Also, no encoder);

        }
        */
    }
}
