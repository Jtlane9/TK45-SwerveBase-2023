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
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase 
{
    private TalonSRX intakeLeft;
    Servo gripper = new Servo(0);    // TK 45 - NEED TO UPDATE PWM PORT NUMBER
    // Add a second CAN ID for second intake motor
    public boolean intakeCube = true;

    public Intake() 
    {
        intakeLeft = new TalonSRX(70);
        intakeLeft.setInverted(true);       
    }

    public void setSpeed(double speed) 
    {
        intakeLeft.set(ControlMode.PercentOutput, speed); 
    }


    // I think we can go without this, we just won't get feedback on intake velocity, which is probably fine.
    /* 
    public double getSpeed()    
    { 
        return intake.get();
    }
    */

    public void runIntake(Joystick joystick)    // Controls on Operator Controller - Gamepad (PS4 Controller I think)
    {
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value))    // Cube / Cone Intake - Right Front Bumper
        {
            intakeLeft.set(ControlMode.PercentOutput, -1); // TK45 - Probs need to adjust speed
        } 
        else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Cube / Cone Outtake - Left Front Bumper
        {
            intakeLeft.set(ControlMode.PercentOutput, 0.7); // TK45 - Probs need to adjust speed
        } 
        else 
        {
            // Might need to add a line to set intake to cube by default when no buttons pressed
            intakeLeft.set(ControlMode.PercentOutput, 0); // TK45 - Probs need to adjust speed  
            // TK 45 - 10-3-23 Probably need to set to slow constant IN to hold cube (PID won't cut it); (Also, no encoder);

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
