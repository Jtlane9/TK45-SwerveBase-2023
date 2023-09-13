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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class Intake extends SubsystemBase 
{
    private CANSparkMax intake;
    Servo gripper = new Servo(0);    // TK 45 - NEED TO UPDATE PWM PORT NUMBER
    public boolean intakeCube = true;

    public Intake() 
    {
        intake = new CANSparkMax(33, MotorType.kBrushed);
        intake.restoreFactoryDefaults();
        intake.setInverted(true);       
    }

    public void setSpeed(double speed) 
    {
        intake.set(speed); 
    }
    
    public double getSpeed() 
    { 
        return intake.get(); 
    }

    public void runIntake(Joystick joystick)    // Controls on Operator Controller - Gamepad (PS4 Controller I think)
    {
        if (joystick.getRawButton(XboxController.Button.kRightBumper.value))    // Cube Intake - Right Front Bumper
        {
            intake.set(-1);
            intakeCube = true;
            gripper.setAngle(90);  // TK 45 - CHANGE VALUE // Set to Open (Cube) Mode

        } 
        else if (joystick.getRawButton(XboxController.Button.kLeftBumper.value)) // Cube Outtake - Left Front Bumper
        {
            intake.set(0.7);
            intakeCube = true;
            gripper.setAngle(90);  // TK 45 - CHANGE VALUE // Set to Open (Cube) Mode

        } 
        else if (joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.2)    // Cone Intake - Right Back Trigger
        {
            intake.set(0.65);
            intakeCube = false;
            gripper.setAngle(0);  // TK 45 - CHANGE VALUE // Set to Closed (Cone) Mode

        } 
        else if (joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.2)     // Cone Outtake - Left Back Trigger
        {
            intake.set(-0.7);
            intakeCube = false;
            gripper.setAngle(0);  // TK 45 - CHANGE VALUE // Set to Closed (Cone) Mode

        } 
        else 
        {
            intake.set(0);
        }
    }
}
