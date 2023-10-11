package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.reduxrobotics.canand.CANandDevice;
import com.reduxrobotics.canand.CANandEventLoop;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.XboxController;

public class Arm extends SubsystemBase
{
    private TalonSRX arm;
    private PIDController pidController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

    double target = 0;
    int angle;
    DigitalInput armSwitchForward = new DigitalInput(1); //limit switch that re-zeros the arm encoder when forward // Probably won't use
    //CANandcoder m_encoder;  // Use if CANanCoder doesn't work
    DutyCycleEncoder m_encoder;
    //PIDController pid;
    //double set_value;

    public Arm() 
    {
        arm = new TalonSRX(Constants.ArmID);
        arm.setInverted(false);
        //m_encoder = new CANandcoder(Constants.ArmEncoderID);  // Use if CANandCoder
        m_encoder = new DutyCycleEncoder(0);  // Use if CANanCoder doesn't work
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Arm Angle", getAngle());
    }
    
    /*
      // JTL 10-10-23 Comment out to work with REV Through Bore
    public CANandcoder getEncoder()
    {
        return m_encoder;
    }
    */

    public double getTarget() 
    {
        return target;
    }

    public double getAngle()
    {
        // Debug:
        //System.out.print("Get Angle: ");
        //System.out.println(m_encoder.getAbsolutePosition()*360-151.8);
        return m_encoder.getAbsolutePosition()*360-151.8;
    }

    //goto a preset

    public void setArmPreset(double target)
    {
        //System.out.print("set preset");
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));    //JTL 10-9-23 CHECK THIS CONTROL MODE
    }

    public void setAngle(double angle)  // For Auto
    {
        // Debug:
        /*
        System.out.print("setting angle: ");
        System.out.println(pidController.calculate(getAngle(), angle));
        System.out.print("Current: ");
        System.out.println(arm.getSupplyCurrent());
        */
        arm.set(ControlMode.PercentOutput, pidController.calculate(getAngle(), angle));
        target = angle;
    }
    

    public void moveAngle(Joystick opJoystick, Joystick drJoystick)    // For Teleop
    {   
        // IF OP Joystick IS Connected:
        if(opJoystick.isConnected())
        {
            // Check Buttons
            if (opJoystick.getRawButton(XboxController.Button.kY.value))  // Y Button Pressed - MID FRONT SCORE - OP
            {
                //System.out.print("MID FRONT SCORE");
                target = Constants.ARM_MID_FRONT_SCORE; 
            }
            else if (opJoystick.getRawButton(XboxController.Button.kA.value))  // A Button Pressed - LOW FRONT SCORE - OP
            {
                //System.out.print("LOW FRONT SCORE");
                target = Constants.ARM_LOW_FRONT_SCORE; 
            }
            else    // No Button Pressed
            {
                //System.out.print("HOLD ANGLE");
                holdAngle();
            }
        }
        else    // OP Joystick NOT Connected
        {
            if (drJoystick.getRawButton(XboxController.Button.kY.value))  // Y Button Pressed - MID FRONT SCORE - DRIVER
            {
                //System.out.print("MID FRONT SCORE");
                target = Constants.ARM_MID_FRONT_SCORE; 
            }
            else if (drJoystick.getRawButton(XboxController.Button.kA.value))  // A Button Pressed - LOW FRONT SCORE - DRIVER
            {
                //System.out.print("LOW FRONT SCORE");
                target = Constants.ARM_LOW_FRONT_SCORE; 
            }
            else    // No Button Pressed
            {
                //System.out.print("HOLD ANGLE");
                holdAngle();
            }
        }

        // Move to target
        arm.set(ControlMode.PercentOutput, pidController.calculate(getAngle(), target));
    }
    

    public void holdAngle() // Maintains the current angle using PID.
    {
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));  //JTL 10-9-23 CHECK THIS CONTROL MODE
    }
}