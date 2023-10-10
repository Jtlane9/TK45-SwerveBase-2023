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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.reduxrobotics.canand.CANandDevice;
import com.reduxrobotics.canand.CANandEventLoop;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Arm extends SubsystemBase
{
    private TalonSRX arm;
    private PIDController pidController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

    double target = 0;
    int angle;
    DigitalInput armSwitchForward = new DigitalInput(1); //limit switch that re-zeros the arm encoder when forward // Probably won't use
    //CANandcoder m_encoder;  // Use if CANanCoder doesn't work
    DutyCycleEncoder m_encoder;   

    public Arm() 
    {
        arm = new TalonSRX(Constants.ArmID);
        arm.setInverted(false);
        //m_encoder = new CANandcoder(Constants.ArmEncoderID);  // Use if CANandCoder
        m_encoder = new DutyCycleEncoder(0);  // Use if CANanCoder doesn't work
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
    
    /*
    public double armPower() 
    {
        return arm.getAppliedOutput();
    }
    */

    //goto a preset

    public void setArmPreset(double target)
    {
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));    //JTL 10-9-23 CHECK THIS CONTROL MODE
    }

    public void setAngle(double angle) 
    {
        System.out.print("setting angle");
        if (angle < m_encoder.getAbsolutePosition() && !forwardArmSwitchTriggered()) // Re-Zeroes Encoder    // May need to remove limit switch reference - if not using
        {
            m_encoder.reset();
            angle = 0;
        } 
        else if (angle > m_encoder.getAbsolutePosition() && m_encoder.getAbsolutePosition() > Constants.ARM_REVERSE_LIMIT)    // Resets upper limit to prevent exceeding maximum
        {
            angle = Constants.ARM_FORWARD_LIMIT;
        }
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), angle));    // Move arm to calculated postion (based on PID)   //JTL 10-9-23 CHECK THIS CONTROL MODE
        target = angle;
    }

    public void moveArm(double movementVector)  // What actually moves the arm when a position is set. (JTL - ????)
    {
        
        if(movementVector < 0 && forwardArmSwitchTriggered())
        {
            target = 0;
            holdAngle();
            return;
        }
        else if(movementVector > 0 && m_encoder.getAbsolutePosition() > Constants.ARM_FORWARD_LIMIT)
        {
            target = Constants.ARM_FORWARD_LIMIT;
            holdAngle();
            return;
        }
        if(forwardArmSwitchTriggered())
        {
            m_encoder.reset();       // TK45 - CHANGE VALUES?
        }
        arm.set(ControlMode.Position, movementVector);   //JTL 10-9-23 CHECK THIS CONTROL MODE
        target = m_encoder.getAbsolutePosition();
    }

    public void holdAngle() // Maintains the current angle using PID.
    {
        arm.set(ControlMode.Position, pidController.calculate(m_encoder.getAbsolutePosition(), target));  //JTL 10-9-23 CHECK THIS CONTROL MODE
    }

    public boolean forwardArmSwitchTriggered()  // Reads limit switches
    {
        return !armSwitchForward.get();
    }

    public void checkLimitSwitches()    // Updates limit switch variables if need be - rezeroes arm position.
    {
        if(forwardArmSwitchTriggered())
        {
            m_encoder.reset();   // TK45 - CHANGE VALUES?
        }
    }
}