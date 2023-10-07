package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.reduxrobotics.canand.CANandDevice;
import com.reduxrobotics.canand.CANandEventLoop;
import com.reduxrobotics.sensors.canandcoder.CANandcoder;



public class Arm extends SubsystemBase
{
    private CANSparkMax arm;
    private PIDController pidController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

    double target = 0;
    DigitalInput armSwitchForward = new DigitalInput(1); //limit switch that re-zeros the arm encoder when forward // Probably won't use
    CANandcoder m_encoder;

    public Arm() 
    {
        arm = new CANSparkMax(60, MotorType.kBrushed);   //TK 45 - // NEED to Assign CAN ID    // NEED to change to Talon
        m_encoder = new CANandcoder(61); // CANandCoder ID 61

        arm.restoreFactoryDefaults();
        arm.setSmartCurrentLimit(70);
        arm.setInverted(false);      
    }
    
    
    public CANandcoder getEncoder()
    {
        return m_encoder;
    }
    
    public double getTarget() 
    {
        return target;
    }
    
    public double armPower() 
    {
        return arm.getAppliedOutput();
    }

    //goto a preset

    public void setArmPreset(double target)
    {
        arm.set(pidController.calculate(m_encoder.getAbsPosition(), target));
    }

    public void setAngle(double angle) 
    {
        System.out.print("setting angle");
        if (angle < m_encoder.getAbsPosition() && !forwardArmSwitchTriggered()) // Re-Zeroes Encoder    // May need to remove limit switch reference - if not using
        {
            m_encoder.setPosition(0);
            angle = 0;
        } 
        else if (angle > m_encoder.getAbsPosition() && m_encoder.getAbsPosition() > Constants.ARM_REVERSE_LIMIT)    // Resets upper limit to prevent exceeding maximum
        {
            angle = Constants.ARM_FORWARD_LIMIT;
        }
        arm.set(pidController.calculate(m_encoder.getAbsPosition(), angle));    // Move arm to calculated postion (based on PID)
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
        else if(movementVector > 0 && m_encoder.getAbsPosition() > Constants.ARM_FORWARD_LIMIT)
        {
            target = Constants.ARM_FORWARD_LIMIT;
            holdAngle();
            return;
        }
        if(forwardArmSwitchTriggered())
        {
            m_encoder.setPosition(0);   // TK45 - CHANGE VALUES?
        }
        arm.set(movementVector);
        target = m_encoder.getAbsPosition();
    }

    public void holdAngle() // Maintains the current angle using PID.
    {
        arm.set(pidController.calculate(m_encoder.getAbsPosition(), target));
    }

    public boolean forwardArmSwitchTriggered()  // Reads limit switches
    {
        return !armSwitchForward.get();
    }


    public void checkLimitSwitches()    // Updates limit switch variables if need be - rezeroes arm position.
    {
        if(forwardArmSwitchTriggered())
        {
            m_encoder.setPosition(0);   // TK45 - CHANGE VALUES?
        }
    }

}
