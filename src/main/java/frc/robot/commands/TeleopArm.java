package frc.robot.commands;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.Joystick;

public class TeleopArm extends CommandBase 
{    
    private Arm s_Arm;    
    private Joystick operator;
    private boolean aButton;
    private boolean yButton;

    public TeleopArm(Arm s_Arm, Joystick operator, boolean aSup, boolean ySup) 
    {
        this.s_Arm = s_Arm;
        this.operator = operator;
        addRequirements(s_Arm);
    }

    @Override
    public void execute() 
    {
        s_Arm.moveAngle(operator);          // Move to appropriate angle
        /*
        if(aButton == true || yButton == true)  // If A or Y Button Pressed...
        {
            s_Arm.moveAngle(operator);          // Move to appropriate angle
        }
        else                                    // If not...
        {
            s_Arm.holdAngle();                  // Hold current angle
        }
        */
    }
}