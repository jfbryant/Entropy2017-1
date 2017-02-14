package org.usfirst.frc.team138.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team138.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    Joystick driverStick = new Joystick(0);
    Joystick operatorStick = new Joystick(1);
    
    Button toggleGearRamButton = new JoystickButton(operatorStick, 1);
    Button toggleChuteGuardButton = new JoystickButton(operatorStick, 4);
    Button toggleWristButton = new JoystickButton(operatorStick, 5);
    Button toggleClawButton = new JoystickButton(operatorStick, 6);
    
    public OI(){
    	toggleGearRamButton.whenPressed(new PushGear());
    	toggleChuteGuardButton.whenPressed(new SetGuardPosition());
    	toggleWristButton.whenPressed(new SetWristPosition());
    	toggleClawButton.whenPressed(new SetClawPosition());
    }
    
	public double getMoveSpeed()
	{
		return driverStick.getRawAxis(1);
	}
	
	public double getRotateSpeed()
	{
		return driverStick.getRawAxis(4);
	}
	
	public double getClimbSpeed()
	{
		return operatorStick.getRawAxis(1);
	}
}

