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
    
    Button toggleGearRamButton 			= new JoystickButton(operatorStick, 1);
    Button toggleRopeGrabberButton 		= new JoystickButton(operatorStick, 2);
    Button pulseRopeButton				= new JoystickButton(operatorStick, 3);
    Button floorAcquireButton	 		= new JoystickButton(operatorStick, 4);
    Button toggleWristButton 			= new JoystickButton(operatorStick, 5);
    Button toggleClawButton 			= new JoystickButton(operatorStick, 6);
    Button shootButton 					= new JoystickButton(operatorStick, 7);
    Button autoPositionShooterButton 	= new JoystickButton(operatorStick, 9);
    Button autoGearPlaceButton 			= new JoystickButton(operatorStick, 10);
    Button cancelAutoRoutinesButton 	= new JoystickButton(operatorStick, 11);
    
    Button driverAutoGearButton 		= new JoystickButton(driverStick, 4);
    
    public OI(){
    	toggleGearRamButton.whileHeld(new PushGear());
    	pulseRopeButton.whenPressed(new PulseClimb());
    	//floorAcquireButton.whenPressed(new FloorAcquire());
    	toggleWristButton.whenPressed(new SetWristPosition());
    	toggleClawButton.whenPressed(new SetClawPosition());
    	toggleRopeGrabberButton.whenPressed(new GraspRope());
    	shootButton.whileHeld(new Shoot());
    	//autoPositionShooterButton.whenPressed(new VisionCorrect(false, 4));
    	//autoGearPlaceButton.whenPressed(new VisionCorrect2Step(4));
    	
    	driverAutoGearButton.whenPressed(new VisionCorrect(true, 4));
    }
    
    public boolean autoRoutinesCancelled()
    {
    	System.out.println("cancelled auto routines");
    	return cancelAutoRoutinesButton.get();
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

