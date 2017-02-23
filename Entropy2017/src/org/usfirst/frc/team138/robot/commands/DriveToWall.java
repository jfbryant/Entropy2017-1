package org.usfirst.frc.team138.robot.commands;

import org.usfirst.frc.team138.robot.Robot;
import org.usfirst.frc.team138.robot.Sensors;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToWall extends Command {
	
	Command driveCommand;
	
	public DriveToWall(){
	}

	protected void initialize() {
		double distance = Sensors.distanceFromWall();
		if (distance > 0)
		{
			driveCommand = new AutoDrive(-0.7, Sensors.distanceFromWall());
			driveCommand.start();
		}
		else
		{
			getGroup().cancel();
		}
	}

	protected void execute() {
		
	}

	protected boolean isFinished() {
		return !driveCommand.isRunning();
	}

	protected void end() {
	}

	protected void interrupted() {
	}

}