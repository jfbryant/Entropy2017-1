package org.usfirst.frc.team138.robot.commands;

import org.usfirst.frc.team138.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import java.util.ArrayList;
import org.usfirst.frc.team138.robot.Sensors;
import org.usfirst.frc.team138.robot.subsystems.vision2017.Entropy2017Targeting;

import edu.wpi.first.wpilibj.command.Command;

public class GearCorrect extends Command {
	
	Command driveCommand;
	boolean isDone = false;
	int framesToAverage;
	ArrayList<Entropy2017Targeting.TargetInformation> infoList = new ArrayList<Entropy2017Targeting.TargetInformation>();
	
	public GearCorrect(int numFrames){
		framesToAverage = numFrames;
	}

	protected void initialize() {
		Sensors.targetingCameraMode();
		Sensors.cameraProcessor.processFrames(framesToAverage, "peg");
	}

	protected void execute() {
		if (driveCommand == null)
		{
			infoList.addAll(Sensors.cameraProcessor.getTargetInformation());
			if (infoList.size() == framesToAverage)
			{
				double cumulation = 0;
				int targetsFound = framesToAverage;
				for (Entropy2017Targeting.TargetInformation info : infoList)
				{
					if (info.targetFound)
					{
						cumulation += info.correctionAngle;
					}
					else
					{
						targetsFound--;
					}
				}
				if (targetsFound > 0)
				{
					driveCommand = new AutoDrive(cumulation / targetsFound);
				}
				else
				{
					if (getGroup() != null)
					{
						getGroup().cancel();
					}
					else
					{
						isDone = true;	
					}
				}
				
				driveCommand.start();
			}
		}
		else
		{
			isDone = !driveCommand.isRunning();
		}
	}

	protected boolean isFinished() {
		return isDone || (timeSinceInitialized() > 2 && Robot.mode == "teleop");
	}

	protected void end() {
		Sensors.cameraProcessor.cancelProcessing();
		Sensors.standardCameraMode();
	}

	protected void interrupted() {
		end();
	}

}