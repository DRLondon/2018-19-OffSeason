package org.usfirst.frc.team87.robot.commands;

import org.usfirst.frc.team87.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command {

	double Angle;
	boolean isFinished = false;
	boolean isInErrorZone = false;
	int count;

	public TurnToAngle(double angle) {
		angle = Angle;
		requires(Robot.driveBase);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// PID Controller is on and off
		// Does not need to be called in execute, will automatically run until turned
		// off
		Robot.driveBase.rotate(Angle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double error = Robot.driveBase.turnContoller.getError();
		isInErrorZone = Math.abs(error) < 2;

		// If true
		if (isInErrorZone) {
			count++;
			// isFinished = count >= 5;
			if (count >= 5) {
				isFinished = true;
			}
		} else {
			count = 0;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isFinished;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
