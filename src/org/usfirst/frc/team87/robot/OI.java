/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team87.robot;

import org.usfirst.frc.team87.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

	double deadZone = 0.05;

	public final Joystick joystick = new Joystick(RobotMap.JOYSTICK);
	public final Joystick gamepad = new Joystick(RobotMap.GAMEPAD);
	
	Button turner = new JoystickButton(gamepad, 2);
	Button resetNavx = new JoystickButton(gamepad, 3);

	public OI() {
		turner.whenPressed(new TurnToAngle(180.0));
	}

	public double getJoystickX() {
		double joystickXValue = joystick.getX();
		// If value is less than joystick deadzone return 0; else, return value
		return Math.abs(joystickXValue) < deadZone ? 0.0 : joystickXValue;
	}

	public double getJoytickY() {
		double joystickYValue = joystick.getY();
		return Math.abs(joystickYValue) < deadZone ? 0.0 : joystickYValue;
	}

	public double getGamepadDrive() {
		double gamepadDriveValue = gamepad.getRawAxis(RobotMap.GAMEPAD_DRIVE_AXIS);
		return Math.abs(gamepadDriveValue) < deadZone ? 0.0 : gamepadDriveValue;
	}

	public double getGamepadTurn() {
		double gamepadTurnValue = gamepad.getRawAxis(RobotMap.GAMEPAD_TURN_AXIS);
		return Math.abs(gamepadTurnValue) < deadZone ? 0.0 : gamepadTurnValue;
	}

}
