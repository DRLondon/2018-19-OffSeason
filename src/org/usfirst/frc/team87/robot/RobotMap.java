/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team87.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// VALUES ARE TEMPORARY
	
	public static int LEFTFRONTMOTOR = 0;
	public static int LEFTREARMOTOR = 1;
	public static int RIGHTFRONTMOTOR = 2;
	public static int RIGHTREARMOTOR = 3;
	
	public static int JOYSTICK = 0;
	public static int GAMEPAD = 1;
	
	public static int GAMEPAD_DRIVE_AXIS = 1;
	public static int GAMEPAD_TURN_AXIS = 3;
	
	public static int FRONTCAMERA = 0;
	public static int REARCAMERA = 1;
}
