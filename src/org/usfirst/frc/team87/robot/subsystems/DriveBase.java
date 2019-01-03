package org.usfirst.frc.team87.robot.subsystems;

import org.usfirst.frc.team87.robot.RobotMap;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class DriveBase extends Subsystem implements PIDOutput {

	private final ADXRS450_Gyro gyro;
	private final AHRS ahrs;

	Timer timer;

	// Collision Detection Variables
	double currentWorldLinearAccelerationX, currentJerkX, lastWorldLinearAccelerationX;
	double currentWorldLinearAccelerationY, currentJerkY, lastWorldLinearAccelerationY;
	boolean collisionX, collisionY = false;
	final static double kCollisionThreshold_DeltaG = 0.5f;

	private final double kP = 0;
	private final double kI = 0;
	private final double kD = 0;

	public final PIDController turnContoller;
	private WPI_TalonSRX leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;

	SpeedControllerGroup leftDrive, rightDrive;

	DifferentialDrive robotDrive;

	public DriveBase() {
		// Big ass port on top-center of Robo-Rio that our Nav-X connects to
		ahrs = new AHRS(SPI.Port.kMXP);
		gyro = new ADXRS450_Gyro();

		// PID Tuning
		turnContoller = new PIDController(kP, kI, kD, ahrs, this);
		turnContoller.setInputRange(-180, 180);
		// Percentage of output
		turnContoller.setOutputRange(0.50, 0.50);
		turnContoller.setAbsoluteTolerance(2.0f);
		turnContoller.setContinuous();

		// Drive
		leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFTFRONTMOTOR);
		leftRearMotor = new WPI_TalonSRX(RobotMap.LEFTREARMOTOR);
		rightFrontMotor = new WPI_TalonSRX(RobotMap.RIGHTFRONTMOTOR);
		rightRearMotor = new WPI_TalonSRX(RobotMap.RIGHTREARMOTOR);
		leftDrive = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
		rightDrive = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);
		robotDrive = new DifferentialDrive(leftDrive, rightDrive);

		// No Encoders on the robot as of now
		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		leftRearMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightRearMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

	}

	// Rotate relative to robots position
	public void rotate(double angle) {
		ahrs.reset();
		turnContoller.reset();

		turnContoller.setPID(kP, kI, kD);
		turnContoller.setSetpoint(angle);
		turnContoller.enable();
	}

	public void resetNavx() {
		ahrs.reset();
	}

	// Collision Detection of X and Y Axis
	public void collisionDetection() {
		// X-Axis Collision Detection
		currentWorldLinearAccelerationX = ahrs.getWorldLinearAccelX();
		currentJerkX = currentWorldLinearAccelerationX - lastWorldLinearAccelerationX;
		lastWorldLinearAccelerationX = currentWorldLinearAccelerationX;

		// Y-Axis Collision Detection
		currentWorldLinearAccelerationY = ahrs.getWorldLinearAccelY();
		currentJerkY = currentWorldLinearAccelerationY - lastWorldLinearAccelerationY;
		lastWorldLinearAccelerationY = currentWorldLinearAccelerationY;

		collisionX = Math.abs(currentJerkX) > kCollisionThreshold_DeltaG;
		collisionY = Math.abs(currentJerkY) > kCollisionThreshold_DeltaG;

		// If Jerk is greater than 0.5 g's of force
		/*
		 * if (Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) { collisionX = true;
		 * } else if (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) { collisionY
		 * = true; }
		 */
	}

	public void driveTank(double leftspeed, double rightspeed) {
		leftFrontMotor.set(leftspeed);
		leftRearMotor.set(leftspeed);

		rightFrontMotor.set(rightspeed);
		rightRearMotor.set(rightspeed);
	}

	public void driveArcade(ControlMode controlMode, double speed, double rotation) {
		// Might be removed
		collisionDetection();

		double leftSpeed = speed + rotation;
		double rightSpeed = speed - rotation;

		leftFrontMotor.set(controlMode, leftSpeed);
		leftRearMotor.set(controlMode, leftSpeed);
		rightFrontMotor.set(controlMode, rightSpeed);
		rightRearMotor.set(controlMode, rightSpeed);
	}

	public void ds() {
		timer.reset();
		timer.start();

		if (timer.get() < 5) {
			robotDrive.tankDrive(0.75, 0.75);
		} else {
			robotDrive.stopMotor();
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		driveTank(-output, output);
	}
}
