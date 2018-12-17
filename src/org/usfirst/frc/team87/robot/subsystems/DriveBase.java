package org.usfirst.frc.team87.robot.subsystems;

import org.usfirst.frc.team87.robot.RobotMap;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class DriveBase extends Subsystem {
	
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	Timer timer;
	AHRS ahrs;
	
	// Collision Detection Variables
	double currentWorldLinearAccelerationX, currentJerkX, lastWorldLinearAccelerationX;
	double currentWorldLinealAcceleraionY, currentJerkY, lastWorldLinearAccelerationY;
	boolean collisionX, collisionY = false;
	final static double kCollisionThreshold_DeltaG = 0.5f;
	
	WPI_TalonSRX leftFrontMotor; 
	WPI_TalonSRX leftRearMotor; 
	WPI_TalonSRX rightFrontMotor; 
	WPI_TalonSRX rightRearMotor; 
	
	SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
	SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);
	DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);
	
	public DriveBase() {
		ahrs = new AHRS(SPI.Port.kMXP);
		
		leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFTFRONTMOTOR);
		leftRearMotor = new WPI_TalonSRX(RobotMap.LEFTREARMOTOR);
		rightFrontMotor = new WPI_TalonSRX(RobotMap.RIGHTFRONTMOTOR);
		rightRearMotor = new WPI_TalonSRX(RobotMap.RIGHTREARMOTOR);
		
		leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		leftRearMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		rightRearMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
	}
	
	// Collision Detection of X and Y Axis
	void collisionDetection() {
		currentWorldLinearAccelerationX = ahrs.getWorldLinearAccelX();
		currentJerkX = currentWorldLinearAccelerationX - lastWorldLinearAccelerationX;
		lastWorldLinearAccelerationX = currentWorldLinearAccelerationX;
		
        if(Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) {
               collisionX = true;
        } else if (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) {
        	collisionY = true;
        }	
	}
	
	public void driveTank(double left, double right) {
		robotDrive.tankDrive(left, right);
	}
	
	public void drive(double speed, double rotation) {
		// Might be removed
		collisionDetection();
		
		double leftSpeed = speed + rotation;
		double rightSpeed = speed - rotation;
		
		leftFrontMotor.set(ControlMode.PercentOutput, leftSpeed);
		leftRearMotor.set(ControlMode.PercentOutput, leftSpeed);
		rightFrontMotor.set(ControlMode.PercentOutput, rightSpeed);
		rightRearMotor.set(ControlMode.PercentOutput, rightSpeed);
	}
	
	public void ds() {
		timer.reset();
		timer.start();
		
		if(timer.get() < 5) {			
			robotDrive.tankDrive(0.75, 0.75);
		} else {
			robotDrive.stopMotor();
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

