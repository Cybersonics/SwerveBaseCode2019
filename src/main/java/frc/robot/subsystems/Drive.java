/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.FieldCentricSwerveDrive;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

public class Drive extends Subsystem {

	private TalonSRX driveLeftFront;
	private TalonSRX driveLeftRear;
	private TalonSRX driveRightFront;
	private TalonSRX driveRightRear;
	private TalonSRX steerLeftFront;
	private TalonSRX steerLeftRear;
	private TalonSRX steerRightFront;
	private TalonSRX steerRightRear;

	public static final double WHEEL_BASE_LENGTH = 20; //28.0;
  public static final double WHEEL_BASE_WIDTH = 24; //22.0;
  //Set the ENCODER_COUNT_PER_ROTATION to the value for the pivot encoder
  //MA3 encoders = 1024, Versaplanetary (SRXEncoders) = 4096
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;

	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
	public static final double DEADZONE = 0.08;

	//private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
	private static final double STEER_P = 2.0, STEER_I = 0.0, STEER_D = 20.0;
	private static final int STATUS_FRAME_PERIOD = 20;

	public Drive() {
		driveLeftFront = new TalonSRX(RobotMap.DRIVE_LEFT_FRONT_TALON);
		driveLeftFront.configFactoryDefault();
    driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //If you calibrate your drive encoder to zero with the pivot facing backwards the drive will
    //run backwards. If this occurs you will need to invert the drive
    driveLeftFront.setInverted(true);
        
		driveLeftRear = new TalonSRX(RobotMap.DRIVE_LEFT_REAR_TALON);
		driveLeftRear.configFactoryDefault();
    driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //If you calibrate your drive encoder to zero with the pivot facing backwards the drive will
    //run backwards. If this occurs you will need to invert the drive
    driveLeftRear.setInverted(true);
  
		driveRightFront = new TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_TALON);
		driveRightFront.configFactoryDefault();
    driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //If you calibrate your drive encoder to zero with the pivot facing backwards the drive will
    //run backwards. If this occurs you will need to invert the drive
		driveRightFront.setInverted(true);
  
		driveRightRear = new TalonSRX(RobotMap.DRIVE_RIGHT_REAR_TALON);
		driveRightRear.configFactoryDefault();
    driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    //If you calibrate your drive encoder to zero with the pivot facing backwards the drive will
    //run backwards. If this occurs you will need to invert the drive
    driveRightRear.setInverted(true);
  
		steerLeftFront = new TalonSRX(RobotMap.STEER_LEFT_FRONT_TALON);
    steerLeftFront.configFactoryDefault();
    // use this for MA3 encoders
    //steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    steerLeftFront.setInverted(true);
    steerLeftFront.config_kP(0, STEER_P, 0);
    steerLeftFront.config_kI(0, STEER_I, 0);
    steerLeftFront.config_kD(0, STEER_D, 0);
    steerLeftFront.config_IntegralZone(0, 100, 0);
    steerLeftFront.configAllowableClosedloopError(0, 5, 0);
    steerLeftFront.setNeutralMode(NeutralMode.Brake);
    steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerLeftRear = new TalonSRX(RobotMap.STEER_LEFT_REAR_TALON);
    steerLeftRear.configFactoryDefault();
    // use this for MA3 encoders
    //steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
		steerLeftRear.setInverted(true);
    steerLeftRear.config_kP(0, STEER_P, 0);
    steerLeftRear.config_kI(0, STEER_I, 0);
    steerLeftRear.config_kD(0, STEER_D, 0);
    steerLeftRear.config_IntegralZone(0, 100, 0);
    steerLeftRear.configAllowableClosedloopError(0, 5, 0);
    steerLeftRear.setNeutralMode(NeutralMode.Brake);
    steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightFront = new TalonSRX(RobotMap.STEER_RIGHT_FRONT_TALON);
    steerRightFront.configFactoryDefault();
    // use this for MA3 encoders
    //steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    steerRightFront.setInverted(true);
    steerRightFront.config_kP(0, STEER_P, 0);
    steerRightFront.config_kI(0, STEER_I, 0);
    steerRightFront.config_kD(0, STEER_D, 0);
    steerRightFront.config_IntegralZone(0, 100, 0);
    steerRightFront.configAllowableClosedloopError(0, 5, 0);
    steerRightFront.setNeutralMode(NeutralMode.Brake);
    steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

		steerRightRear = new TalonSRX(RobotMap.STEER_RIGHT_REAR_TALON);
    steerRightRear.configFactoryDefault();
    // use this for MA3 encoders
    //steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    //Use this for VersaPlanetary encoders
    steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    //If the pivots turn inward forming an X pattern change the inversion of the motors
    steerRightRear.setInverted(true);
    steerRightRear.config_kP(0, STEER_P, 0);
    steerRightRear.config_kI(0, STEER_I, 0);
    steerRightRear.config_kD(0, STEER_D, 0);
    steerRightRear.config_IntegralZone(0, 100, 0);
    steerRightRear.configAllowableClosedloopError(0, 5, 0);
    steerRightRear.setNeutralMode(NeutralMode.Brake);
    steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
	}

	public void swerveDrive(double strafe, double forward, double omega) {
    double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
    
    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;
    
    // Compute the drive motor speeds
    double speedLF = speed(B, D);
    double speedLR = speed(A, D);
    double speedRF = speed(B, C);
    double speedRR = speed(A, C);
    
		// ... and angles for the steering motors 
		// When drives are calibrated for zero position on encoders they can be at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
    // for initial position/calibration of drives if the drive zero position faces the side of
    // the robot.

		double angleLF = angle(B, D) + 90;
    double angleLR = angle(A, D) - 90;
    double angleRF = angle(B, C) + 90;
    double angleRR = angle(A, C) - 90;
    // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    setSwerveModule(steerLeftFront, driveLeftFront, angleLF, speedLF / maxSpeed);
    setSwerveModule(steerLeftRear, driveLeftRear, angleLR, speedLR / maxSpeed);
    setSwerveModule(steerRightFront, driveRightFront, angleRF, speedRF / maxSpeed);
		setSwerveModule(steerRightRear, driveRightRear, angleRR, speedRR / maxSpeed);
	}
	
	private double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }
  
  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));
  }
	

	private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
    double currentPosition = steer.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    // The angle from the encoder is in the range [0, 360], but the swerve computations
    // return angles in the range [-180, 180], so transform the encoder angle to this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    // TODO: Properly invert the steering motors so this isn't necessary
    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		
		//if (Math.abs(speed) <= MAX_SPEED){
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		drive.set(ControlMode.PercentOutput, speed);

	}

	//get Encoder values
	
	public double getSteerLFEncoder() {
		return steerLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerLREncoder() {
		return steerLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getSteerRFEncoder() {
		return steerRightFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerRREncoder() {
		return steerRightRear.getSelectedSensorPosition(0);
	}

	// //setting motors
	public void setDriveLeftFront(double speed){
		driveLeftFront.set(ControlMode.PercentOutput, speed);
	}

	public void setDriveLeftRear(double speed){
		driveLeftRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setDriveRightFront(double speed){
		driveRightFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setDriveRightRear(double speed){
		driveRightRear.set(ControlMode.PercentOutput, speed);
	}

	
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new FieldCentricSwerveDrive());
  }
	
}