/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class FieldCentricSwerveDrive extends Command {
	
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

	private double originHeading = 0.0;
	private double originCorr = 0;
  	private double leftPow = 1.0;
	private double rightPow = 1.0;
	
	public FieldCentricSwerveDrive() {
        requires(Robot.drive);
    }
	
	@Override
	protected void initialize() {
		originHeading = Robot.zeroHeading;
	}

    @Override
	protected void execute() {
		if (Robot.oi.getLeftJoyButton(7)) {
			originHeading = RobotMap.navX.getFusedHeading();
		}

		double originOffset = 360 - originHeading;
		originCorr = RobotMap.navX.getFusedHeading() + originOffset;

		double strafe = Math.pow(Math.abs(Robot.oi.leftJoy.getX()), leftPow) * Math.signum(Robot.oi.leftJoy.getX());
		double forward = Math.pow(Math.abs(Robot.oi.leftJoy.getY()), leftPow) * -Math.signum(Robot.oi.leftJoy.getY());
        double omega = Math.pow(Math.abs(Robot.oi.rightJoy.getX()), rightPow) * Math.signum(Robot.oi.rightJoy.getX()) * OMEGA_SCALE;
       		
        // Add a small deadzone on the joysticks
        if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow)) strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow)) forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE) omega = 0.0;
		
		
		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			Robot.drive.setDriveLeftFront(0.0);
			Robot.drive.setDriveLeftRear(0.0);
			Robot.drive.setDriveRightFront(0.0);
			Robot.drive.setDriveRightRear(0.0);
			return;
   		}	

		if (!Robot.oi.leftJoy.getTrigger()) {
        	// When the Left Joystick trigger is not pressed, The robot is in Field Centric Mode.
        	// The calculations correct the forward and strafe values for field centric attitude. 
    		
    		// Rotate the velocity vector from the joystick by the difference between our
    		// current orientation and the current origin heading
    		double originCorrection = Math.toRadians(originHeading - RobotMap.navX.getFusedHeading());
    		double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
    		strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
    		forward = temp;
		}
		
		
        Robot.drive.swerveDrive(strafe, forward, omega);
    }

    @Override
	protected boolean isFinished() {
        return false;
    }

    @Override
	protected void end() {
    }

    @Override
	protected void interrupted() {
    	end();
    }

}
