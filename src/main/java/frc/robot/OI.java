/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
  public Joystick leftJoy;
  public Joystick rightJoy;
  public XboxController controller;

  public  OI(){
    leftJoy = new Joystick(RobotMap.LEFT_JOYSTICK);
    rightJoy = new Joystick(RobotMap.RIGHT_JOYSTICK);
    controller = new XboxController(RobotMap.CONTROLLER);
  }
  public boolean getAButtonPress(){
    return controller.getAButtonPressed();
  }

  public boolean getAButtonRelease(){
      return controller.getAButtonReleased();
  }

  public boolean getBButtonn(){
      return controller.getBButton();
  }

  public boolean getBButtonPress(){
      return controller.getBButtonPressed();
  }

  public boolean getBButtonRelease(){
      return controller.getBButtonReleased();
  }

  public boolean getLeftJoyButton (int buttonNumber){
      return leftJoy.getRawButton(buttonNumber);
  }

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
