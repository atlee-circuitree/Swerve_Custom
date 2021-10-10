// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import java.lang.Math;

public class DriveWithXbox extends CommandBase {

  private final Drivetrain drivetrain;
  private double joystickDegrees = 0;
  private double finalRotateDegrees = 0;
  private double speed;

  public static String driveWithXboxDashboard;
  
  public DriveWithXbox(Drivetrain dt) {
    
    drivetrain = dt;
    
    addRequirements(drivetrain);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /*
    Holy cow this is going to be A LOT of code eventually...
    (Actually, it's pretty compact/efficient currently. I thought this was going to need a lot more code... - Simon 8/3/21)

    Steps of what we need to do:
    1. Convert joystick X/Y values to degrees   
    2. Modify that value by NavX position to do field orientation **Skipped this step for now**
    3. Feed final value to a rotateModules() function
    4. Get speed value from joystick
    5. Feed that to a driveMotors() function 
    6. Add in a rotateEntireRobot() function using the other joystick and hope it doesnt break anything **Currently on this step - Simon**
    7. Debug the heck out of this command
    */

    //tan-1 of slope of the line through the orgin and (JoyX, JoyY) gives us degree value
    try{
      joystickDegrees = Math.atan(RobotContainer.xbox.getY(Hand.kLeft)/RobotContainer.xbox.getX(Hand.kLeft));
    }
    //But if X is 0 (when degrees is pi/2 or -pi/2) we catch the DivideByZeroError and assign joystickDegrees pi/2 or -pi/2 depending on the Y position
    catch(Exception e){
      joystickDegrees = (Math.PI/2) * (RobotContainer.xbox.getY(Hand.kLeft) / Math.abs(RobotContainer.xbox.getY(Hand.kLeft))); 
    }

    //Since tan-1 only returns values on the right side of the unit circle, this flips it if we want a value out of [pi/2,-pi/2]
    if(RobotContainer.xbox.getX(Hand.kLeft) < 0 && RobotContainer.xbox.getY(Hand.kLeft) > 0){
      joystickDegrees = joystickDegrees + Math.PI;
    }
    else if((RobotContainer.xbox.getX(Hand.kLeft) < 0 && RobotContainer.xbox.getY(Hand.kLeft) < 0)){
      joystickDegrees = joystickDegrees - Math.PI;
    }

    //joystickDegrees is actually in radians right now, so we convert it to degrees (change later if necessary)
    joystickDegrees = joystickDegrees*(180/Math.PI);

    //Put step 2 here:

    //Pass joystickDegrees directly to finalRotateDegrees for now
    finalRotateDegrees = joystickDegrees;

    //Speed modified for testing, change when needed
    if(RobotContainer.xbox.getX(Hand.kLeft) == 0 && RobotContainer.xbox.getY(Hand.kLeft) == 0){
      drivetrain.rotateAllModulesNonLinear(finalRotateDegrees, 0);  
    }
    else{
      drivetrain.rotateAllModulesNonLinear(finalRotateDegrees, 0.1);
    }

    if(Math.abs(RobotContainer.xbox.getX(Hand.kLeft)) > Math.abs(RobotContainer.xbox.getY(Hand.kLeft))){
      speed = RobotContainer.xbox.getX(Hand.kLeft);
    }
    else if(Math.abs(RobotContainer.xbox.getX(Hand.kLeft)) < Math.abs(RobotContainer.xbox.getY(Hand.kLeft))){
      speed = RobotContainer.xbox.getY(Hand.kLeft);
    }
    else{
      speed = 0;
    }

    //Speed modifier for future testing, remove or change later 
    speed = speed * 0.2;

    drivetrain.driveAllModulesNonLinear(speed);

    //Show important values on shuffleboard

    driveWithXboxDashboard = "joystickDegrees/" + String.valueOf(joystickDegrees) + ";";
    driveWithXboxDashboard = driveWithXboxDashboard + "speed/" + String.valueOf(speed);
  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
