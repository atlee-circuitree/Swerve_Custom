// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveModule;

import java.lang.Math;

public class DriveWithXbox extends CommandBase {

  private final Drivetrain drivetrain;
  private double joystickDegrees = 0;
  private double finalRotateDegrees = 0;
  private double speed;

  public static String driveWithXboxShuffleboard;
  
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

    //ASSUMING Math.acos() is like the cos-1 function on calculators, will have to run tests
    joystickDegrees = Math.acos(RobotContainer.xbox.getX(Hand.kLeft));

    //Since cos-1 only returns positive values, this flips it if we actually want a negative value (joystick pointing down)
    if(RobotContainer.xbox.getY(Hand.kLeft) > 0){
      joystickDegrees = joystickDegrees*-1;
    }

    //joystickDegrees is actually in radians right now, so we convert it to degrees
    joystickDegrees = joystickDegrees*(180/Math.PI);

    //Put step 2 here:

    //Pass joystickDegrees directly to finalRotateDegrees for now
    finalRotateDegrees = joystickDegrees;

    //Speed modified for testing, change when needed
    drivetrain.rotateAllModulesNonLinear(finalRotateDegrees, 0.1);

    if(RobotContainer.xbox.getX(Hand.kLeft) > RobotContainer.xbox.getY(Hand.kLeft)){
      speed = RobotContainer.xbox.getX(Hand.kLeft);
    }
    else if(RobotContainer.xbox.getX(Hand.kLeft) < RobotContainer.xbox.getY(Hand.kLeft)){
      speed = RobotContainer.xbox.getY(Hand.kLeft);
    }
    else{
      speed = 0;
    }

    //Speed modifier for future testing, remove or change later 
    speed = speed * 0;

    drivetrain.driveAllModulesNonLinear(speed);

    //Show important values on shuffleboard

    driveWithXboxShuffleboard = "joystickDegrees;" + String.valueOf(joystickDegrees) + ";speed;" + String.valueOf(speed);

  
  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
