// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import java.lang.Math;

public class DriveWithXbox extends CommandBase {

  private final Drivetrain drivetrain;
  private double joystickDegrees = 0;
  
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

    Steps of what we need to do:
    1. Convert joystick X/Y values to degrees   **Currently on this step - Simon**
    2. Modify that value by NavX position to do field orientation
    3. Feed final value to a rotateModules() function
    4. Get speed value from joystick
    5. Feed that to a driveMotors() function
    6. Add in a rotateEntireRobot() function using the other joystick and hope it doesnt break anything
    7. Debug the heck out of this command
    */

    //ASSUMING Math.acos() is like the cos-1 function on calculators, will have to run tests
    joystickDegrees = Math.acos(RobotContainer.xbox.getX());

    //Since cos-1 only returns positive degrees, this flips it if we actually want a negative value
    if(RobotContainer.xbox.getY() < 0){
      joystickDegrees = joystickDegrees*-1;
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
