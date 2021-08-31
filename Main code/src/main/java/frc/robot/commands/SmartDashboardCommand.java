// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class SmartDashboardCommand extends CommandBase {

  
  public SmartDashboardCommand() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //LOCAL VARS FROM COMMANDS


    //DriveWithXbox display local vars  
    
    String[] splitStringArrayDWX = DriveWithXbox.driveWithXboxDashboard.split(";");

    for(int i = 0; i <= splitStringArrayDWX.length-1; i++){

      String[] splitSplitStringArrayDWX = splitStringArrayDWX[0].split("/");
      SmartDashboard.putString(splitSplitStringArrayDWX[0], splitSplitStringArrayDWX[1]);

    }
    
    SmartDashboard.putString("driveWithXboxDashboard", DriveWithXbox.driveWithXboxDashboard);

    //VARS FROM ROBOTCONTAINER AND DRIVETRAIN

    SmartDashboard.putNumber("Xbox left X value", RobotContainer.xbox.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xbox left Y value", RobotContainer.xbox.getY(Hand.kLeft));
    /*    
    SmartDashboard.putNumber("frontLeft encoder value", drivetrain.getRotEncoderValue(SwerveModule.FRONT_LEFT));
    SmartDashboard.putNumber("frontLeft PID value", drivetrain.getRotPIDOutput(SwerveModule.FRONT_LEFT));
    
    SmartDashboard.putNumber("frontRight encoder value", drivetrain.getRotEncoderValue(SwerveModule.FRONT_RIGHT));
    SmartDashboard.putNumber("frontRight PID value", drivetrain.getRotPIDOutput(SwerveModule.FRONT_RIGHT));
    
    SmartDashboard.putNumber("rearLeft encoder value", drivetrain.getRotEncoderValue(SwerveModule.REAR_LEFT));
    SmartDashboard.putNumber("rearLeft PID value", drivetrain.getRotPIDOutput(SwerveModule.REAR_LEFT));

    SmartDashboard.putNumber("rearRight encoder value", drivetrain.getRotEncoderValue(SwerveModule.REAR_RIGHT));
    SmartDashboard.putNumber("rearRight PID value", drivetrain.getRotPIDOutput(SwerveModule.REAR_RIGHT));
    */

  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
