// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.Motors;
import frc.robot.subsystems.Drivetrain.SwerveModule;

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
    (You were a fool, past Simon. This is going to be a lot of code, and even more math - Simon 10/11/21)

    IF YOU DO WANT TO EDIT THIS COMMAND, BE SURE TO READ THE SWERVE PDFs
    (can be found on chief delphi, search for "4 wheel independent drive independent steering swerve", should be 1st 2 PDFs)

    Steps of what we need to do:
    1. Convert joystick X/Y values to degrees   
    2. Modify that value by NavX position to do field orientation **Skipped this step for now**
    3. Feed final value to a rotateModules() function
    4. Get speed value from joystick
    5. Feed that to a driveMotors() function 
    6. Add in a rotateEntireRobot() function using the other joystick and hope it doesnt break anything **Currently on this step - Simon**
    7. Debug the heck out of this command
    */

    //arctan of slope of the line through the orgin and (JoyX, JoyY) gives us degree value (atan2 does that, plus catches all exeptions)
    joystickDegrees = Math.atan2(-RobotContainer.xbox.getY(Hand.kLeft),RobotContainer.xbox.getX(Hand.kLeft));
    
    //joystickDegrees is actually in radians right now, so we convert it to degrees (change later if necessary)
    joystickDegrees = joystickDegrees*(180/Math.PI);

  
    //Define robot target vector variables (X,Y,Z respectively)  
    double forward = -RobotContainer.xbox.getY(Hand.kLeft);
    double strafe = RobotContainer.xbox.getX(Hand.kLeft);
    double rotation = RobotContainer.xbox.getX(Hand.kRight);

    //Modify target values for field orientation (temp used to save calculations before original forward and strafe values are modified)
    double temp = forward * Math.cos(drivetrain.getNavXOutput()) + strafe * Math.sin(drivetrain.getNavXOutput()); 
    strafe = -forward * Math.sin(drivetrain.getNavXOutput()) + strafe * Math.cos(drivetrain.getNavXOutput()); 
    forward = temp;

    //Do some math to actually define the target vectors
    //I don't have enough space to say what A,B,C and D represent, but the swerve documentation does it well 
    double A = strafe - (rotation * (Constants.wheelbase/2));
    double B = strafe + (rotation * (Constants.wheelbase/2));
    double C = forward - (rotation * (Constants.trackwidth/2));
    double D = forward + (rotation * (Constants.trackwidth/2));

    //Set speeds for modules
    drivetrain.rotateMotor(Motors.FRONT_LEFT_ROT, Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2)));
    drivetrain.rotateMotor(Motors.FRONT_RIGHT_ROT, Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)));
    drivetrain.rotateMotor(Motors.REAR_LEFT_ROT, Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2)));
    drivetrain.rotateMotor(Motors.REAR_RIGHT_ROT, Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2)));

    //Set angles for modules (change speed mod later if needed)
    drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_LEFT, Math.atan2(B, D), 0.2);
    drivetrain.rotateModuleNonLinear(SwerveModule.FRONT_RIGHT, Math.atan2(B, C), 0.2);
    drivetrain.rotateModuleNonLinear(SwerveModule.REAR_LEFT, Math.atan2(A, D), 0.2);
    drivetrain.rotateModuleNonLinear(SwerveModule.REAR_RIGHT, Math.atan2(A, C), 0.2);

    //Show important values on shuffleboard
    driveWithXboxDashboard = "FL Module/" + "Speed: " + String.valueOf(Math.round(Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2)))) + " Angle: " + String.valueOf(Math.round(Math.atan2(B, D))) + ";";
    driveWithXboxDashboard = driveWithXboxDashboard + "FR Module/" + "Speed: " + String.valueOf(Math.round(Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)))) + " Angle: " + String.valueOf(Math.round(Math.atan2(B, C))) + ";";
    driveWithXboxDashboard = driveWithXboxDashboard + "RL Module/" + "Speed: " + String.valueOf(Math.round(Math.sqrt(Math.pow(A, 2) + Math.pow(A, 2)))) + " Angle: " + String.valueOf(Math.round(Math.atan2(A, D))) + ";";
    driveWithXboxDashboard = driveWithXboxDashboard + "RR Module/" + "Speed: " + String.valueOf(Math.round(Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2)))) + " Angle: " + String.valueOf(Math.round(Math.atan2(A, C))) + ";";

  }  

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
