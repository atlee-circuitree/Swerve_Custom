// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithXbox;
import frc.robot.commands.TestDriveCommand;
import frc.robot.commands.TestRotateModules;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  

  public static XboxController xbox;
  private final Drivetrain drivetrain;
  private final DriveWithXbox driveWithXbox;
  private final TestRotateModules testRotateModules;
  private final TestDriveCommand testDriveCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //Drive Setup
    drivetrain = new Drivetrain();

    driveWithXbox = new DriveWithXbox(drivetrain);
    //driveWithXbox.addRequirements(drivetrain);
    //drivetrain.setDefaultCommand(driveWithXbox);

    testDriveCommand = new TestDriveCommand(drivetrain);
    testDriveCommand.addRequirements(drivetrain);
    drivetrain.setDefaultCommand(testDriveCommand);

    //Auto Setup
    testRotateModules = new TestRotateModules(drivetrain);

    //Controller Setup
    xbox = new XboxController(0);

    configureButtonBindings();

    //Other Setup
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return testRotateModules;
  }
}
