// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Talon;
import com.revrobotics.CANEncoder;




public class ExampleSubsystem extends SubsystemBase {

  Talon frontLeftDrvMotor;
  Talon frontRightDrvMotor;
  Talon rearLeftDrvMotor;
  Talon rearRightDrvMotor;

  CANEncoder frontLeftDrvEncoder;


  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    frontLeftDrvMotor = new Talon(Constants.frontLeftDrvMotorPort);
    frontRightDrvMotor = new Talon(Constants.frontRightDrvMotorPort);
    rearLeftDrvMotor = new Talon(Constants.rearLeftDrvMotorPort);
    rearRightDrvMotor = new Talon(Constants.rearRightDrvMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
