// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;


public class Drivetrain extends SubsystemBase {

  Talon frontLeftDrvMotor;
  Talon frontRightDrvMotor;
  Talon rearLeftDrvMotor;
  Talon rearRightDrvMotor;

  Talon frontLeftRotMotor;
  Talon frontRightRotMotor;
  Talon rearLeftRotMotor;
  Talon rearRightRotMotor;

  CANCoder frontLeftRotEncoder;
  CANCoder frontRightRotEncoder;
  CANCoder rearLeftRotEncoder;
  CANCoder rearRightRotEncoder;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    frontLeftDrvMotor = new Talon(Constants.frontLeftDrvMotorPort);
    frontRightDrvMotor = new Talon(Constants.frontRightDrvMotorPort);
    rearLeftDrvMotor = new Talon(Constants.rearLeftDrvMotorPort);
    rearRightDrvMotor = new Talon(Constants.rearRightDrvMotorPort);

    frontLeftRotMotor = new Talon(Constants.frontLeftRotMotorPort);
    frontRightRotMotor = new Talon(Constants.frontRightRotMotorPort);
    rearLeftRotMotor = new Talon(Constants.rearLeftRotMotorPort);
    rearRightRotMotor = new Talon(Constants.rearRightRotMotorPort);

    frontLeftRotEncoder = new CANCoder(Constants.frontLeftRotEncoderPort);
    frontRightRotEncoder = new CANCoder(Constants.frontRightRotEncoderPort);
    rearLeftRotEncoder = new CANCoder(Constants.rearLeftRotEncoderPort);
    rearRightRotEncoder = new CANCoder(Constants.rearRightRotEncoderPort);

    //Changes encoders from (0,360) to (-180,180)
    frontLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    frontRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void driveWithXbox(double FLDpwr, double FRDpwr, double RLDpwr, double RRDpwr, double FLrot, double FRrot, double RLrot, double RRrot){
    
  }

  public void rotateAllModulesNonLinear(double targetDegrees){

        //FRONT LEFT
        if(frontLeftRotEncoder.getPosition() == targetDegrees){
          frontLeftRotMotor.set(0);
        }
        else if(frontLeftRotEncoder.getPosition() > targetDegrees){
          //Need to check motor power vs direction of rotation
          frontLeftRotMotor.set(0.75);
        }
        else if(frontLeftRotEncoder.getPosition() < targetDegrees){
          frontLeftRotMotor.set(-0.75);
        }

        //FRONT RIGHT
        if(frontRightRotEncoder.getPosition() == targetDegrees){
          frontRightRotMotor.set(0);
        }
        else if(frontRightRotEncoder.getPosition() > targetDegrees){
          //Need to check motor power vs direction of rotation
          frontRightRotMotor.set(0.75);
        }
        else if(frontLeftRotEncoder.getPosition() < targetDegrees){
          frontRightRotMotor.set(-0.75);
        }

        //REAR LEFT
        if(rearLeftRotEncoder.getPosition() == targetDegrees){
          rearLeftRotMotor.set(0);
        }
        else if(rearLeftRotEncoder.getPosition() > targetDegrees){
          //Need to check motor power vs direction of rotation
          rearLeftRotMotor.set(0.75);
        }
        else if(rearLeftRotEncoder.getPosition() < targetDegrees){
          rearLeftRotMotor.set(-0.75);
        }

        //REAR RIGHT
        if(rearRightRotEncoder.getPosition() == targetDegrees){
          frontLeftRotMotor.set(0);
        }
        else if(rearRightRotEncoder.getPosition() > targetDegrees){
          //Need to check motor power vs direction of rotation
          rearRightRotMotor.set(0.75);
        }
        else if(rearRightRotEncoder.getPosition() < targetDegrees){
          rearRightRotMotor.set(-0.75);
        }

  }

}

