// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;


public class Drivetrain extends SubsystemBase {

  TalonFX frontLeftDrvMotor;
  TalonFX frontRightDrvMotor;
  TalonFX rearLeftDrvMotor;
  TalonFX rearRightDrvMotor;

  TalonFX frontLeftRotMotor;
  TalonFX frontRightRotMotor;
  TalonFX rearLeftRotMotor;
  TalonFX rearRightRotMotor;

  CANCoder frontLeftRotEncoder;
  CANCoder frontRightRotEncoder;
  CANCoder rearLeftRotEncoder;
  CANCoder rearRightRotEncoder;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    frontLeftDrvMotor = new TalonFX(Constants.frontLeftDrvMotorPort);
    frontRightDrvMotor = new TalonFX(Constants.frontRightDrvMotorPort);
    rearLeftDrvMotor = new TalonFX(Constants.rearLeftDrvMotorPort);
    rearRightDrvMotor = new TalonFX(Constants.rearRightDrvMotorPort);

    frontLeftRotMotor = new TalonFX(Constants.frontLeftRotMotorPort);
    frontRightRotMotor = new TalonFX(Constants.frontRightRotMotorPort);
    rearLeftRotMotor = new TalonFX(Constants.rearLeftRotMotorPort);
    rearRightRotMotor = new TalonFX(Constants.rearRightRotMotorPort);

    frontLeftRotEncoder = new CANCoder(Constants.frontLeftRotEncoderPort);
    frontRightRotEncoder = new CANCoder(Constants.frontRightRotEncoderPort);
    rearLeftRotEncoder = new CANCoder(Constants.rearLeftRotEncoderPort);
    rearRightRotEncoder = new CANCoder(Constants.rearRightRotEncoderPort);

    //Changes encoders from (0,360) to (-180,180)
    frontLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    frontRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearRightRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    frontLeftRotEncoder.setPosition(0);
    frontRightRotEncoder.setPosition(0);
    rearLeftRotEncoder.setPosition(0);
    rearRightRotEncoder.setPosition(0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void rotateAllModulesNonLinear(double targetDegrees, double speed){

    //FRONT LEFT
    if(frontLeftRotEncoder.getPosition() == targetDegrees){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(frontLeftRotEncoder.getPosition() > targetDegrees){
      //Need to check motor power vs direction of rotation
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(frontLeftRotEncoder.getPosition() < targetDegrees){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

    //FRONT RIGHT
    if(frontRightRotEncoder.getPosition() == targetDegrees){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(frontRightRotEncoder.getPosition() > targetDegrees){
      //Need to check motor power vs direction of rotation
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(frontLeftRotEncoder.getPosition() < targetDegrees){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

    //REAR LEFT
    if(rearLeftRotEncoder.getPosition() == targetDegrees){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(rearLeftRotEncoder.getPosition() > targetDegrees){
      //Need to check motor power vs direction of rotation
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    } 
    else if(rearLeftRotEncoder.getPosition() < targetDegrees){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

    //REAR RIGHT
    if(rearRightRotEncoder.getPosition() == targetDegrees){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else if(rearRightRotEncoder.getPosition() > targetDegrees){
      //Need to check motor power vs direction of rotation
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput,speed);
    }
    else if(rearRightRotEncoder.getPosition() < targetDegrees){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

  } 


  //The only difference between rotateAllModulesNonLinear() and rotateAllModulesLinear() is that the non linear is meant to be
  //called in a bigger loop, and the linear one makes its own loop

  public void rotateAllModulesLinear(double targetDegrees, double speed){

    while(frontLeftRotEncoder.getPosition() != targetDegrees && frontRightRotEncoder.getPosition() != targetDegrees && rearLeftRotEncoder.getPosition() != targetDegrees && rearRightRotEncoder.getPosition() != targetDegrees){

      if(frontLeftRotEncoder.getPosition() == targetDegrees){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput,  0);
      }
      else if(frontLeftRotEncoder.getPosition() > targetDegrees){
        //Need to check motor power vs direction of rotation
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
      }
      else if(frontLeftRotEncoder.getPosition() < targetDegrees){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
      }

      //FRONT RIGHT
      if(frontRightRotEncoder.getPosition() == targetDegrees){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else if(frontRightRotEncoder.getPosition() > targetDegrees){
        //Need to check motor power vs direction of rotation
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
      }
      else if(frontLeftRotEncoder.getPosition() < targetDegrees){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
      }

      //REAR LEFT
      if(rearLeftRotEncoder.getPosition() == targetDegrees){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else if(rearLeftRotEncoder.getPosition() > targetDegrees){
        //Need to check motor power vs direction of rotation
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
      }
      else if(rearLeftRotEncoder.getPosition() < targetDegrees){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
      }

      //REAR RIGHT
      if(rearRightRotEncoder.getPosition() == targetDegrees){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else if(rearRightRotEncoder.getPosition() > targetDegrees){
        //Need to check motor power vs direction of rotation
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
      }
      else if(rearRightRotEncoder.getPosition() < targetDegrees){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, -speed);
      }

    }

  }


  public void driveAllModulesNonLinear(double speed){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
  
  }

  public void debugRunMotor(){
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, 0.5);
  }

  public double getEncoderValue(){
    return frontLeftRotEncoder.getAbsolutePosition();
  }


}

