// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;


public class Drivetrain extends SubsystemBase {

  TalonFX frontLeftDrvMotor;

  TalonFX frontLeftRotMotor;

  CANCoder frontLeftRotEncoder;

  PIDController frontLeftPID;


  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    frontLeftDrvMotor = new TalonFX(Constants.frontLeftDrvMotorPort);

    frontLeftRotMotor = new TalonFX(Constants.frontLeftRotMotorPort);

    frontLeftRotEncoder = new CANCoder(Constants.rearLeftRotEncoderPort);

    //Changes encoders from (0,360) to (-180,180)
    frontLeftRotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    frontLeftRotEncoder.setPosition(0);

    //not sure if these are the right values, just grabbed them from Circuitseed_2021
    frontLeftPID = new PIDController(1.0, 0.00, 0.00);

    frontLeftPID.enableContinuousInput(-180, 180);

    frontLeftPID.setTolerance(2.0);
    

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

    frontLeftPID.setSetpoint(targetDegrees);

    //FRONT LEFT
    if(frontLeftPID.atSetpoint()){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, mapValues(frontLeftPID.calculate(frontLeftRotEncoder.getAbsolutePosition()), speed, 0));
    }

  } 


  //The only difference between rotateAllModulesNonLinear() and rotateAllModulesLinear() is that the non linear is meant to be
  //called in a bigger loop, and the linear one makes its own loop

  public void rotateAllModulesLinear(double targetDegrees, double speed){

    frontLeftPID.setSetpoint(targetDegrees);

    while(!frontLeftPID.atSetpoint()){

      //FRONT LEFT
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, mapValues(frontLeftPID.calculate(frontLeftRotEncoder.getAbsolutePosition()), speed, 0.1));
      }

    }

  }


  public void driveAllModulesNonLinear(double speed){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
  
  }

  public double mapValues(double value, double highest, double lowest){
    if(value >= 0){
      return MathUtil.clamp(value, lowest, highest);
    }
    else{
      return MathUtil.clamp(value, -highest, -lowest);
  }
}

  public double getRotEncoderValue(SwerveModule module){
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftRotEncoder.getAbsolutePosition();
    }
    else{
      return 0;
    }
  }
  public double getRotPIDOutput(SwerveModule module){
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftPID.calculate(frontLeftRotEncoder.getAbsolutePosition());
    }
    else{
      return 0;
    }
  }

  public enum SwerveModule{
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
  }

  public void sendShuffleboard(){

    SmartDashboard.putNumber("Xbox left X value", RobotContainer.xbox.getX(Hand.kLeft));
    SmartDashboard.putNumber("Xbox left Y value", RobotContainer.xbox.getY(Hand.kLeft));
    
    SmartDashboard.putNumber("frontLeft encoder value", getRotEncoderValue(SwerveModule.FRONT_LEFT));
    SmartDashboard.putNumber("frontLeft PID value", getRotPIDOutput(SwerveModule.FRONT_LEFT));

  }

  public void recieveLocalShuffleboard(String string, int totalVars){

    String[] splitStringArray = string.split(";");

    for(int i = 0; i >= totalVars-1 ; i++){
      
      SmartDashboard.putString(splitStringArray[i], splitStringArray[i+1]);
    }
  }


}

