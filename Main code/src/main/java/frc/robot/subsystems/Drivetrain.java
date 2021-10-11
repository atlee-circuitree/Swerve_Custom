// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;


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

  PIDController frontLeftPID;
  PIDController frontRightPID;
  PIDController rearLeftPID;
  PIDController rearRightPID;

  AHRS navx;

  public static String drivetrainDashboard;

  public Drivetrain() {
    frontLeftDrvMotor = new TalonFX(Constants.frontLeftDrvMotorPort);
    frontRightDrvMotor = new TalonFX(Constants.frontRightDrvMotorPort);
    rearLeftDrvMotor = new TalonFX(Constants.rearLeftDrvMotorPort);
    rearRightDrvMotor = new TalonFX(Constants.rearRightDrvMotorPort);

    frontLeftRotMotor = new TalonFX(Constants.frontLeftRotMotorPort);
    frontRightRotMotor = new TalonFX(Constants.frontRightRotMotorPort);
    rearLeftRotMotor = new TalonFX(Constants.rearLeftRotMotorPort);
    rearRightRotMotor = new TalonFX(Constants.rearRightRotMotorPort);

    frontLeftRotMotor.setNeutralMode(NeutralMode.Brake);
    frontRightRotMotor.setNeutralMode(NeutralMode.Brake);
    rearLeftRotMotor.setNeutralMode(NeutralMode.Brake);
    rearRightRotMotor.setNeutralMode(NeutralMode.Brake);

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

    //not sure if these are the right values, just grabbed them from Circuitseed_2021
    frontLeftPID = new PIDController(1.0, 0.00, 0.00);
    frontRightPID = new PIDController(1.0, 0.00, 0.00);
    rearLeftPID = new PIDController(1.0, 0.00, 0.00);
    rearRightPID = new PIDController(1.0, 0.00, 0.00);

    frontLeftPID.enableContinuousInput(-180, 180);
    frontRightPID.enableContinuousInput(-180, 180);
    rearLeftPID.enableContinuousInput(-180, 180);
    rearRightPID.enableContinuousInput(-180, 180);

    frontLeftPID.setTolerance(2.0);
    frontRightPID.setTolerance(2.0);
    rearLeftPID.setTolerance(2.0);
    rearRightPID.setTolerance(2.0);
    
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    drivetrainDashboard = "frontLeft rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID/" + getRotPIDOutput(SwerveModule.FRONT_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "frontRight rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID/" + getRotPIDOutput(SwerveModule.FRONT_RIGHT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearLeft rot encoder/" + getRotEncoderValue(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearLeft PID/" + getRotPIDOutput(SwerveModule.REAR_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearRight rot encoder/" + getRotEncoderValue(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID/" + getRotPIDOutput(SwerveModule.REAR_RIGHT); 

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //------------------------------------------------------------------------------------------------------------------------------------
  //ENUMS
  //------------------------------------------------------------------------------------------------------------------------------------
  public enum SwerveModule{
    FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
  }

  public enum Motors{
    FRONT_LEFT_ROT, FRONT_RIGHT_ROT, REAR_LEFT_ROT, REAR_RIGHT_ROT, FRONT_LEFT_DRV, FRONT_RIGHT_DRV, REAR_LEFT_DRV, REAR_RIGHT_DRV
  }

  //------------------------------------------------------------------------------------------------------------------------------------
  //DRIVE/ROTATION
  //------------------------------------------------------------------------------------------------------------------------------------

  public void rotateAllModulesNonLinear(double targetDegrees, double speed){

    frontLeftPID.setSetpoint(targetDegrees);
    frontRightPID.setSetpoint(targetDegrees);
    rearLeftPID.setSetpoint(targetDegrees);
    rearRightPID.setSetpoint(targetDegrees);

    //FRONT LEFT
    if(frontLeftPID.atSetpoint()){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
    }

    //FRONT RIGHT
    if(frontRightPID.atSetpoint()){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
    }

    //REAR LEFT
    if(rearLeftPID.atSetpoint()){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
    }

    //REAR RIGHT
    if(rearRightPID.atSetpoint()){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    else{
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
    }

  } 


  //The only difference between rotateAllModulesNonLinear() and rotateAllModulesLinear() is that the non linear is meant to be
  //called in a bigger loop, and the linear one makes its own loop

  public void rotateAllModulesLinear(double targetDegrees, double speed){

    frontLeftPID.setSetpoint(targetDegrees);
    frontRightPID.setSetpoint(targetDegrees);
    rearLeftPID.setSetpoint(targetDegrees);
    rearRightPID.setSetpoint(targetDegrees);

    while(!frontLeftPID.atSetpoint() || !frontRightPID.atSetpoint() || !rearLeftPID.atSetpoint() || !rearRightPID.atSetpoint()){

      //FRONT LEFT
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
      }

      //FRONT RIGHT
      if(frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
      }

      //REAR LEFT
      if(rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
      }

      //REAR RIGHT
      if(rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
      }
    }

  }


  public void driveAllModulesNonLinear(double speed){
    
    frontLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    frontRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearLeftDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
    rearRightDrvMotor.set(TalonFXControlMode.PercentOutput, speed);
  
  }

  //Add other motors as needed, just make sure to put them in the enum too
  public void rotateMotor(Motors motor, double speed){
    if(motor == Motors.FRONT_LEFT_ROT){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_RIGHT_ROT){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_LEFT_ROT){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_RIGHT_ROT){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_LEFT_DRV){
      frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.FRONT_RIGHT_DRV){
      frontRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_LEFT_DRV){
      rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
    else if(motor == Motors.REAR_RIGHT_DRV){
      rearRightRotMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
  }

  public void rotateModuleNonLinear(SwerveModule module, double targetDegrees, double speed){

    if(module == SwerveModule.FRONT_LEFT){
      frontLeftPID.setSetpoint(targetDegrees);
      if(frontLeftPID.atSetpoint()){
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_LEFT), -speed, speed));
      }
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      frontRightPID.setSetpoint(targetDegrees);
      if(frontRightPID.atSetpoint()){
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        frontRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.FRONT_RIGHT), -speed, speed));
      }
    }
    else if(module == SwerveModule.REAR_LEFT){
      rearLeftPID.setSetpoint(targetDegrees);
      if(rearLeftPID.atSetpoint()){
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearLeftRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_LEFT), -speed, speed));
      }
    }
    else if(module == SwerveModule.REAR_RIGHT){
      rearRightPID.setSetpoint(targetDegrees);
      if(rearRightPID.atSetpoint()){
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, 0);
      }
      else{
        rearRightRotMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(getRotPIDOutput(SwerveModule.REAR_RIGHT), -speed, speed));
      }
    }

  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //SENSORS
  //------------------------------------------------------------------------------------------------------------------------------------

  public double getRotEncoderValue(SwerveModule module){

    double encoderValue = 0;

    //Assigns offset encoder absolute position based on input SwerveModule parameter
    if(module == SwerveModule.FRONT_LEFT){
      encoderValue = frontLeftRotEncoder.getAbsolutePosition() - Constants.frontLeftEncoderOffset;
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      encoderValue = frontRightRotEncoder.getAbsolutePosition() - Constants.frontRightEncoderOffset;
    }
    else if(module == SwerveModule.REAR_LEFT){
      encoderValue = rearLeftRotEncoder.getAbsolutePosition() - Constants.rearLeftEncoderOffset;
    }
    else if(module == SwerveModule.REAR_RIGHT){
      encoderValue = rearRightRotEncoder.getAbsolutePosition() - Constants.rearRightEncoderOffset;
    }
    else{
      return 0;
    }

    //Deals with offset loop bug
    if(encoderValue < 0){
      encoderValue = encoderValue + 360;
    }
    //Signs values ([-180,180] not [0,360])
    if(encoderValue > 180){
      encoderValue = encoderValue - 360;
    }

    return encoderValue;
  }
  public double getRotPIDOutput(SwerveModule module){
    if(module == SwerveModule.FRONT_LEFT){
      return frontLeftPID.calculate(getRotEncoderValue(SwerveModule.FRONT_LEFT));
    }
    else if(module == SwerveModule.FRONT_RIGHT){
      return frontRightPID.calculate(getRotEncoderValue(SwerveModule.FRONT_RIGHT));
    }
    else if(module == SwerveModule.REAR_LEFT){
      return rearLeftPID.calculate(getRotEncoderValue(SwerveModule.REAR_LEFT));
    }
    else if(module == SwerveModule.REAR_RIGHT){
      return rearRightPID.calculate(getRotEncoderValue(SwerveModule.REAR_RIGHT));
    }
    else{
      return 0;
    }
  }
  
  public double getNavXOutput(){
    return navx.getYaw();
  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //OTHER FUNCTIONS
  //------------------------------------------------------------------------------------------------------------------------------------

  public double mapValues(double value, double highest, double lowest){
    if(value >= 0){
      return MathUtil.clamp(value, lowest, highest);
    }
    else{
      return MathUtil.clamp(value, -highest, -lowest);
  }
}

  public String sendDashboard(){
    
    String drivetrainDashboard;

    drivetrainDashboard = "frontLeft rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontLeft PID/" + getRotPIDOutput(SwerveModule.FRONT_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "frontRight rot encoder/" + getRotEncoderValue(SwerveModule.FRONT_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "frontRight PID/" + getRotPIDOutput(SwerveModule.FRONT_RIGHT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearLeft rot encoder/" + getRotEncoderValue(SwerveModule.REAR_LEFT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearLeft PID/" + getRotPIDOutput(SwerveModule.REAR_LEFT) + ";";

    drivetrainDashboard = drivetrainDashboard + "rearRight rot encoder/" + getRotEncoderValue(SwerveModule.REAR_RIGHT) + ";";
    drivetrainDashboard = drivetrainDashboard + "rearRight PID/" + getRotPIDOutput(SwerveModule.REAR_RIGHT);

    return drivetrainDashboard;
  }


}

