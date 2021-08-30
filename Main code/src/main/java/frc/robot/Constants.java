// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int frontLeftDrvMotorPort = 4;
    public static final int frontRightDrvMotorPort = 2;
    public static final int rearLeftDrvMotorPort = 6;
    public static final int rearRightDrvMotorPort = 8;

    public static final int frontLeftRotMotorPort = 3;
    public static final int frontRightRotMotorPort = 1;
    public static final int rearLeftRotMotorPort = 5;
    public static final int rearRightRotMotorPort = 7;

    public static final int frontLeftRotEncoderPort = 11;
    public static final int frontRightRotEncoderPort = 9;
    public static final int rearLeftRotEncoderPort = 10;
    public static final int rearRightRotEncoderPort = 12;

    // In degrees
    public static final double frontLeftEncoderOffset = Math.toRadians(214.225);
    public static final double frontRightEncoderOffset = Math.toRadians(118.0);
    public static final double rearLeftEncoderOffset = Math.toRadians(251.15);
    public static final double rearRightEncoderOffset = Math.toRadians(261.0);

    public static final int xboxControllerPort = 0;

}
