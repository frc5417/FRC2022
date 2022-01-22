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
    public static final int masterRightMotor = 1; 
    public static final int slaveRightMotor = 17; 
    public static final int masterLeftMotor = 14; 
    public static final int slaveLeftMotor = 15; 
    public static final int turretPort = 9;
    
    public static final double wheelDiameter = 0.1016;
    public static final double driveTrain_width = .4699;
    public static final double driveGearingRatio = 18;

    //Drive Constants
    public static final double drivekP = .0312;
    public static final double drivekI = .000000000001;

    // Turret Constants
    public static final double turretkP = .007;
    public static final double turretkI = .00000000000001;
    public static final double turretkD = 0;
    public static final double turretClimbEncoderPos = 80000;}
