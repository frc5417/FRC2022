// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double intakeSpeed = .5;
    public static final double pusherSpeed = .5;

    public final static int driveMasterLeft = 11;
    public final static int driveMasterRight = 14;
    public final static int driveSlaveLeft1 = 12;
    public final static int driveSlaveLeft2 = 13;
    public final static int driveSlaveRight1 = 15;
    public final static int driveSlaveRight2 = 16;
    public final static int intake = 21;
    public final static int intestineBottom = 31;
    public final static int intestineKicker = 32;
    public final static int intestineBeamBreak = 1;
    public final static int shooter1 = 42;
    public final static int shooter2 = 43;
    public final static int turret = 51;
    public final static int climbMasterLeft = 61;
    public final static int climbMasterRight = 63;
    public final static int climbSlaveLeft = 62;
    public final static int climbSlaveRight = 64;
    public final static int solenoid = 75;

    //Drive Constants
    public static final double drivekP = .0312;
    public static final double drivekI = .000000000001;

    //Auton Constants
    public static final double driveGearingRatio = 18;

    //valid k-values are from characterization, commented values are from 2021
    public static final double kSAuto = .126; //.12; 
    public static final double kVAuto = 7.12; //4.69;
    public static final double kAAuto = .997; //.179;
    public static final double kPAuto = 0.0;

    public static final double kCosAuto = 0;
    public static final double rSquaredAuto = .998;
    public static final double trackWidth = .599426775;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);

    public static final double maxVoltage = 10;

    public static final double maxSpeed = 3; // m/s
    public static final double maxAcceleration =  3;//m/s^2

    public static final double wheelDiameter = 0.1016;
    
    public static final int climbLeft1 = 41;
    public static final int climbLeft2 = 42;
    public static final int climbRight1 = 43;
    public static final int climbRight2 = 44;

    public static final int climbBottomLeftLimitSwitch = 0;
    public static final int climbBottomRightLimitSwitch = 1;

    public static final int howLongClimbHasToBePushedDown = 900; // In ms.

    public static final int climberExtendPos = 0;
    public static final int climberRetractPos = 0;

    public static final int climberExtendSlightlyPos = 0;
    public static final int climberRetractSlightlyPos = 0;

    public static final double climberkP = 1;

    public static final int passiveSolenoid = 0;
    public static final int activeSolenoid = 1;

    public static final int driveLMaster = 14;
    public static final int driveLSlave = 15;
    public static final int driveRMaster = 1;
    public static final int driveRSlave = 17;

    //turret constants
    public static final double kPturn = 1.0;
    public static final double kPturnCenter = 1.0;
    public static final double minCommand = 0.1;
    public static final double limeLightErrorAllowed = 5;
    public static final double turretSpeed = 0.2;
    public static final double maxTurretTurn = 1.0; // change this (revolutions)

    //limelight constants
    public static final double targetHeight = 104.75; //inches
    public static final double limelightAngle = 28.66; // degrees
    public static final double limelightHeight = 26; //inches

    //shooter constants
    public static final double shooterP = 0.2;
    public static final double shooterI = 0;
    public static final int shootsetPointConst = -3100;
    public static final int shootMaxRPM = 5700;

}