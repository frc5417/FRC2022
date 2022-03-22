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

    public static final double intakeSpeed = 1;

    public final static int driveMasterLeft = 10;
    public final static int driveMasterRight = 13;
    public final static int driveSlaveLeft1 = 11;
    public final static int driveSlaveLeft2 = 12;
    public final static int driveSlaveRight1 = 14;
    public final static int driveSlaveRight2 = 15;
    public final static int intake = 21;
    public final static int intestineBottom = 22;
    public final static int intestineKicker = 20;
    public final static int intestineBeamBreak = 1;
    public final static int shooter1 = 30;
    public final static int turret = 32;
    public static final int climbRight1 = 5;
    public static final int climbRight2 = 2;
    public static final int climbLeft1 = 3;
    public static final int climbLeft2 = 4;
    public final static int solenoid = 75;

    //Drive Constants
    public static final double drivekP = 0.025; //used to be 0.0312
    public static final double drivekI = 0.000000000001;

    // Auton Constants
    public static final double driveGearingRatio = 18;

    // valid k-values are from characterization, commented values are from 2021
    public static final double kSAuto = .12;
    public static final double kVAuto = 4.69;
    public static final double kAAuto = .179;
    public static final double kPAuto = 0.0;

    public static final double kCosAuto = 0;
    public static final double rSquaredAuto = .998;
    public static final double trackWidth = .599426775;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);

    public static final double maxVoltage = 10;

    public static final double maxSpeed = 3; // m/s
    public static final double maxAcceleration =  3;//m/s^2

    public static final double wheelDiameter = 0.1524;

    public static final int climbBottomLeftLimitSwitch = 0;
    public static final int climbBottomRightLimitSwitch = 1;

    public static final int howLongClimbHasToBePushedDown = 900; // In ms.

    public static final int climberExtendPos = 25;
    public static final int climberRetractPos = 0;

    public static final int climberExtendSlightlyPos = 30;
    public static final int climberRetractSlightlyPos = 20;

    public static final double climberkP = 2;

    public static final int climbBottomLSolenoid = 0;
    public static final int climbBottomRSolenoid = 1;
    public static final int intakeRSolenoid = 2;
    public static final int intakeLSolenoid = 3;
    public static final int climbTopLSolenoid = 4;
    public static final int climbTopRSolenoid = 5;

    public static final int driveLMaster = 14;
    public static final int driveLSlave = 15;
    public static final int driveRMaster = 1;
    public static final int driveRSlave = 17;

    // turret constants
    public static final double kPturn = 1.0;
    public static final double kPturnCenter = 1.0;
    public static final double minCommand = 0.1;
    public static final double limeLightErrorAllowed = 5;
    public static final double turretSpeed = 0.2;
    public static final double maxTurretTurn = 120.0; // degrees
    public static final double turretWheelCircumference = 39.4;
    public static final double motorWheelCircumference = 6.5;
    public static final double turretMotorGearRatio = 1/20;
    public static final double turretRatio = turretMotorGearRatio * motorWheelCircumference / turretWheelCircumference;

    // limelight constants
    public static final double targetHeight = 104.75; //inches
    public static final double limelightAngle = 28.66; // degrees
    public static final double limelightHeight = 26; //inches
    public static final double driveMinCommand = 0.1;

    //shooter constants
    public static final double shooterP = .0005;
    public static final double shooterI = 0.0;
    public static final double shooterFF = 0.00018;
    public static final int shootsetPointConst = -3100;
    public static final int shootMaxRPM = 5700;

}