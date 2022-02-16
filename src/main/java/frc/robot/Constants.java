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

    public static double intakeSpeed = .5;
    public static double pusherSpeed = .5;

    public static final int masterRightMotor = 1; 
    public static final int slaveRightMotor = 17; 
    public static final int masterLeftMotor = 14;
    public static final int slaveLeftMotor = 15; 

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
    
    public static int climbLeft1 = 41;
    public static int climbLeft2 = 42;
    public static int climbRight1 = 43;
    public static int climbRight2 = 44;

    public static int climbBottomLeftLimitSwitch = 0;
    public static int climbBottomRightLimitSwitch = 1;

    public static int howLongClimbHasToBePushedDown = 900; // In ms.

    public static int climberExtendPos = 0;
    public static int climberRetractPos = 0;

    public static int climberExtendSlightlyPos = 0;
    public static int climberRetractSlightlyPos = 0;

    public static double climberkP = 1;

    public static int passiveSolenoid = 0;
    public static int activeSolenoid = 1;

    public static int driveLMaster = 14;
    public static int driveLSlave = 15;
    public static int driveRMaster = 1;
    public static int driveRSlave = 17;
    public static int shooter1 = 2;
    public static int shooter2 = 3;
    public static int turret = 4;

    public static double kP = 1.0;
    public static double minCommand = 0.1;
    public static double limeLightErrorAllowed = .1;

    public static final double targetHeight = 104.75; //inches
    public static final double limelightAngle = 28.66; // degrees
    public static final double limelightHeight = 26; //inches

    public static double shooterP = 0.2;
    public static double shooterI = 0;
    public static int shootsetPointVariable = -3100;
    public static int shootMaxRPM = 5700;

}