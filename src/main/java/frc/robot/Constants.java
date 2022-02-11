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
    public static int climbLeft1 = 41;
    public static int climbLeft2 = 42;
    public static int climbRight1 = 43;
    public static int climbRight2 = 44;

    public static int climbBottomLeftLimitSwitch = 0;
    public static int climbBottomRightLimitSwitch = 1;

    public static int howLongClimbHasToBePushedDown = 900; // In ms.

    public static int climberExtendPos = 0;
    public static int climberRetractPos = 0;

    public static double climberkP = 1;

    public static int passiveSolenoid = 0;
    public static int activeSolenoid = 1;

    public static int driveLMaster = 14;
    public static int driveLSlave = 15;
    public static int driveRMaster = 1;
    public static int driveRSlave = 17;
}
