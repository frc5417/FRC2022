// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.RelativeEncoder;

import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonDrive extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private Drivetrain drivetrain = new Drivetrain();
  private RelativeEncoder neoEncoderL = drivetrain.getDriveMasterL().getEncoder();
  private RelativeEncoder neoEncoderR = drivetrain.getDriveMasterR().getEncoder();
  private RelativeEncoder neoEncoderL2 = drivetrain.getDriveSlaveL().getEncoder();
  private RelativeEncoder neoEncoderR2 = drivetrain.getDriveSlaveR().getEncoder();
  private DifferentialDriveOdometry driveOdom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  private AHRS gyro = new AHRS(Port.kUSB);

  private void encoderReset() {
    neoEncoderL.setPosition(0);
    neoEncoderR.setPosition(0);
    neoEncoderL2.setPosition(0);
    neoEncoderR2.setPosition(0);
  }

  public AutonDrive() {
    neoEncoderL.setPositionConversionFactor(Math.PI * .1524 / Constants.driveGearingRatio);
    neoEncoderR.setPositionConversionFactor(Math.PI * .1524 / Constants.driveGearingRatio);
    neoEncoderL.setVelocityConversionFactor((Math.PI * .1524 / Constants.driveGearingRatio) / 60);
    neoEncoderR.setVelocityConversionFactor((Math.PI * .1524 / Constants.driveGearingRatio) / 60);
    encoderReset();
    gyro.zeroYaw();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(neoEncoderL.getVelocity(), -neoEncoderR.getVelocity());
  }
  public Pose2d getPose() {
    return driveOdom.getPoseMeters();
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public boolean resetOdometry(Pose2d pose) {
    encoderReset();
    gyro.zeroYaw();
    driveOdom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    return true;
  }
  

  public double getAverageEncoderDistance(){
    return(neoEncoderL.getPosition() + neoEncoderR.getPosition()) / 2.0;
  }
  public double getLeftDistance(){
    return neoEncoderL.getPosition();
  }
  public double getRightDistance(){
    return neoEncoderR.getPosition();
  }
  public RelativeEncoder getLeftNeoEncoder(){
    return neoEncoderL;
  }
  public RelativeEncoder getRightNeoEncoder(){
    return neoEncoderR;
  }

  @Override
  public void periodic() {
    driveOdom.update(Rotation2d.fromDegrees(getHeading()), neoEncoderL.getPosition(), -neoEncoderR.getPosition());
  }
}
