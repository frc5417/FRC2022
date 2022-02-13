// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.RobotController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax driveMasterR;
  private CANSparkMax driveSlaveR;
  private CANSparkMax driveMasterL;
  private CANSparkMax driveSlaveL;
  private DifferentialDrive drive;
  private RelativeEncoder neoEncoderL;
  private RelativeEncoder neoEncoderR;
  private RelativeEncoder neoEncoderL2;
  private RelativeEncoder neoEncoderR2;
  private DifferentialDriveOdometry driveOdom;
  private AHRS gyro;

  public Drivetrain() {
    driveMasterR = new CANSparkMax(Constants.masterRightMotor, MotorType.kBrushless);
    driveSlaveR = new CANSparkMax(Constants.slaveRightMotor, MotorType.kBrushless);
    driveMasterL = new CANSparkMax(Constants.masterLeftMotor, MotorType.kBrushless);
    driveSlaveL = new CANSparkMax(Constants.slaveLeftMotor, MotorType.kBrushless);
    driveMasterR.setInverted(true);
    driveSlaveR.setInverted(true);
    neoEncoderR = driveMasterR.getEncoder();
    neoEncoderL = driveMasterL.getEncoder();
    neoEncoderL2 = driveSlaveL.getEncoder();
    neoEncoderR2 = driveSlaveR.getEncoder();
    neoEncoderR.setPositionConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderL.setPositionConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderR.setVelocityConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderL.setVelocityConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    driveSlaveL.follow(driveMasterL);
    driveSlaveR.follow(driveMasterR);
    setIdleModes(IdleMode.kCoast);
    setPIDConstants();
    gyro = new AHRS(Port.kUSB); //big navx: kmxp
    driveOdom  = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    drive  = new DifferentialDrive(driveMasterL, driveMasterR);
    drive.setSafetyEnabled(false);
  }

  private void setIdleModes(IdleMode mode) {
    driveMasterL.setIdleMode(mode);
    driveMasterR.setIdleMode(mode);
    driveSlaveL.setIdleMode(mode);
    driveSlaveR.setIdleMode(mode);
  }

  private void setPIDConstants() {
    driveMasterL.getPIDController().setP(Constants.drivekP);
    driveMasterR.getPIDController().setP(Constants.drivekP);
    driveSlaveL.getPIDController().setP(Constants.drivekP);
    driveSlaveR.getPIDController().setP(Constants.drivekP);
    driveMasterL.getPIDController().setI(Constants.drivekI);
    driveMasterR.getPIDController().setI(Constants.drivekI);
    driveSlaveL.getPIDController().setI(Constants.drivekI);
    driveSlaveR.getPIDController().setI(Constants.drivekI);
  }

  public void setPower(Joystick joystick){
    driveMasterL.set(joystick.getRawAxis(1));
    driveMasterR.set(joystick.getRawAxis(5));
    System.out.println("left pos:" + neoEncoderL.getPosition());
    System.out.println("right pos:" + neoEncoderR.getPosition());
  }
  
  // public void setPower(double leftPower, double rightPower){
  //   if(Math.abs(leftPower) > .15){
  //     driveMasterL.set(-leftPower);
  //   } else {
  //     driveMasterL.set(0);
  //   }
  //   if(Math.abs(rightPower) > .15){
  //     driveMasterR.set(rightPower);
  //   } else {
  //     driveMasterR.set(0);
  //   }
  // }

  public AHRS getGyro () {
    return gyro;
  }

  public CANSparkMax getDriveSlaveL () {
    return driveSlaveL;
  }

  public CANSparkMax getDriveSlaveR () {
    return driveSlaveR;
  }

  public CANSparkMax getDriveMasterL () {
    return driveMasterL;
  }

  public CANSparkMax getDriveMasterR () {
    return driveMasterR;
  }

  public DifferentialDrive getDifferentialDrive() {
    return drive;
  }

  public void setMaxOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
  }

  public void rawMotorPower(double leftPower, double rightPower) {
    driveMasterL.set(leftPower);
    driveMasterR.set(rightPower);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    rawMotorPower(leftVolts / RobotController.getBatteryVoltage(), rightVolts / RobotController.getBatteryVoltage());
    drive.feed();
  }

  public void autonPower(double driveMasterLPos, double driveMasterRPos, double driveSlaveLPos, double driveSlaveRPos) {
    setIdleModes(IdleMode.kBrake);
    driveMasterL.getPIDController().setReference(driveMasterLPos, CANSparkMax.ControlType.kPosition);
    driveMasterR.getPIDController().setReference(driveMasterRPos, CANSparkMax.ControlType.kPosition);
    driveSlaveL.getPIDController().setReference(driveSlaveLPos, CANSparkMax.ControlType.kPosition);
    driveSlaveR.getPIDController().setReference(driveSlaveRPos, CANSparkMax.ControlType.kPosition);
  }

  public void setDriveTrainRampRate(double rightRampRate, double leftRampRate) {
    driveMasterR.setClosedLoopRampRate(rightRampRate);
    driveMasterL.setClosedLoopRampRate(leftRampRate);
    driveSlaveR.setClosedLoopRampRate(rightRampRate);
    driveSlaveL.setClosedLoopRampRate(leftRampRate);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(neoEncoderL.getVelocity(), neoEncoderR.getVelocity());
  }
  public Pose2d getPose() {
    return driveOdom.getPoseMeters();
  }
  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360); //default for navx: clockwise is positive
  }
  public boolean resetOdometry(Pose2d pose) {
    encoderReset();
    gyro.zeroYaw();
    driveOdom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    return true;
  }
  
  private void encoderReset() {
    neoEncoderL.setPosition(0);
    neoEncoderR.setPosition(0);
    neoEncoderL2.setPosition(0);
    neoEncoderR2.setPosition(0);
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
    driveOdom.update(Rotation2d.fromDegrees(getHeading()), neoEncoderL.getPosition(), neoEncoderR.getPosition());
  }
}
