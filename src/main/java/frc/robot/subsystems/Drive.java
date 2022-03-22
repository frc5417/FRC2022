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

public class Drive extends SubsystemBase {
  /** Creates a new Drivetrain. */
private CANSparkMax driveMasterL;
  private CANSparkMax driveSlaveL1;
  private CANSparkMax driveSlaveL2;
  private CANSparkMax driveMasterR;
  private CANSparkMax driveSlaveR1;
  private CANSparkMax driveSlaveR2;
  
  private DifferentialDrive drive;
  private RelativeEncoder neoEncoderML;
  private RelativeEncoder neoEncoderMR;
  private RelativeEncoder neoEncoderSL1;
  private RelativeEncoder neoEncoderSL2;
  private RelativeEncoder neoEncoderSR1;
  private RelativeEncoder neoEncoderSR2;
  private DifferentialDriveOdometry driveOdom;
  private AHRS gyro;

  public Drive() {
    driveMasterL = new CANSparkMax(Constants.driveMasterLeft, MotorType.kBrushless);
    driveSlaveL1 = new CANSparkMax(Constants.driveSlaveLeft1, MotorType.kBrushless);
    driveSlaveL2 = new CANSparkMax(Constants.driveSlaveLeft2, MotorType.kBrushless);
    driveMasterR = new CANSparkMax(Constants.driveMasterRight, MotorType.kBrushless);
    driveSlaveR1 = new CANSparkMax(Constants.driveSlaveRight1, MotorType.kBrushless);
    driveSlaveR2 = new CANSparkMax(Constants.driveSlaveRight2, MotorType.kBrushless);
    driveMasterL.setInverted(true);
    driveSlaveL1.setInverted(true);
    driveSlaveL2.setInverted(true);
    driveSlaveL1.follow(driveMasterL);
    driveSlaveL2.follow(driveMasterL);
    driveSlaveR1.follow(driveMasterR);
    driveSlaveR2.follow(driveMasterR);

    neoEncoderML = driveMasterL.getEncoder();
    neoEncoderMR = driveMasterR.getEncoder();
    neoEncoderSL1 = driveSlaveL1.getEncoder();
    neoEncoderSL1 = driveSlaveL2.getEncoder();
    neoEncoderSR1 = driveSlaveR1.getEncoder();
    neoEncoderSR2 = driveSlaveR2.getEncoder();
    neoEncoderML.setPositionConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderML.setVelocityConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderMR.setPositionConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    neoEncoderMR.setVelocityConversionFactor(2*Math.PI*Constants.wheelDiameter / Constants.driveGearingRatio);
    
    setIdleModes(IdleMode.kBrake);
    setPIDConstants();
    gyro = new AHRS(Port.kMXP);
    driveOdom  = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    drive  = new DifferentialDrive(driveMasterL, driveMasterR);
    drive.setSafetyEnabled(false);
  }

  private void setIdleModes(IdleMode mode) {
    driveMasterL.setIdleMode(mode);
    driveSlaveL1.setIdleMode(mode);
    driveSlaveL2.setIdleMode(mode);
    driveMasterR.setIdleMode(mode);
    driveSlaveR1.setIdleMode(mode);
    driveSlaveR2.setIdleMode(mode);
  }

  private void setPIDConstants() {
    driveMasterL.getPIDController().setP(Constants.drivekP);
    driveSlaveL1.getPIDController().setP(Constants.drivekP);
    driveSlaveL2.getPIDController().setP(Constants.drivekP);
    driveMasterR.getPIDController().setP(Constants.drivekP);
    driveSlaveR1.getPIDController().setP(Constants.drivekP);
    driveSlaveR2.getPIDController().setP(Constants.drivekP);

    driveMasterL.getPIDController().setI(Constants.drivekI);
    driveSlaveL1.getPIDController().setI(Constants.drivekI);
    driveSlaveL2.getPIDController().setI(Constants.drivekI);
    driveMasterR.getPIDController().setI(Constants.drivekI);
    driveSlaveR1.getPIDController().setI(Constants.drivekI);
    driveSlaveR2.getPIDController().setI(Constants.drivekI);
  }

  public void setPower(Joystick joystick){
    driveMasterL.set(joystick.getRawAxis(1));
    driveMasterR.set(joystick.getRawAxis(5));
    // System.out.println("left pos:" + neoEncoderML.getPosition());
    // System.out.println("right pos:" + neoEncoderMR.getPosition());
  }
  
  public void setPower(double leftPower, double rightPower){

    if(Math.abs(leftPower) > .2) {
      driveMasterL.set(leftPower);
    }
    else if(Math.abs(leftPower) > .6) {
      driveMasterL.set(.6);
    }
    else {
      driveMasterL.set(0);
    }
    if(Math.abs(rightPower) > .2) {
      driveMasterR.set(rightPower);
    }
    else if(Math.abs(rightPower) > .6){
      driveMasterR.set(.6);
    }
    else {
      driveMasterR.set(0);
    }
  }

  public AHRS getGyro () {
    return gyro;
  }

  public CANSparkMax getDriveMasterL () {
    return driveMasterL;
  }

  public CANSparkMax getDriveMasterR () {
    return driveMasterR;
  }

  public CANSparkMax getDriveSlaveL1 () {
    return driveSlaveL1;
  }

  public CANSparkMax getDriveSlaveL2 () {
    return driveSlaveL2;
  }

  public CANSparkMax getDriveSlaveR1 () {
    return driveSlaveR1;
  }

  public CANSparkMax getDriveSlaveR2 () {
    return driveSlaveR2;
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
    rawMotorPower(leftVolts / RobotController.getBatteryVoltage(), -rightVolts / RobotController.getBatteryVoltage());
    drive.feed();
  }

  public void autonPower(double driveMasterLPos, double driveMasterRPos, double driveSlaveL1Pos, double driveSlaveL2Pos, double driveSlaveR1Pos, double driveSlaveR2Pos) {
    setIdleModes(IdleMode.kBrake);
    driveMasterL.getPIDController().setReference(driveMasterLPos, CANSparkMax.ControlType.kPosition);
    driveMasterR.getPIDController().setReference(driveMasterRPos, CANSparkMax.ControlType.kPosition);
    driveSlaveL1.getPIDController().setReference(driveSlaveL1Pos, CANSparkMax.ControlType.kPosition);
    driveSlaveL2.getPIDController().setReference(driveSlaveL2Pos, CANSparkMax.ControlType.kPosition);
    driveSlaveR1.getPIDController().setReference(driveSlaveR1Pos, CANSparkMax.ControlType.kPosition);
    driveSlaveR2.getPIDController().setReference(driveSlaveR2Pos, CANSparkMax.ControlType.kPosition);
  }

  public void setDriveTrainRampRate(double rightRampRate, double leftRampRate) {
    driveMasterL.setClosedLoopRampRate(leftRampRate);
    driveMasterR.setClosedLoopRampRate(rightRampRate);
    driveSlaveL1.setClosedLoopRampRate(leftRampRate);
    driveSlaveL2.setClosedLoopRampRate(leftRampRate);
    driveSlaveR1.setClosedLoopRampRate(rightRampRate);
    driveSlaveR2.setClosedLoopRampRate(rightRampRate);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(neoEncoderML.getVelocity(), neoEncoderMR.getVelocity());
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
    neoEncoderML.setPosition(0);
    neoEncoderMR.setPosition(0);
    neoEncoderSL1.setPosition(0);
    neoEncoderSL2.setPosition(0);
    neoEncoderSR1.setPosition(0);
    neoEncoderSR2.setPosition(0);
  }
  
  public double getAverageEncoderDistance(){
    return(neoEncoderML.getPosition() + neoEncoderMR.getPosition()) / 2.0;
  }
  public double getLeftDistance(){
    return neoEncoderML.getPosition();
  }
  public double getRightDistance(){
    return neoEncoderMR.getPosition();
  }
  public RelativeEncoder getLeftNeoEncoder(){
    return neoEncoderML;
  }
  public RelativeEncoder getRightNeoEncoder(){
    return neoEncoderMR;
  }

  @Override
  public void periodic() {
    driveOdom.update(Rotation2d.fromDegrees(getHeading()), neoEncoderML.getPosition(), neoEncoderMR.getPosition());
  }
}
