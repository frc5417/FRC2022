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

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax driveSlaveL = new CANSparkMax(Constants.slaveLeftMotor, MotorType.kBrushless);
  private CANSparkMax driveMasterR = new CANSparkMax(Constants.masterRightMotor, MotorType.kBrushless);
  private CANSparkMax driveSlaveR = new CANSparkMax(Constants.slaveRightMotor, MotorType.kBrushless);
  private CANSparkMax driveMasterL = new CANSparkMax(Constants.masterLeftMotor, MotorType.kBrushless);
  private DifferentialDrive drive = new DifferentialDrive(driveMasterL, driveMasterR);

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

  public Drivetrain() {
    driveSlaveL.follow(driveMasterL);
    driveSlaveR.follow(driveMasterR);
    setIdleModes(IdleMode.kCoast);
    setPIDConstants();
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

  public void setPower(double leftPower, double rightPower){
    rawMotorPower(-Math.pow(leftPower, 3), Math.pow(rightPower, 3));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    rawMotorPower(leftVolts / RobotController.getBatteryVoltage(), -rightVolts / RobotController.getBatteryVoltage());
    drive.feed();
  }

  public void tankDriveVoltsNerfed(double leftVolts, double rightVolts){
    rawMotorPower(.5*leftVolts / RobotController.getBatteryVoltage(), -.5*rightVolts / RobotController.getBatteryVoltage());
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
}
