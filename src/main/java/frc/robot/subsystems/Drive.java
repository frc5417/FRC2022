// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final CANSparkMax driveLMaster;
  private final CANSparkMax driveLSlave;
  private final CANSparkMax driveRMaster;
  private final CANSparkMax driveRSlave;

  /** Creates a new Drive. */
  public Drive() {
    driveLMaster = new CANSparkMax(Constants.driveLMaster, MotorType.kBrushless);
    driveLSlave = new CANSparkMax(Constants.driveLSlave, MotorType.kBrushless);
    driveRMaster = new CANSparkMax(Constants.driveRMaster, MotorType.kBrushless);
    driveRSlave = new CANSparkMax(Constants.driveRSlave, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double leftPower, double rightPower){
    driveLMaster.setIdleMode(IdleMode.kCoast);
    driveLSlave.setIdleMode(IdleMode.kCoast);
    driveRMaster.setIdleMode(IdleMode.kCoast);
    driveRSlave.setIdleMode(IdleMode.kCoast);

    driveLSlave.follow(driveLMaster);
    driveRSlave.follow(driveRMaster);

    driveLMaster.set(-leftPower);
    driveRMaster.set(rightPower);
  }
}
