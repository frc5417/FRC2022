// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1;
  private final CANSparkMax shooter2;

  /** Creates a new Shooter. */
  public Shooter() {
    this.shooter1 = new CANSparkMax(Constants.shooter1, MotorType.kBrushless);
    this.shooter2 = new CANSparkMax(Constants.shooter2, MotorType.kBrushless);

    this.shooter1.getPIDController().setP(Constants.shooterP);
    this.shooter2.getPIDController().setP(Constants.shooterP);

    this.shooter1.getPIDController().setI(Constants.shooterI);
    this.shooter2.getPIDController().setI(Constants.shooterI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity() {
    this.setVelocity(Constants.shootsetPointVariable*Constants.shootMaxRPM);
  }

  public void setVelocity(double velocity){
    this.shooter1.setIdleMode(IdleMode.kCoast);
    this.shooter2.setIdleMode(IdleMode.kCoast);

    this.shooter1.getPIDController().setReference(velocity, ControlType.kVelocity);
    this.shooter2.getPIDController().setReference(velocity, ControlType.kVelocity);
  }
}
