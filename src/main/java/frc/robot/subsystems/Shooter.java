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

  /** Creates a new Shooter. */
  public Shooter() {
    this.shooter1 = new CANSparkMax(Constants.shooter1, MotorType.kBrushless);

    this.shooter1.getPIDController().setP(Constants.shooterP);

    this.shooter1.getPIDController().setI(Constants.shooterI);
    
    this.shooter1.getPIDController().setD(Constants.shooterD);

    this.shooter1.getPIDController().setFF(Constants.shooterFF);
    this.shooter1.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAutoVelocity() {
    this.setVelocity(Constants.shootsetPointConst*Constants.shootMaxRPM);
  }
  public void setPower(double power){
    this.shooter1.set(power);
  }
  public void setVelocity(double velocity){

    this.shooter1.getPIDController().setReference(velocity, ControlType.kVelocity);
    System.out.println(this.shooter1.getEncoder().getVelocity());
  }

  public CANSparkMax getShooter1(){
    return shooter1;
  }
}