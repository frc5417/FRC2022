// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax climbL1 = new CANSparkMax(Constants.climbLeft1, MotorType.kBrushless);
  private CANSparkMax climbL2 = new CANSparkMax(Constants.climbLeft2, MotorType.kBrushless);
  private CANSparkMax climbR1 = new CANSparkMax(Constants.climbRight1, MotorType.kBrushless);
  private CANSparkMax climbR2 = new CANSparkMax(Constants.climbRight2, MotorType.kBrushless);

  private RelativeEncoder climbL1Encoder = climbL1.getEncoder();
  private RelativeEncoder climbL2Encoder = climbL2.getEncoder();
  private RelativeEncoder climbLR1Encoder = climbR1.getEncoder();
  private RelativeEncoder climbR2Encoder = climbR2.getEncoder();

  private Solenoid sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.passiveSolenoid);
  private Solenoid sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.activeSolenoid);

  public Climber() {
    climbL2.follow(climbL1);
    climbR2.follow(climbR1);
  }

  public void extendUp(){
    
  }

}
