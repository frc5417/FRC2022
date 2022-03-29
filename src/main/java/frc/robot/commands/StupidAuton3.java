package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class StupidAuton3 extends CommandBase {
  /** Creates a new StupidAuton. */
  private final Drive drive;
  private final Shooter shoot;
  private final Intake intake;
  private final Limelight limelight;
  private int count;
  private double[] wheelSpeeds;
  public StupidAuton3(Drive drive, Shooter shoot, Intake intake, Limelight limelight) {
    this.drive = drive;
    this.shoot = shoot;
    this.intake = intake;
    this.limelight = limelight;
    addRequirements(drive, shoot, intake, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forwards and intake
    if(count <= 1000){
      shoot.setVelocity(0);
      intake.runIntestine(1);
      drive.setPower(0.6, 0.6);
    }
    // Go backwards a little
    else if(count <= 1250){
      intake.runIntestine(0);
      drive.setPower(-0.6, -0.6);
    }

    // Turn 360
    else if(count <= 1900){
      drive.setPower(-0.6, 0.6);
      intake.runIntestine(0);
    }

    // Go forward a bit
    else if(count <= 2650) {
      drive.setPower(0.6, 0.6);
    }
    // Shoot one ball
    else if(count <= 3400) {
      drive.setPower(0.0, 0.0);
      shoot.setVelocity(2500);
    }
    // Shoot second ball 
    else if(count <= 4150) {
      shoot.setVelocity(2500);
    }
    //Turn off shooter, turn 90 deg. left
    else if(count <= 4410) {
      shoot.setVelocity(0);
      drive.setPower(-0.6, 0.6);
    }
    // Go forward, intake 1 ball
    else if(count <= 5710) {
      intake.runIntestine(1);
      drive.setPower(0.6, 0.6);
    }
    // 110 degree right turn to hoop
    else if(count <=5915) {
      intake.runIntestine(0);
      drive.setPower(0.6, -0.6);
    }
    // Go forward a bit 
    else if (count <= 6165) {
      drive.setPower(0.6, 0.6);
    }
    // Shoot one ball 
    else if ( count <= 6965) {
      drive.setPower(0.0, 0.0);
      shoot.setVelocity(2500);
    }
    // Stops robot
    else{
      drive.setPower(0, 0);
      shoot.setPower(0);
      intake.runIntestine(0);
    }
    count+=20;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setPower(0, 0);
    shoot.setPower(0);
    intake.runIntestine(0);            
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (count > 8750){
      return true;
    }
    return false;
  }
}