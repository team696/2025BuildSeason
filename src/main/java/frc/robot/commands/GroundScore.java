// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundCoral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundScore extends Command {
  double scoringPos=14;
  double readyToShoot;
  /** Creates a new GroundScore. */
  public GroundScore() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(GroundCoral.get());

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToShoot=999999999;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GroundCoral.get().position(scoringPos);
    if(Math.abs(GroundCoral.get().getPosition()-scoringPos)<0.5){
      GroundCoral.get().rollerMotor.set(1.);
      if (readyToShoot > 99999) {
        readyToShoot = Timer.getFPGATimestamp();
      }
    }else{
      GroundCoral.get().rollerMotor.stopMotor();
    }
    if (readyToShoot > 99999) {
        readyToShoot = Timer.getFPGATimestamp();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GroundCoral.get().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - readyToShoot > 1;
  }
}
