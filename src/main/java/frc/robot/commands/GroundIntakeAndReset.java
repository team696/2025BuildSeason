// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundCoral;

public class GroundIntakeAndReset extends Command {
  boolean hasStalled;
  public GroundIntakeAndReset() {
    addRequirements(GroundCoral.get());
  }

  @Override
  public void initialize() {
    hasStalled=false;

  }

  @Override
  public void execute() {
    GroundCoral.get().rollerMotor.set(1.);
    if(GroundCoral.get().angleMotor.getStatorCurrent().getValue().in(Amps)>100){
      hasStalled=true;
      GroundCoral.get().resetPosition(14);
    }
    if(hasStalled){
      GroundCoral.get().angleMotor.set(0);
    }else{
      GroundCoral.get().angleMotor.set(0.2);
    }
  }

  @Override
  public void end(boolean interrupted) {
    GroundCoral.get().stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
