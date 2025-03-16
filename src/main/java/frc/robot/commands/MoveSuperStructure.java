// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.HumanControls;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.GameInfo.CoralScoringPosition;

public class MoveSuperStructure extends Command {

  CoralScoringPosition position;

  double runRollers = 0;

  double postRollerState = 0;

  boolean requirePress = true;

  public MoveSuperStructure(CoralScoringPosition position, double runRollers, boolean requirePress,
      double postRollerState) {
    this.position = position;

    this.runRollers = runRollers;

    this.postRollerState = postRollerState;

    this.requirePress = requirePress;

    addRequirements(Arm.get(), Elevator.get(), Wrist.get(), EndEffector.get());
  }

  public MoveSuperStructure(CoralScoringPosition position, double runRollers) {
    this(position, runRollers, true, 0.);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Wrist.get().goToPosition(position);
    Elevator.get().goToPosition(position);

    if (Math.abs(Wrist.get().getPosition() - position.wristRot.in(Units.Rotation)) < .5
        && Math.abs(Elevator.get().getPosition() - position.height) < .5) {

      Arm.get().goToPosition(position);
      if (Math.abs(Arm.get().getPosition() - position.armRot.in(Units.Rotation)) < .5
          && (!requirePress || HumanControls.OperatorPanel2025.releaseCoral.getAsBoolean())) {

        EndEffector.get().run(runRollers);
      } else {
        EndEffector.get().run(EndEffector.get().idlePower);
      }
    } else {
      Arm.get().goToPosition(0);
      EndEffector.get().run(EndEffector.get().idlePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.get().stop();
    Wrist.get().stop();
    Elevator.get().stop();
    EndEffector.get().idlePower = postRollerState;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
