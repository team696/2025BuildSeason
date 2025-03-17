
package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.util.GameInfo.CoralScoringPosition;

public class AutoMoveSuperStructure extends Command {

  CoralScoringPosition position;

  double runRollers = 0;

  double postRollerState = 0;

  double readyToShoot;

  boolean waitForStall = false;

  public AutoMoveSuperStructure(CoralScoringPosition position, double runRollers,
      double postRollerState, boolean waitForStall) {
    this.position = position;

    this.runRollers = runRollers;

    this.postRollerState = postRollerState;

    this.waitForStall = waitForStall;

    addRequirements(Arm.get(), Elevator.get(), Wrist.get(), EndEffector.get());
  }

  public AutoMoveSuperStructure(CoralScoringPosition position, double runRollers, double postRollerState) {
    this(position, runRollers, postRollerState, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToShoot = 999999999;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Wrist.get().goToPosition(position);
    Elevator.get().goToPosition(position);
    if (Math.abs(Wrist.get().getPosition() - position.wristRot.in(Units.Rotation)) < .5
        && Math.abs(Elevator.get().getPosition() - position.height) < .5) {

      Arm.get().goToPosition(position);
      if (Math.abs(Arm.get().getPosition() - position.armRot.in(Units.Rotation)) < .5) {

        EndEffector.get().run(runRollers);
        if (readyToShoot > 99999) {
          readyToShoot = Timer.getFPGATimestamp();
        }
      } else {
        EndEffector.get().run(EndEffector.get().idlePower);
      }
    } else {
      EndEffector.get().run(EndEffector.get().idlePower);

      Arm.get().goToPosition(0);
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
    return (Timer.getFPGATimestamp() - readyToShoot > 0.4)
        && (!waitForStall || EndEffector.get().isStalling());
  }
}
