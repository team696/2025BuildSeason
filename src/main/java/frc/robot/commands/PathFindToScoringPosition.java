package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathFindToScoringPosition extends Command {

  Pose2d DesiredPose;
  Command pathFindingCommand;

  public PathFindToScoringPosition() {
  }

  @Override
  public void initialize() {
    DesiredPose = new Pose2d();

    pathFindingCommand = AutoBuilder.pathfindToPose(DesiredPose,
        new PathConstraints(3., 3., Units.degreesToRadians(540), Units.degreesToRadians(540)))
        .andThen(new PIDtoPosition(DesiredPose));

    pathFindingCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return pathFindingCommand.isFinished();
  }
}
