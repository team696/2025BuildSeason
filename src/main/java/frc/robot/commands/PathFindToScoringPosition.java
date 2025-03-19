package frc.robot.commands;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GameInfo;

public class PathFindToScoringPosition extends Command {

  Pose2d DesiredPose;
  Command pathFindingCommand;

  public PathFindToScoringPosition() {
  }

  public Translation2d Nearest() {
    Translation2d currentPose = Swerve.get().getPose().getTranslation();

    Translation2d closestPose = new Translation2d();
    double closestDistance = Double.MAX_VALUE;

    for (Map<GameInfo.ReefSide, Translation2d> face : GameInfo.ScoringPosesBlue.values()) {
      for (Translation2d side : face.values()) {
        if (side.getDistance(currentPose) < closestDistance) {
          closestDistance = side.getDistance(currentPose);
          closestPose = side;
        }
      }
    }
    return closestPose;
  }

  public Translation2d Selected() {
    // TOOD
    return new Translation2d();
  }

  @Override
  public void initialize() {
    Translation2d nearestPosition = Nearest();
    DesiredPose = new Pose2d(nearestPosition, Swerve.get().FaceHexFace(nearestPosition));

    pathFindingCommand = AutoBuilder.pathfindToPose(DesiredPose,
        new PathConstraints(3., 3., Units.degreesToRadians(540), Units.degreesToRadians(540)))
        .andThen(new PIDtoPosition(DesiredPose));

    pathFindingCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    if (pathFindingCommand.isScheduled()) {
      pathFindingCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return pathFindingCommand.isFinished();
  }
}
