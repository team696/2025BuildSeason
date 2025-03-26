// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team696.lib.Util;

/**
 * A class contianing all the positions needed to move the superstructure into a
 * scoring position
 */
public class GameInfo {
  public static class CoralScoringPosition {
    /**
     * Creates a new CoralScoringPosition
     * 
     * @param height   The height of the elevator (zero represents the elevator's
     *                 lowest position), positive=up
     * @param armRot   The rotation of the arm
     * @param wristRot The rotation of the wrist
     */
    public CoralScoringPosition(double height, double armRot, double wristRot) {
      this.height = height;
      this.armRot = Rotation.of(armRot);
      this.wristRot = Rotation.of(wristRot);
    }

    public double height;
    // Rotations of the Motor, Im too lazy to change off angle now
    public Angle armRot;
    public Angle wristRot;
  }

  public static CoralScoringPosition Net, ground, Processor, ClimbUp, ClimbDown, L2Algae, L3Algae;

  /* Looking at the Index Dead On */
  public enum ReefSide {
    Right,
    Left
  }

  /* Labels For Blue Side Of Field, Relative To Center Of Hex */
  public enum Index {
    One, // -X
    Two, // -X, +Y
    Three, // +X, +Y
    Four, // +X
    Five, // +X, -Y
    Six // -X, -Y
  }

  public final static Distance fieldLengthMeters = Feet.of(57.53);
  public final static Distance fieldWidthMeters = Feet.of(26.75);

  public static Translation2d mirrorTranslation(Translation2d starting) {
    return new Translation2d(17.55- starting.getX(), starting.getY());
  }
  public static Translation2d mirrorTranslationXY(Translation2d starting) {
    return new Translation2d(17.55- starting.getX(), 8.05-starting.getY());
  }

  public static Map<Index, Map<ReefSide, Pose2d>> getScoringPoses(){
    return (Util.getAlliance()==Alliance.Red)?ScoringPosesRed:ScoringPosesBlue;
  }

  public final static Translation2d blueReef = new Translation2d(4.5, 4.);

  public final static Map<Index, Map<ReefSide, Pose2d>> ScoringPosesBlue, ScoringPosesRed;

  public enum Position {
    L1,
    L2,
    L3,
    L4,
    Intake
  }

  public enum RobotSide {
    Front,
    Back
  }

  public final static Map<Position, Map<RobotSide, CoralScoringPosition>> RobotState;
  public final static double wristOffset = 1.15;
  static {
    ScoringPosesBlue = Map.of(
        Index.One, Map.of(
            ReefSide.Right, new Pose2d(3.29, 3.76, Rotation2d.fromDegrees(90)),
            ReefSide.Left, new Pose2d(3.3, 4.10, Rotation2d.fromDegrees(90))),

        Index.Two, Map.of(
            ReefSide.Right, new Pose2d(3.66, 4.96, Rotation2d.fromDegrees(30)),
            ReefSide.Left, new Pose2d(4.02, 5.1, Rotation2d.fromDegrees(30))),

        Index.Three, Map.of(
            ReefSide.Right, new Pose2d(4.87, 5.21, Rotation2d.fromDegrees(-30)),
            ReefSide.Left, new Pose2d(5.16, 5.02, Rotation2d.fromDegrees(-30))),

        Index.Four, Map.of(
            ReefSide.Right, new Pose2d(5.66, 4.28, Rotation2d.fromDegrees(-90)),
            ReefSide.Left, new Pose2d(5.68, 3.91, Rotation2d.fromDegrees(-90))),

        Index.Five, Map.of(
            ReefSide.Right, new Pose2d(5.30, 3.10, Rotation2d.fromDegrees(-150)),
            ReefSide.Left, new Pose2d(5, 2.94, Rotation2d.fromDegrees(-150))),

        Index.Six, Map.of(
            ReefSide.Right, new Pose2d(4.09, 2.87, Rotation2d.fromDegrees(150)),
            ReefSide.Left, new Pose2d(3.78, 3.05, Rotation2d.fromDegrees(150))));

    ScoringPosesRed = Util.transformNestedMap(ScoringPosesBlue, (p2d) -> {
      return new Pose2d(mirrorTranslationXY(p2d.getTranslation()),
          p2d.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    });
    RobotState = Map.of(
        Position.L1, Map.of(
            RobotSide.Front, new CoralScoringPosition(0., 1.75, 1.1 - wristOffset),
            RobotSide.Back, new CoralScoringPosition(0, -1., -8. - wristOffset)),
        Position.L2, Map.of(
            RobotSide.Front, new CoralScoringPosition(14., 1.75, 1.56 - wristOffset),
            RobotSide.Back, new CoralScoringPosition(3., -1., -8.3 - wristOffset)),
        Position.L3, Map.of(
            RobotSide.Front, new CoralScoringPosition(33., 1.75, 1.1 - wristOffset),
            RobotSide.Back, new CoralScoringPosition(25., -1., -8.3 - wristOffset)),
        Position.L4, Map.of(
            RobotSide.Front, new CoralScoringPosition(67., 0.7, 1.6 - wristOffset),
            RobotSide.Back, new CoralScoringPosition(64, -1.45, -9.5 - wristOffset)),
        Position.Intake, Map.of(
            RobotSide.Front, new CoralScoringPosition(6., 1., -0.9 - wristOffset),
            RobotSide.Back, new CoralScoringPosition(0, 0, 0.3 - wristOffset)));
    L2Algae = new CoralScoringPosition(17., -3., 1. - wristOffset);
    L3Algae = new CoralScoringPosition(38., -3., 1. - wristOffset);
    Net = new CoralScoringPosition(67., 0., 6.5 - wristOffset);
    ClimbUp = new CoralScoringPosition(27, 0, 0 - wristOffset);
    ClimbDown = new CoralScoringPosition(2, 0, 0 - wristOffset);
    ground = new CoralScoringPosition(5., -6.3, -.5 - wristOffset);
    Processor = new CoralScoringPosition(0, -5., 1. - wristOffset);
  }

}
