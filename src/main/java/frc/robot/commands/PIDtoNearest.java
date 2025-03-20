
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HumanControls;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GameInfo;
//import frc.robot.util.PoseUtil;
import frc.robot.util.GameInfo.ReefSide;
import frc.team696.lib.Logging.BackupLogger;

/**
 * PIDToPosition but it takes goes to the nearest scoring pose defined in
 * GameInfo.
 * 
 * @see PIDToPosition
 * @see GameInfo
 */
public class PIDtoNearest extends Command {
  private ProfiledPIDController xController, yController, omegaController;
  private Pose2d goalPose;
  private boolean ignoreLR;

  public static Pose2d findClosestPose(Pose2d referencePose) {
    if (referencePose == null) {
      throw new IllegalArgumentException("referencePose cannot be null");
    }

    Pose2d closestPose = null;
    double minDistance = Double.MAX_VALUE; // Initialize with a very large value

    for (var entry : GameInfo.getScoringPoses().entrySet()) {
      // Calculate the squared Euclidean distance between the translations
      for (var poseEntry : entry.getValue().entrySet()) {
        Pose2d pose = poseEntry.getValue();
        Translation2d referenceTranslation = referencePose.getTranslation();
        double distance = referenceTranslation.getDistance(pose.getTranslation());

        if (distance < minDistance) {
          minDistance = distance;
          closestPose = pose;
        }
      }
    }

    return closestPose;
  }

  public static Pose2d findClosestPose(Pose2d referencePose, ReefSide side) {
    if (referencePose == null) {
      throw new IllegalArgumentException("referencePose cannot be null");
    }

    Pose2d closestPose = null;
    double minDistance = Double.MAX_VALUE; // Initialize with a very large value

    for (var entry : GameInfo.getScoringPoses().entrySet()) {
      // Calculate the squared Euclidean distance between the translations
        Pose2d pose = entry.getValue().get(side);
        Translation2d referenceTranslation = referencePose.getTranslation();
        double distance = referenceTranslation.getDistance(pose.getTranslation());

        if (distance < minDistance) {
          minDistance = distance;
          closestPose = pose;
        }

    }

    return closestPose;
  }

  public PIDtoNearest() {
    this(false);
  }

  /**
   * Creates a new PIDToNearest command
   * 
   * @param ignoreLR If enabled, the robot will go to the nearest scoring pose
   *                 regardless if the operator panel selected left or right.
   */
  public PIDtoNearest(boolean ignoreLR) {
    addRequirements(Swerve.get());
    xController = new ProfiledPIDController(8, 0.0, 0.0, new TrapezoidProfile.Constraints(2.45, 2.2));
    yController = new ProfiledPIDController(8, 0.0, 0.0, new TrapezoidProfile.Constraints(2.45
    , 2.2));
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);

    omegaController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(5, 3.5));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setTolerance(0.08);
    this.ignoreLR = ignoreLR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currPose = Swerve.get().getState().Pose;
    goalPose = ignoreLR?findClosestPose(currPose):findClosestPose(currPose, HumanControls.OperatorPanel2025.leftOrRight.getAsBoolean()?ReefSide.Right:ReefSide.Left);
    //System.out.println("voy a ir a " + goalPose.getX() + " y " + goalPose.getY());
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    omegaController.reset(currPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BackupLogger.addToQueue("wantogo", goalPose);

    Pose2d currPose = Swerve.get().getState().Pose;
    Swerve.get().Drive(new ChassisSpeeds(
        xController.calculate(currPose.getX(), goalPose.getX()),
        yController.calculate(currPose.getY(), goalPose.getY()),
        omegaController.calculate(currPose.getRotation().getRadians(), goalPose.getRotation().getRadians())), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("There!");
    Swerve.get().Drive(new ChassisSpeeds(0, 0, 0));
  }

  public boolean atGoalPose(Pose2d goal, Pose2d curr) {
    return (Math.abs(goal.getX() - curr.getX()) <= 0.02) &&
        (Math.abs(goal.getY() - curr.getY()) <= 0.02) &&
        (Math.abs(goal.getRotation().minus(curr.getRotation()).getDegrees())) < 2;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoalPose(goalPose, Swerve.get().getState().Pose);
  }
}