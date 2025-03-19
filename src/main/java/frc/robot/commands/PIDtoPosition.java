
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/***
 * A Command which can move a swerve drive robot to a specific position on the
 * field
 * Internally, this is simply three PID controllers with trapezoid profiles,
 * (omegaController is continuous), thus there is <strong> no mechanism to avoid
 * field obstacles </strong>
 * Use this to align to a position with high accuacy when the robot is
 * <i>already near that position<i>
 * 
 * @see ChassisSpeeds
 */
public class PIDtoPosition extends Command {
  private ProfiledPIDController xController, yController, omegaController;
  private Pose2d goalPose;

  public PIDtoPosition(Pose2d goalPose) {
    addRequirements(Swerve.get());
    xController = new ProfiledPIDController(5, 0.0, 0.0, new TrapezoidProfile.Constraints(2., 2.));
    yController = new ProfiledPIDController(5, 0.0, 0.0, new TrapezoidProfile.Constraints(2., 2.));
    xController.setTolerance(0.02);
    yController.setTolerance(0.02);

    omegaController = new ProfiledPIDController(5., 0, 0, new TrapezoidProfile.Constraints(180, 360));
    omegaController.enableContinuousInput(-180, 180);
    omegaController.setTolerance(1.);

    this.goalPose = goalPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currPose = Swerve.get().getPose();
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    omegaController.reset(currPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = Swerve.get().getPose();
    Swerve.get().Drive(new ChassisSpeeds(
        xController.calculate(currPose.getX(), goalPose.getX()),
        yController.calculate(currPose.getY(), goalPose.getY()),
        omegaController.calculate(currPose.getRotation().getDegrees(), goalPose.getRotation().getDegrees()) / 180
            * Math.PI),
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Swerve.get().Drive(new ChassisSpeeds(0, 0, 0));
  }

  public boolean atGoalPose(Pose2d goal, Pose2d curr) {
    return (Math.abs(goal.getX() - curr.getX()) < 0.02) &&
        (Math.abs(goal.getY() - curr.getY()) < 0.02) &&
        (Math.abs(goal.getRotation().minus(curr.getRotation()).getDegrees())) < 1.;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;// goalPose.getTranslation().getDistance(Swerve.get().getPose().getTranslation())
                 // < 1
    // || atGoalPose(goalPose, Swerve.get().getPose());
  }
}
