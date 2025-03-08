
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HumanControls.OperatorPanel2025;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GameInfo;
import frc.robot.util.PoseUtil;
import frc.robot.util.GameInfo.ReefSide;
import frc.team696.lib.Logging.BackupLogger;

/**
 * PIDToPosition but it takes goes to the nearest scoring pose defined in GameInfo.
 * @see PIDToPosition
 * @see GameInfo
 */
public class PIDtoNearest extends Command {
  private ProfiledPIDController xController, yController, omegaController;
  private Pose2d goalPose;
  private boolean ignoreLR;

  private double calculateWithTolerance(ProfiledPIDController controller, double measurement, double goal){
    double tolerance=controller.getPositionTolerance();
    double error=goal-measurement;
    return Math.abs(error)<tolerance?0:controller.calculate(measurement, goal);
  }

  public PIDtoNearest() {
    this(false);
  }
  /**
   * Creates a new PIDToNearest command
   * @param ignoreLR If enabled, the robot will go to the nearest scoring pose regardless if the operator panel selected left or right. 
   */
  public PIDtoNearest(boolean ignoreLR){
    addRequirements(Swerve.get());
    xController=new ProfiledPIDController(8, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.4));
    yController=new ProfiledPIDController(8, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.4));
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    
    omegaController=new ProfiledPIDController(6 , 0, 0, new TrapezoidProfile.Constraints(3, 1.2));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setTolerance(0.08);
    this.ignoreLR=ignoreLR;
  }

  public Pose2d getGoalPose(){
    Translation2d goalTranslation;
    if(ignoreLR)
      goalTranslation=GameInfo.findClosestTranslation(Swerve.get().getState().Pose);
    else
      goalTranslation=GameInfo.findClosestTranslation(Swerve.get().getState().Pose, OperatorPanel2025.leftOrRight.getAsBoolean()?ReefSide.Left:ReefSide.Right);
    Rotation2d goalRot=Swerve.get().angleTo(goalTranslation, GameInfo.blueReef);
    goalRot=Rotation2d.fromDegrees(((int)((goalRot.getDegrees() + (30*Math.signum(goalRot.getDegrees()))) / 60)) * 60.);
    return new Pose2d(goalTranslation, goalRot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose=getGoalPose();
    Pose2d currPose=Swerve.get().getState().Pose;
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    omegaController.reset(currPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    BackupLogger.addToQueue("wantogo", goalPose);

    Pose2d currPose=Swerve.get().getState().Pose;
    BackupLogger.addToQueue("Error X", goalPose.getX()-currPose.getX());
    BackupLogger.addToQueue("Error Y", goalPose.getY()-currPose.getY());
    Swerve.get().Drive(new ChassisSpeeds(
        xController.calculate(currPose.getX(),goalPose.getX()),
        yController.calculate(currPose.getY(),goalPose.getY()),
        omegaController.calculate(currPose.getRotation().getRadians(),goalPose.getRotation().getRadians())
      ),true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("There!");
    Swerve.get().Drive(new ChassisSpeeds(0,0,0));
  }
  public boolean atGoalPose(Pose2d goal, Pose2d curr){
    //System.out.println("no not finished yet "+Math.abs(goal.getX()-curr.getX())+" "+(Math.abs(goal.getY()-curr.getY())+" "+Math.abs(goal.getRotation().getDegrees()-curr.getRotation().getDegrees())));

    return 
      (Math.abs(goal.getX()-curr.getX())<=0.02)&&
      (Math.abs(goal.getY()-curr.getY())<=0.02)&&
      (Math.abs(goal.getRotation().minus(curr.getRotation()).getDegrees()))<4;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoalPose(goalPose, Swerve.get().getState().Pose);    
  }    
}
