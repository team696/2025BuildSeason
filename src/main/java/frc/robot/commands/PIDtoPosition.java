
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
 * A Command which can move a swerve drive robot to a specific position on the field
 * Internally, this is simply three PID controllers with trapezoid profiles, (omegaController is continuous), thus there is <strong> no mechanism to avoid field obstacles </strong>
 * Use this to align to a position with high accuacy when the robot is <i>already near that position<i>
 * @see ChassisSpeeds
 */
public class PIDtoPosition extends Command {
  private ProfiledPIDController xController, yController, omegaController;
  private Pose2d goalPose;

  public PIDtoPosition(Pose2d goalPose) {
    System.out.println("Driving to "+goalPose.getX()+","+goalPose.getY());
    

    addRequirements(Swerve.get());
    xController=new ProfiledPIDController(/*1.7*/3, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.4));
    yController=new ProfiledPIDController(/*1.7*/3, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.4));
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    
    omegaController=new ProfiledPIDController(2 , /*1*/0, /*0.3*/0, new TrapezoidProfile.Constraints(1.6, 0.6));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setTolerance(0.08);

    this.goalPose=goalPose;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currPose=Swerve.get().getState().Pose;
    xController.reset(currPose.getX());
    yController.reset(currPose.getY());
    omegaController.reset(currPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //BackupLogger.addToQueue("wantogo", goalPose);

    Pose2d currPose=Swerve.get().getState().Pose;
    /*BackupLogger.addToQueue("Error X", goalPose.getX()-currPose.getX());
    BackupLogger.addToQueue("Error Y", goalPose.getY()-currPose.getY());*/
    Swerve.get().Drive(new ChassisSpeeds(
        xController.calculate(currPose.getX(),goalPose.getX()),
        yController.calculate(currPose.getY(),goalPose.getY()),
        omegaController.calculate(currPose.getRotation().getRadians(),goalPose.getRotation().getRadians())
      ),true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("There!");
    Swerve.get().Drive(new ChassisSpeeds(0,0,0));
  }
  public boolean atGoalPose(Pose2d goal, Pose2d curr){
    return 
      (Math.abs(goal.getX()-curr.getX())<0.03)&&
      (Math.abs(goal.getY()-curr.getY())<0.03)&&
      (Math.abs(goal.getRotation().minus(curr.getRotation()).getDegrees()))<4;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoalPose(goalPose, Swerve.get().getState().Pose);    
  }
}
