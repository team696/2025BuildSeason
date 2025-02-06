// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PIDtoNearest;
import frc.robot.commands.PIDtoPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.team696.lib.Camera.LimelightHelpers;
import frc.team696.lib.Logging.BackupLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed=TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxRotationalRate=RotationsPerSecond.of(3).in(RadiansPerSecond);
  private SwerveTelemetry m_SwerveTelemetry=new SwerveTelemetry(MaxSpeed);


  private void logBuildInfo(){
    BackupLogger.addToQueue("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
    BackupLogger.addToQueue("BuildConstants/Branch", BuildConstants.GIT_BRANCH);
    BackupLogger.addToQueue("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
    BackupLogger.addToQueue("BuildConstants/GitDate", BuildConstants.GIT_DATE);
    BackupLogger.addToQueue("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
    switch(BuildConstants.DIRTY){
      case 0:
        BackupLogger.addToQueue("BuildConstants/GitDirty", "All Changes Comitted");
        break;
      case 1:
        BackupLogger.addToQueue("BuildConstants/GitDirty", "Uncomitted changes");
        break;
      case 2:
        BackupLogger.addToQueue("BuildConstants/GitDirty", "Unknown");
        break;
    }


  }
  public double applyDeadband(double x, double deadband){
    return Math.abs(x)<deadband?0:x;
  }
  public void putCommandButtons(){
    SmartDashboard.putData("sim/pidToNearest", new PIDtoNearest());
    SmartDashboard.putBoolean("pathfindingConfigured", AutoBuilder.isPathfindingConfigured());
    SmartDashboard.putData("sim/pathfindToMiddle", AutoBuilder.pathfindToPose(new Pose2d(7,3,Rotation2d.fromDegrees(12)),new PathConstraints(1, 1, Math.PI,Math.PI)));

  }
  private final SendableChooser<Command> autoChooser;
  public Robot() {
    
    SmartDashboard.putData(CommandScheduler.getInstance());

    configureDriverStationBinds();

    CommandSwerveDrivetrain.get().registerTelemetry(m_SwerveTelemetry::telemeterize);

    // Log Build information
    logBuildInfo();
    
    //Auto.Initialize(CommandSwerveDrivetrain.get(), false, new NamedCommand("hi", new WaitCommand(3)));

    // TODO: Restore TeleopSwerve
    CommandSwerveDrivetrain.get().setDefaultCommand(CommandSwerveDrivetrain.get().applyRequest(
      ()->CommandSwerveDrivetrain.fcDriveReq.withVelocityX(
        applyDeadband(HumanControls.DriverStation.leftJoyX.getAsDouble(), 0.07)*MaxSpeed)
        .withVelocityY(applyDeadband(HumanControls.DriverStation.leftJoyY.getAsDouble(), 0.07)*MaxSpeed)
        .withRotationalRate(applyDeadband(HumanControls.DriverStation.rightJoyX.getAsDouble(), 0.07)*MaxRotationalRate)));
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
    NamedCommands.registerCommand("PIDtoNearest", new PIDtoNearest());
    
    SmartDashboard.putData("Drive Forward (Robot Relative)",
    CommandSwerveDrivetrain.get().applyRequest(()->CommandSwerveDrivetrain.rcDriveReq.withSpeeds(new ChassisSpeeds(1, 0, 0))));

    HumanControls.DriverStation.resetGyro.onTrue(CommandSwerveDrivetrain.get().runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));

    HumanControls.DriverStation.test1.onTrue(
      new PIDtoPosition(new Pose2d(13.61,2.65, Rotation2d.fromDegrees(32)))
      .andThen(new PIDtoPosition(new Pose2d(13.34, 0.76, Rotation2d.fromDegrees(107))))
      .repeatedly());

    if(Robot.isSimulation()){
      HumanControls.DriverStation.test2.whileTrue(
        new PIDtoNearest()
      );
      putCommandButtons();

    }

    // TODO: decide whether or not this will be a temporary fix
    autoChooser=AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup Commands for PathPlanner
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDriverStationBinds(){
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    BackupLogger.logSystemInformation();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();//Auto.PathFind(new Pose2d(10,1,Rotation2d.fromDegrees(0)));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if(m_autonomousCommand!=null){
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void updateVision(){
    double heading=CommandSwerveDrivetrain.get().getState().Pose.getRotation().getDegrees();
    
    LimelightHelpers.SetRobotOrientation("limelight-corner",heading,0.0,0.0,0.0,0.0,0.0);
    var measurement=LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-corner");
    if(measurement!=null){
      if(measurement.tagCount>0&&measurement.rawFiducials[0].ambiguity<0.5&&measurement.rawFiducials[0].distToCamera<6){
        double trustMetric=(Math.pow(measurement.rawFiducials[0].distToCamera,2)/60);
        CommandSwerveDrivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(trustMetric,trustMetric,trustMetric));
        CommandSwerveDrivetrain.get().addVisionMeasurement(measurement.pose, Utils.fpgaToCurrentTime(measurement.timestampSeconds));
        SmartDashboard.putNumberArray("Balzac",new double[]{measurement.pose.getX(), measurement.pose.getY()});
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    // TODO: implement common library style vision controls
    updateVision();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void simulationPeriodic(){
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
