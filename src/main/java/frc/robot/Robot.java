// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    BackupLogger.addToQueue("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
    BackupLogger.addToQueue("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
    BackupLogger.addToQueue("BuildConstants/Branch", BuildConstants.GIT_BRANCH);
    BackupLogger.addToQueue("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
    BackupLogger.addToQueue("BuildConstants/GitDate", BuildConstants.GIT_DATE);
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
    SmartDashboard.putData("sim/pathfindToMiddle", new PrintCommand("s").andThen(AutoBuilder.pathfindToPose(new Pose2d(7,3,Rotation2d.fromDegrees(12)),new PathConstraints(1, 1, Math.PI,Math.PI))));

  }
  public void putSwerveSysIDCalibrationButtons(){
    SmartDashboard.putData("CTRESwerveCalibration/DynamicForward",CommandSwerveDrivetrain.get().sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/DynamicReverse",CommandSwerveDrivetrain.get().sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticForward",CommandSwerveDrivetrain.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticReverse",CommandSwerveDrivetrain.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }
  private final SendableChooser<Command> autoChooser;
  public Robot() {
    
    SmartDashboard.putData(CommandScheduler.getInstance());

    configureDriverStationBinds();

    CommandSwerveDrivetrain.get().registerTelemetry(m_SwerveTelemetry::telemeterize);

    // Log Build information
    logBuildInfo();

    // TODO: Restore TeleopSwerve
    CommandSwerveDrivetrain.get().setDefaultCommand(CommandSwerveDrivetrain.get().applyRequest(
      ()->CommandSwerveDrivetrain.fcDriveReq.withVelocityX(
        applyDeadband(HumanControls.DriverStation.leftJoyY.getAsDouble(), 0.07)*MaxSpeed)
        .withVelocityY(applyDeadband(HumanControls.DriverStation.leftJoyX.getAsDouble(), 0.07)*MaxSpeed)
        .withRotationalRate(applyDeadband(HumanControls.DriverStation.rightJoyX.getAsDouble(), 0.07)*MaxRotationalRate)));
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
  
    NamedCommands.registerCommand("PIDtoNearest", new PIDtoNearest(true));

    // TODO: decide whether or not this will be a temporary fix
    autoChooser=AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup Commands for PathPlanner
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDriverStationBinds(){
    HumanControls.DriverStation.resetGyro.onTrue(Commands.runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
    HumanControls.OperatorPanel2024.Ground.whileTrue(new PIDtoNearest(true));
  }
  @Override
  public void robotPeriodic() {
    long start=RobotController.getTime();
    CommandScheduler.getInstance().run();
    long elapsed=RobotController.getTime()-start;
    BackupLogger.addToQueue("SchedulerTimeMS", elapsed);
    // TODO: implement common library style vision controls 
    updateVision();
    BackupLogger.logSystemInformation();

    /*int i = 0;
    for (SwerveModule<TalonFX, TalonFX, CANcoder> mod : CommandSwerveDrivetrain.get().getModules()) {
      SmartDashboard.putNumber("Encoder" + ++i, mod.getEncoder().getPosition().getValueAsDouble());
    }*/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

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
      if(measurement.tagCount>0&&measurement.rawFiducials[0].ambiguity<0.5&&measurement.rawFiducials[0].distToCamera<5){
        // Experimental
        double trustMetric=(Math.pow(measurement.rawFiducials[0].distToCamera,2)*measurement.rawFiducials[0].ambiguity/35);
        CommandSwerveDrivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(trustMetric,trustMetric,trustMetric));
        CommandSwerveDrivetrain.get().addVisionMeasurement(measurement.pose, Utils.fpgaToCurrentTime(measurement.timestampSeconds));
      }
    }
  }

  @Override
  public void teleopPeriodic() {

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
