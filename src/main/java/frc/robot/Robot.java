// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auto.NamedCommand;
import frc.robot.commands.PIDtoNearest;
import frc.robot.commands.PIDtoPosition;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.GameInfo;
import frc.robot.util.PoseUtil;
import frc.team696.lib.Camera.LimelightHelpers;
import frc.team696.lib.Logging.BackupLogger;
import frc.team696.lib.Swerve.Commands.TeleopSwerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public double MaxSpeed=TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private SwerveTelemetry m_SwerveTelemetry=new SwerveTelemetry(MaxSpeed);
  public void logBuildInfo(){
  }
  public double applyDeadband(double x, double deadband){
    return Math.abs(x)<deadband?0:x;
  }
  public void putCommandButtons(){
    SmartDashboard.putData("sim/pidToNearest", new PIDtoNearest());


  }
  private final SendableChooser<Command> autoChooser;
  public Robot() {
    
    SmartDashboard.putData(CommandScheduler.getInstance());

    configureDriverStationBinds();

    CommandSwerveDrivetrain.get().registerTelemetry(m_SwerveTelemetry::telemeterize);

    //Auto.Initialize(CommandSwerveDrivetrain.get(), false, new NamedCommand("hi", new WaitCommand(3)));

    // TODO: Restore TeleopSwerve
    CommandSwerveDrivetrain.get().setDefaultCommand(CommandSwerveDrivetrain.get().applyRequest(
      ()->CommandSwerveDrivetrain.fcDriveReq.withVelocityX(
        applyDeadband(HumanControls.DriverStation.leftJoyX.getAsDouble(), 0.07)*MaxSpeed)
        .withVelocityY(applyDeadband(HumanControls.DriverStation.leftJoyY.getAsDouble(), 0.07)*MaxSpeed)
        .withRotationalRate(applyDeadband(HumanControls.DriverStation.rightJoyX.getAsDouble(), 0.07)*9)));
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
    
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
    //SmartDashboard.putData("test/pathfindtopose",Auto.PathFind(new Pose2d(5,5,Rotation2d.fromDegrees(0))));

    //Swerve.get().setDefaultCommand(TeleopSwerve.New());
    autoChooser=AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverStationBinds(){
    //TeleopSwerve.config(Swerve.get(), HumanControls.DriverStation.leftJoyX, HumanControls.DriverStation.leftJoyY, HumanControls.DriverStation.rightJoyX, null, 0.07);  
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
    System.out.println("autonomous exit");
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
