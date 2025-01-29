// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
  public Robot() {
    
    SmartDashboard.putData(CommandScheduler.getInstance());
    configureDriverStationBinds();
    
    CommandSwerveDrivetrain.get().registerTelemetry(m_SwerveTelemetry::telemeterize);
    CommandSwerveDrivetrain.get().setDefaultCommand(CommandSwerveDrivetrain.get().applyRequest(
      ()->CommandSwerveDrivetrain.fcDriveReq.withVelocityX(
        applyDeadband(HumanControls.DriverStation.leftJoyX.getAsDouble(), 0.07)*MaxSpeed)
        .withVelocityY(applyDeadband(HumanControls.DriverStation.leftJoyY.getAsDouble(), 0.07)*MaxSpeed)
        .withRotationalRate(applyDeadband(HumanControls.DriverStation.rightJoyX.getAsDouble(), 0.07)*3)));
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
    SmartDashboard.putData("Drive Forward (Robot Relative)",
    CommandSwerveDrivetrain.get().applyRequest(()->CommandSwerveDrivetrain.rcDriveReq.withSpeeds(new ChassisSpeeds(1, 0, 0))));
    HumanControls.DriverStation.resetGyro.onTrue(CommandSwerveDrivetrain.get().runOnce(()->CommandSwerveDrivetrain.get().seedFieldCentric()));
    //Swerve.get().setDefaultCommand(TeleopSwerve.New());
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
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    double heading=CommandSwerveDrivetrain.get().getState().Pose.getRotation().getDegrees();
    
    LimelightHelpers.SetRobotOrientation("limelight-corner",heading,0.0,0.0,0.0,0.0,0.0);
    var measurement=LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-corner");
    if(measurement!=null){
      CommandSwerveDrivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(0.001,0.001,0.001));
      CommandSwerveDrivetrain.get().addVisionMeasurement(measurement.pose, measurement.timestampSeconds);
      SmartDashboard.putNumberArray("Balzac",new double[]{measurement.pose.getX(), measurement.pose.getY()});
    }
    }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
