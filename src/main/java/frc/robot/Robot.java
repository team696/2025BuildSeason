// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.MoveSuperStructure;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GameInfo;
import frc.team696.lib.Camera.LimelightHelpers;
import frc.team696.lib.Logging.BackupLogger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxRotationalRate = RotationsPerSecond.of(10).in(RadiansPerSecond);
  private SwerveTelemetry m_SwerveTelemetry = new SwerveTelemetry(MaxSpeed);


  private void logBuildInfo() {
    BackupLogger.addToQueue("BuildConstants/ProjectName", BuildConstants.MAVEN_NAME);
    BackupLogger.addToQueue("BuildConstants/BuildDate", BuildConstants.BUILD_DATE);
    BackupLogger.addToQueue("BuildConstants/Branch", BuildConstants.GIT_BRANCH);
    BackupLogger.addToQueue("BuildConstants/GitSHA", BuildConstants.GIT_SHA);
    BackupLogger.addToQueue("BuildConstants/GitDate", BuildConstants.GIT_DATE);
    switch (BuildConstants.DIRTY) {
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

  public double applyDeadband(double x, double deadband) {
    return Math.abs(x) < deadband ? 0 : x;
  }


  public void putSwerveSysIDCalibrationButtons() {
    SmartDashboard.putData("CTRESwerveCalibrationc/DynamicForward",
        Swerve.get().sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/DynamicReverse",
        Swerve.get().sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticForward",
        Swerve.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticReverse",
        Swerve.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  private final SendableChooser<Command> autoChooser;

  public Robot() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(Elevator.get());
    SmartDashboard.putData(Arm.get());
    SmartDashboard.putData(EndEffector.get());
    SmartDashboard.putData(Wrist.get());

    Swerve.get().registerTelemetry(m_SwerveTelemetry::telemeterize);

    DriverStation.silenceJoystickConnectionWarning(true);
    configureDriverStationBinds();

    // Log Build information
    logBuildInfo();

    Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
        () -> Swerve.fcDriveReq.withVelocityX(
            Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.08),2) * Math.signum(HumanControls.DriverPanel.leftJoyY.getAsDouble()) * MaxSpeed)
            .withVelocityY(Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.08),2) * Math.signum(HumanControls.DriverPanel.leftJoyX.getAsDouble()) * MaxSpeed)
            .withRotationalRate(
              Math.pow(applyDeadband(HumanControls.DriverPanel.rightJoyX.getAsDouble(), 0.08), 2) * Math.signum(HumanControls.DriverPanel.rightJoyX.getAsDouble()) * MaxRotationalRate)));

    SmartDashboard.putData("Face Hex", Swerve.get().applyRequest(
      () -> Swerve.fcDriveReq.withVelocityX(
          applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.1) * MaxSpeed)
          .withVelocityY(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.1) * MaxSpeed)
          .withRotationalRate(
              (Swerve.get().getPose().getRotation().minus(Swerve.get().FaceHexFace())).getDegrees() / -120 * MaxRotationalRate)));

    SmartDashboard.putData("Face Net", Swerve.get().applyRequest(
      () -> Swerve.fcDriveReq.withVelocityX(
          applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.1) * MaxSpeed)
          .withVelocityY(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.1) * MaxSpeed)
          .withRotationalRate(
            (Swerve.get().getPose().getRotation().minus(Swerve.get().FaceNet())).getDegrees() / 30 * MaxRotationalRate)));

    SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> Swerve.get().seedFieldCentric()));

    NamedCommands.registerCommand("hello", new InstantCommand(
        () -> SmartDashboard.putBoolean("auto", true)));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup Commands for PathPlanner
    PathfindingCommand.warmupCommand().schedule();

    Elevator.get().setDefaultCommand(Elevator.get().positionCommand(0));
    Wrist.get().setDefaultCommand(Wrist.get().Position(0));
    Arm.get().setDefaultCommand(Arm.get().Position(()->0));
    EndEffector.get().setDefaultCommand(EndEffector.get().spin(()->EndEffector.get().idlePower));
  }

  private void configureDriverStationBinds() {
    HumanControls.OperatorPanel2025.gyro.onTrue(new InstantCommand(()->Swerve.get().seedFieldCentric()));
    HumanControls.OperatorPanel2025.releaseCoral.onTrue(new InstantCommand(()-> {EndEffector.get().idlePower = -0.6;}));

    HumanControls.OperatorPanel2025.L1.whileTrue(
      new ConditionalCommand(
        new MoveSuperStructure(GameInfo.ground, -0.8, false, -.8), 
        new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L1).get(GameInfo.RobotSide.Back), -0.4), 
        HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean
      ));
    
    HumanControls.OperatorPanel2025.L2.whileTrue(
      new ConditionalCommand(
        new MoveSuperStructure(GameInfo.L2Algae, -0.8, false, -0.8), 
        new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L2).get(GameInfo.RobotSide.Back), -0.6), 
        HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean
      ));

    HumanControls.OperatorPanel2025.L3.whileTrue(
      new ConditionalCommand(
        new MoveSuperStructure(GameInfo.L3Algae, -0.8, false, -1.), 
        new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L3).get(GameInfo.RobotSide.Back), -0.6),
        HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean
      ));

    HumanControls.OperatorPanel2025.L4.whileTrue(new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Back), -0.6));
    HumanControls.OperatorPanel2025.Climb2.whileTrue(new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), 0.5, false, 0.2));
    HumanControls.OperatorPanel2025.Barge.whileTrue(new MoveSuperStructure(GameInfo.Net, 1.));
    HumanControls.OperatorPanel2025.Climb1.whileTrue(new MoveSuperStructure(GameInfo.ClimbUp, 0));
    HumanControls.OperatorPanel2025.Processor.whileTrue(new MoveSuperStructure(GameInfo.Processor, 0.6));
    //HumanControls.OperatorPanel2025.pickupAlgae.whileTrue(new MoveSuperStructure(GameInfo.ground, -0.8, false, -0.8));
  }

  @Override
  public void robotPeriodic() {
    long start = RobotController.getTime();
    CommandScheduler.getInstance().run();
    long elapsed = RobotController.getTime() - start;
    BackupLogger.addToQueue("SchedulerTimeMS", elapsed); // Scheduler Time in Microseconds, anything over 20,000 should
                                                         // trigger the watchdog
    updateVision("limelight-corner");
    BackupLogger.logSystemInformation();

    /*
     * int i = 0;
     * for (SwerveModule<TalonFX, TalonFX, CANcoder> mod :
     * CommandSwerveDrivetrain.get().getModules()) {
     * SmartDashboard.putNumber("Encoder" + ++i,
     * mod.getEncoder().getPosition().getValueAsDouble());
     * }
     */
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void updateVision(String limelightName) {
    double heading = Swerve.get().getState().Pose.getRotation().getDegrees();

    LimelightHelpers.SetRobotOrientation(limelightName, heading, 0.0, 0.0, 0.0, 0.0, 0.0);
    var measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if (measurement != null) {
      if (measurement.tagCount > 0 && measurement.rawFiducials[0].ambiguity < 0.5
          && measurement.rawFiducials[0].distToCamera < 5) {
        // Experimental
        double trustMetric = (Math.pow(measurement.rawFiducials[0].distToCamera, 2)
            * measurement.rawFiducials[0].ambiguity / 35);
        Swerve.get().setVisionMeasurementStdDevs(VecBuilder.fill(trustMetric, trustMetric, trustMetric));
        Swerve.get().addVisionMeasurement(measurement.pose, Utils.fpgaToCurrentTime(measurement.timestampSeconds));
      }
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
