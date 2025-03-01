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
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.MoveSuperStructure;
import frc.robot.commands.PIDtoNearest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GameInfo;
import frc.robot.util.GameInfo.CoralScoringPosition;
import frc.team696.lib.Camera.LimelightHelpers;
import frc.team696.lib.Logging.BackupLogger;
import frc.robot.subsystems.ClimberIntake;
import frc.robot.subsystems.ArmAndWrist;
import frc.robot.subsystems.EndEffector;
import frc.robot.HumanControls.OperatorPanel2025;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double MaxSpeed=TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxRotationalRate=RotationsPerSecond.of(3).in(RadiansPerSecond);
  private SwerveTelemetry m_SwerveTelemetry=new SwerveTelemetry(MaxSpeed);

  private void configureBinds(){
    OperatorPanel2025.gyro.onTrue(new InstantCommand(()->Swerve.get().seedFieldCentric()));
    OperatorPanel2025.releaseCoral.onTrue(new InstantCommand());
  }

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
    //SmartDashboard.putData("sim/pidToNearest", new PIDtoNearest());
    //SmartDashboard.putBoolean("pathfindingConfigured", AutoBuilder.isPathfindingConfigured());
    SmartDashboard.putData("sim/pathfindToMiddle", new PrintCommand("s").andThen(AutoBuilder.pathfindToPose(new Pose2d(7,3,Rotation2d.fromDegrees(12)),new PathConstraints(1, 1, Math.PI,Math.PI))));
    SmartDashboard.putData("ScoreL4", Elevator.get().positionCommand(GameInfo.L4));
    SmartDashboard.putData("ScoreL3", ArmAndWrist.get().Position(GameInfo.L3).alongWith(Elevator.get().positionCommand(GameInfo.L3))) ;
    SmartDashboard.putData("ScoreL2", Elevator.get().positionCommand(GameInfo.L2));



  }
  public void putSwerveSysIDCalibrationButtons(){
    SmartDashboard.putData("CTRESwerveCalibrationc/DynamicForward",Swerve.get().sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/DynamicReverse",Swerve.get().sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticForward",Swerve.get().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("CTRESwerveCalibration/QuasistaticReverse",Swerve.get().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  private final SendableChooser<Command> autoChooser;
  public Robot() {

    // For remote layout downloading
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(Elevator.get());
    //SmartDashboard.putData(ClimberIntake.get());
    SmartDashboard.putData(ArmAndWrist.get());
    SmartDashboard.putData(EndEffector.get());


    Swerve.get().registerTelemetry(m_SwerveTelemetry::telemeterize);

    DriverStation.silenceJoystickConnectionWarning(true);
    configureDriverStationBinds();

    // Log Build information
    logBuildInfo();

    // TODO: Restore TeleopSwerve
    Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
      ()->Swerve.fcDriveReq.withVelocityX(
        applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.1)*MaxSpeed)
        .withVelocityY(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.1)*MaxSpeed)
        .withRotationalRate(applyDeadband(HumanControls.DriverPanel.rightJoyX.getAsDouble(), 0.1)*MaxRotationalRate)));
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(()->Swerve.get().seedFieldCentric()));
  
    NamedCommands.registerCommand("PIDtoNearest", new PIDtoNearest(true));
    NamedCommands.registerCommand("ScoreL4",
    Elevator.get().positionCommand(GameInfo.L4));
    NamedCommands.registerCommand("ScoreL3",ArmAndWrist.get().Position(GameInfo.L3).alongWith(Elevator.get().positionCommand(GameInfo.L3)));
    NamedCommands.registerCommand("ScoreL2",Elevator.get().positionCommand(GameInfo.L2));
    NamedCommands.registerCommand("ScoreL1",Elevator.get().positionCommand(GameInfo.L1));
    putCommandButtons();
    NamedCommands.registerCommand("hello", new InstantCommand(
      ()->
      SmartDashboard.putBoolean("auto", true)
      ));


    // TODO: decide whether or not this will be a temporary fix
    autoChooser=AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup Commands for PathPlanner
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDriverStationBinds(){
    //HumanControls.DriverPanel.resetGyro.onTrue(Commands.runOnce(()->Swerve.get().seedFieldCentric()));
    //HumanControls.OperatorPanel2025.L1.whileTrue(Elevator.get().positionCommand(GameInfo.L1));
    //HumanControls.OperatorPanel2025.L2.whileTrue(Elevator.get().positionCommand(GameInfo.L2));
    //HumanControls.OperatorPanel2025.L3.whileTrue(Elevator.get().positionCommand(GameInfo.L3));
    //HumanControls.OperatorPanel2025.L4.whileTrue(Elevator.get().positionCommand(GameInfo.L4));

    HumanControls.DriverPanel.OtherButton.whileTrue(new MoveSuperStructure(new CoralScoringPosition(0, 9., 0), 0.6));
    HumanControls.DriverPanel.resetGyro.whileTrue(new MoveSuperStructure(new CoralScoringPosition(32., -4., -3.), 0.6));
  }
  
  @Override
  public void robotPeriodic() {
    long start=RobotController.getTime();
    CommandScheduler.getInstance().run();
    long elapsed=RobotController.getTime()-start;
    BackupLogger.addToQueue("SchedulerTimeMS", elapsed); // Scheduler Time in Microseconds, anything over 20,000 should trigger the watchdog
    updateVision("limelight-corner");
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

  public void updateVision(String limelightName){
    double heading=Swerve.get().getState().Pose.getRotation().getDegrees();
    
    LimelightHelpers.SetRobotOrientation(limelightName,heading,0.0,0.0,0.0,0.0,0.0);
    var measurement=LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if(measurement!=null){
      if(measurement.tagCount>0&&measurement.rawFiducials[0].ambiguity<0.5&&measurement.rawFiducials[0].distToCamera<5){
        // Experimental
        double trustMetric=(Math.pow(measurement.rawFiducials[0].distToCamera,2)*measurement.rawFiducials[0].ambiguity/35);
        Swerve.get().setVisionMeasurementStdDevs(VecBuilder.fill(trustMetric,trustMetric,trustMetric));
        Swerve.get().addVisionMeasurement(measurement.pose, Utils.fpgaToCurrentTime(measurement.timestampSeconds));
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
