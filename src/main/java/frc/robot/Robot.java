// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoMoveSuperStructure;
import frc.robot.commands.MoveSuperStructure;
import frc.robot.commands.PIDtoNearest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.util.GameInfo;
import frc.robot.util.GameInfo.ReefSide;
import frc.team696.lib.Logging.BackupLogger;
import frc.team696.lib.Swerve.SwerveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private double MaxSpeed = SwerveConstants.MAX_VELOCITY.in(MetersPerSecond);// aTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	private double MaxRotationalRate = RotationsPerSecond.of(/* 10 */7).in(RadiansPerSecond);
	private SwerveTelemetry m_SwerveTelemetry = new SwerveTelemetry(MaxSpeed);

	private ProfiledPIDController thetaController = new ProfiledPIDController(1. / 200., 0, 0.,
			new TrapezoidProfile.Constraints(360, 480));

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

	private Command OldTimesAuto() {
		SequentialCommandGroup commandGroup = new SequentialCommandGroup();
		PathConstraints constraints = new PathConstraints(2, 1, 15, 10);
		for (var index : GameInfo.getScoringPoses().entrySet()) {
			Pose2d goalPoseLeft = index.getValue().get(GameInfo.ReefSide.Left);
			Pose2d goalPoseRight = index.getValue().get(GameInfo.ReefSide.Right);

			commandGroup.addCommands(
					AutoBuilder.pathfindToPose(goalPoseLeft, constraints),
					new AutoMoveSuperStructure(
							GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Back), -0.6, 0.0).asProxy(),
					AutoBuilder.pathfindToPose(new Pose2d(1.397, 7.55, Rotation2d.fromDegrees(35)), constraints),
					new AutoMoveSuperStructure(
							GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), .6, .1, true).asProxy(),
					AutoBuilder.pathfindToPose(goalPoseRight, constraints),
					new AutoMoveSuperStructure(
							GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Back), -0.6, 0.0).asProxy(),
					AutoBuilder.pathfindToPose(new Pose2d(1.397, 7.55, Rotation2d.fromDegrees(35)), constraints),
					new AutoMoveSuperStructure(
							GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), .6, .1, true).asProxy());
		}
		return commandGroup;
	}

	public Robot() {
		// TODO: strip out groundcoral system
		thetaController.enableContinuousInput(-180, 180);
		Arm.get();
		Elevator.get();
		EndEffector.get();
		Swerve.get();
		Wrist.get();
		DriverStation.silenceJoystickConnectionWarning(true);
		logBuildInfo();
		SignalLogger.start();
		configureDriverStationBinds();
		Swerve.get().setDefaultCommand(Swerve.get().applyRequest(
				() -> Swerve.fcDriveReq.withVelocityX(
						Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.09), 2)
								* Math.signum(HumanControls.DriverPanel.leftJoyY.getAsDouble()) * MaxSpeed)
						.withVelocityY(Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.09), 2)
								* Math.signum(HumanControls.DriverPanel.leftJoyX.getAsDouble()) * MaxSpeed)
						.withRotationalRate(
								Math.pow(applyDeadband(HumanControls.DriverPanel.rightJoyX.getAsDouble(), 0.09), 2)
										* Math.signum(HumanControls.DriverPanel.rightJoyX.getAsDouble()) * MaxRotationalRate)));

		/*
		 * HumanControls.DriverPanel.OtherButton.whileTrue(Swerve.get().applyRequest(
		 * () -> Swerve.fcDriveReq.withVelocityX(
		 * Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(),
		 * 0.09), 2)
		 * Math.signum(HumanControls.DriverPanel.leftJoyY.getAsDouble()) * MaxSpeed)
		 * .withVelocityY(Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyX.
		 * getAsDouble(), 0.09), 2)
		 * Math.signum(HumanControls.DriverPanel.leftJoyX.getAsDouble()) * MaxSpeed)
		 * 
		 * .withRotationalRate(
		 * (thetaController.calculate(Swerve.get().getPose().getRotation().getDegrees(),
		 * Swerve.get().goalRotation.get().getDegrees()))
		 * MaxRotationalRate))
		 * .alongWith(
		 * Commands.startEnd(() ->
		 * thetaController.reset(Swerve.get().getPose().getRotation().getDegrees()), ()
		 * -> {
		 * })));
		 */
		HumanControls.DriverPanel.OtherButton.whileTrue(Swerve.get().applyRequest(
				() -> Swerve.fcDriveReq.withVelocityX(
						Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyY.getAsDouble(), 0.09), 2)
								* Math.signum(HumanControls.DriverPanel.leftJoyY.getAsDouble()) * MaxSpeed)
						.withVelocityY(Math.pow(applyDeadband(HumanControls.DriverPanel.leftJoyX.getAsDouble(), 0.09), 2)
								* Math.signum(HumanControls.DriverPanel.leftJoyX.getAsDouble()) * MaxSpeed)

						.withRotationalRate(
								(thetaController.calculate(Swerve.get().getPose().getRotation().getDegrees(),
										Swerve.get().getGoalRotation().getDegrees()))
										* MaxRotationalRate))
				.alongWith(
						Commands.startEnd(() -> thetaController.reset(Swerve.get().getPose().getRotation().getDegrees()), () -> {
						})));
		NamedCommands.registerCommand("L4", new AutoMoveSuperStructure(
				GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Back), -0.6, 0.0).asProxy());
		NamedCommands.registerCommand("L2", new AutoMoveSuperStructure(
				GameInfo.RobotState.get(GameInfo.Position.L2).get(GameInfo.RobotSide.Back), -0.6, 0.0).asProxy());

		NamedCommands.registerCommand("Intake", new AutoMoveSuperStructure(
				GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), .6, .1, true).asProxy());
		NamedCommands.registerCommand("AfterIntake",
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Back), 0.15,
						false, 0.1).asProxy());
		NamedCommands.registerCommand("Barge", new AutoMoveSuperStructure(GameInfo.Net, 1., 0).asProxy());
		NamedCommands.registerCommand("L3Algae", new MoveSuperStructure(GameInfo.L3Algae, -0.8, false, -1.)
				.until(() -> EndEffector.get().isStalling()).asProxy());
		NamedCommands.registerCommand("L2Algae",
				new MoveSuperStructure(GameInfo.L2AlgaeLow, -0.8, false, -1.).withTimeout(2).asProxy());
		NamedCommands.registerCommand("AlgaeUp", new MoveSuperStructure(GameInfo.algaeUp, -0.3).asProxy());
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		SmartDashboard.putData("L1",
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L1).get(GameInfo.RobotSide.Front), 0));
		SmartDashboard.putData("L2",
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L2).get(GameInfo.RobotSide.Front), 0));
		SmartDashboard.putData("L3",
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L3).get(GameInfo.RobotSide.Front), 0));
		SmartDashboard.putData("L4",
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Front), 0));
		SmartDashboard.putData("Old times Auto", OldTimesAuto());

		// Warmup Commands for PathPlanner
		// PathfindingCommand.warmupCommand().schedule();

		Elevator.get().setDefaultCommand(Elevator.get().positionCommand(() -> {
			return 0;
		}));
		Wrist.get().setDefaultCommand(Wrist.get().Position(-0.3));
		Arm.get().setDefaultCommand(Arm.get().Position(() -> .7));
		EndEffector.get().setDefaultCommand(EndEffector.get().spin(() -> EndEffector.get().idlePower));
	}

	private void configureDriverStationBinds() {
		HumanControls.DriverPanel.resetGyro.whileTrue(new PIDtoNearest(false));
		HumanControls.OperatorPanel2025.gyro.onTrue(new InstantCommand(() -> Swerve.get().seedFieldCentric()));
		HumanControls.OperatorPanel2025.releaseCoral.whileTrue(
				new InstantCommand(() -> {
					EndEffector.get().idlePower = -0.6;
				}));

		HumanControls.OperatorPanel2025.L1.whileTrue(
				new ConditionalCommand(
						new MoveSuperStructure(GameInfo.ground, -0.8, false, -.8),
						new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L1).get(GameInfo.RobotSide.Back), -0.15),
						HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean)
						.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceHexFace, Swerve.get()::FaceSource)));

		HumanControls.OperatorPanel2025.L2.whileTrue(
				new ConditionalCommand(
						new MoveSuperStructure(GameInfo.L2Algae, -0.8, false, -0.8),
						new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L2).get(GameInfo.RobotSide.Back), -0.6),
						HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean)
						.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceHexFace, Swerve.get()::FaceSource)));

		HumanControls.OperatorPanel2025.L3.whileTrue(
				new ConditionalCommand(
						new MoveSuperStructure(GameInfo.L3Algae, -0.8, false, -1.),
						new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L3).get(GameInfo.RobotSide.Back), -0.6),
						HumanControls.OperatorPanel2025.pickupAlgae::getAsBoolean)
						.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceHexFace, Swerve.get()::FaceSource)));

		HumanControls.OperatorPanel2025.L4.whileTrue(
				new MoveSuperStructure(GameInfo.RobotState.get(GameInfo.Position.L4).get(GameInfo.RobotSide.Back), -0.6)
						.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceHexFace, Swerve.get()::FaceSource))); //

		HumanControls.OperatorPanel2025.Barge.whileTrue(
				new MoveSuperStructure(GameInfo.Net, 1.)
						.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceNet, Swerve.get()::FaceSource)));

		HumanControls.OperatorPanel2025.SouceCoral.whileTrue((new MoveSuperStructure(
				GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), 0.6, false, 0.1))
				.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceSource, Swerve.get()::FaceHexFace)));
		HumanControls.OperatorPanel2025.SouceCoral.onFalse(new MoveSuperStructure(
				GameInfo.RobotState.get(GameInfo.Position.Intake).get(GameInfo.RobotSide.Front), 0.15, false, 0.1));
		HumanControls.OperatorPanel2025.Climb1.whileTrue(new PIDtoNearest(false));
		HumanControls.OperatorPanel2025.Processor.whileTrue(new MoveSuperStructure(GameInfo.Processor, 0.6)
				.deadlineFor(Swerve.get().setGoalRotation(Swerve.get()::FaceProcessor, Swerve.get()::FaceSource)));
		/*
		 * HumanControls.OperatorPanel2025.releaseCoral.and(HumanControls.
		 * OperatorPanel2025.pickupAlgae)
		 * .whileTrue(new PrintCommand("trebuchet"));
		 */

	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		m_SwerveTelemetry.telemeterize(Swerve.get().getState());
		BackupLogger.logSystemInformation();
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
