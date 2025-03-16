// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.robot.util.GameInfo;

public class Arm extends SubsystemBase {
  private static Arm arm = null;

  public static synchronized Arm get() {
    if (arm == null) {
      arm = new Arm();
    }
    return arm;
  }

  TalonFX master = new TalonFX(BotConstants.Arm.masterID, BotConstants.rioBus);

  MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  VoltageOut voltageRequest = new VoltageOut(0);

  boolean slowMode = false;

  // to
  // use
  // this
  // because
  // CTR SHITTYTRONICS IS
  // GATEKEEPING BASIC
  // FUNCTIONALITY
  //
  ProfiledPIDController slowPidController = new ProfiledPIDController(1., 0, 0,
      new TrapezoidProfile.Constraints(60., 35.));

  /** Creates a new Arm. */
  private Arm() {
    master.getConfigurator().apply(BotConstants.Arm.cfg);

    zeroArm();

    slowPidController.reset(0);
  }

  public void stop() {
    master.stopMotor();
  }

  public void resetArmPosition(double newPosition) {
    master.setPosition(newPosition);
  }

  public void zeroArm() {
    resetArmPosition(0);
  }

  public void goToPosition(double position) {
    if (!slowMode) {
      master.setControl(positionRequest.withPosition(position));
    } else {
      master.setControl(voltageRequest.withOutput(slowPidController.calculate(getPosition(), position)));
    }
  }

  public void goToPosition(GameInfo.CoralScoringPosition position) {
    goToPosition(position.armRot.in(Rotations));
  }

  public void goToPosition(DoubleSupplier position) {
    goToPosition(position.getAsDouble());
  }

  public Command Position(DoubleSupplier position) {
    return this.runEnd(() -> goToPosition(position),
        () -> stop());
  }

  public Command Position(GameInfo.CoralScoringPosition position) {
    return this.startEnd(() -> goToPosition(position),
        () -> stop());
  }

  public double getPosition() {
    return master.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    slowPidController.calculate(getPosition());

  }
}
