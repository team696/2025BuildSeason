// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.team696.lib.Logging.BackupLogger;

public class ClimberIntake extends SubsystemBase {
  private static ClimberIntake m_ClimberIntake = null;

  public static ClimberIntake get() {
    if (m_ClimberIntake == null)
      m_ClimberIntake = new ClimberIntake();
    return m_ClimberIntake;
  }

  TalonFX master = new TalonFX(BotConstants.ClimberIntake.masterID, BotConstants.rioBus),
      slave = new TalonFX(BotConstants.ClimberIntake.slaveID, BotConstants.rioBus);

  StatusSignal<AngularVelocity> masterVelocitySignal, slaveVelocitySignal;
  StatusSignal<Angle> masterPositionSignal, slavePositionSignal;
  StatusSignal<Voltage> masterVoltageSignal, slaveVoltageSignal;
  StatusSignal<Current> masterCurrentSignal, slaveCurrentSignal;

  MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  VoltageOut voltageRequest = new VoltageOut(0);

  public Command Position(double position) {
    return this.startEnd(() -> master.setControl(positionRequest.withPosition(position)), () -> master.set(0));
  }

  public Command Position(DoubleSupplier position) {
    return this.startEnd(() -> master.setControl(positionRequest.withPosition(position.getAsDouble())),
        () -> master.set(0));
  }

  public void resetPosition(double newPosition) {
    master.setPosition(newPosition);
    slave.setPosition(newPosition);
  }

  /**
   * 
   * @param speed [-1, 1]
   * @return a command which spins the mechanism at the speed <i> speed </i>
   */
  public Command spin(double speed) {
    return this.startEnd(() -> master.set(speed), () -> master.set(0));
  }

  /** Creates a new ClimberIntake. */
  private ClimberIntake() {
    master.getConfigurator().apply(BotConstants.ClimberIntake.cfg);
    slave.getConfigurator().apply(BotConstants.ClimberIntake.cfg);
    slave.setControl(new Follower(master.getDeviceID(), true));
    masterVelocitySignal = master.getVelocity();
    masterPositionSignal = master.getPosition();
    masterVoltageSignal = master.getMotorVoltage();
    masterCurrentSignal = master.getStatorCurrent();
    slaveVelocitySignal = slave.getVelocity();
    slavePositionSignal = slave.getPosition();
    slaveVoltageSignal = slave.getMotorVoltage();
    slaveCurrentSignal = slave.getStatorCurrent();

    SmartDashboard.putData("Spin Pos", spin(0.1));
    SmartDashboard.putData("Spin Neg", spin(-0.1));
  }

  @Override
  public void periodic() {
    // log motor values to NT
    BackupLogger.addToQueue("ClimberIntake/masterVelocityRps",
        masterVelocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("ClimberIntake/masterPositionRot", masterPositionSignal.refresh().getValue().in(Rotations));
    BackupLogger.addToQueue("ClimberIntake/masterVoltageVolts", masterVoltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("ClimberIntake/masterCurrentAmps", masterCurrentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("ClimberIntake/slaveVelocityRps",
        slaveVelocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("ClimberIntake/slavePositionRot", slavePositionSignal.refresh().getValue().in(Rotations));
    BackupLogger.addToQueue("ClimberIntake/slaveVoltageVolts", slaveVoltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("ClimberIntake/slaveCurrentAmps", slaveCurrentSignal.refresh().getValue().in(Amps));

  }
}
