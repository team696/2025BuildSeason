// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.robot.util.GameInfo;
import frc.robot.util.TriggerNTDouble;
import frc.team696.lib.Logging.BackupLogger;

public class Arm extends SubsystemBase {
  private static Arm arm = null;

  public static synchronized Arm get() {
    if (arm == null) {
      arm = new Arm();
    }
    return arm;
  }

  TalonFX master = new TalonFX(BotConstants.Arm.masterID, BotConstants.rioBus);
  TalonFX slave = new TalonFX(BotConstants.Arm.slaveID, BotConstants.rioBus);
  StatusSignal<AngularVelocity> velocitySignal;
  StatusSignal<Angle> positionSignal;
  StatusSignal<Voltage> voltageSignal;
  StatusSignal<Current> currentSignal;

  MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  VoltageOut voltageRequest = new VoltageOut(0);

  double ntpos = 0;

  /** Creates a new Arm. */
  private Arm() {
    master.getConfigurator().apply(BotConstants.Arm.cfg);
    slave.getConfigurator().apply(BotConstants.Arm.cfg);
    velocitySignal = master.getVelocity();
    positionSignal = master.getPosition();
    voltageSignal = master.getMotorVoltage();
    currentSignal = master.getStatorCurrent();

    slave.setControl(new Follower(master.getDeviceID(), false));

    zeroArm();
    // this.setDefaultCommand(Position(()->0));
    this.setDefaultCommand(ArmWithNTPosition());
    new TriggerNTDouble("testing/armAngle", ntpos, (ev) -> ntpos = ev);

    SmartDashboard.putData("ArmPlus", Spin(0.1));
    SmartDashboard.putData("ArmMinus", Spin(-0.1));
    SmartDashboard.putData("Zero Arm", this.runOnce(() -> zeroArm()).ignoringDisable(true));
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

  public void goToPosition(GameInfo.CoralScoringPosition position) {
    master.setControl(positionRequest.withPosition(position.armRot.in(Rotations)));
  }

  public Command Position(DoubleSupplier position) {
    return this.runEnd(() -> master.setControl(positionRequest.withPosition(position.getAsDouble())),
        () -> master.set(0));
  }

  public Command Position(GameInfo.CoralScoringPosition position) {
    return this.startEnd(() -> master.setControl(positionRequest.withPosition(position.armRot.in(Rotations))),
        () -> master.set(0));
  }

  public double getPosition() {
    return master.getPosition().getValueAsDouble();
  }

  public Command ArmWithNTPosition() {
    return this.runEnd(() -> master.setControl(positionRequest.withPosition(ntpos)), () -> master.stopMotor());
  }

  /**
   * Spins the arm at a certain fraction of the motors
   * 
   * @param speed [-1, 1]
   * @return the command that spins the arm
   */
  public Command Spin(double speed) {
    return this.runEnd(() -> master.setControl(voltageRequest.withOutput(speed * 12)), () -> master.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BackupLogger.addToQueue("Arm/VelocityRpsSquared", velocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("Arm/CurrentAmps", currentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("Arm/VoltageVolts", voltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("Arm/PositionRot", positionSignal.refresh().getValue().in(Rotations));
  }
}
