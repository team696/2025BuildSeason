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
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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

/**
 * This class represents the Coral scoring mechanism attached to the arm of the
 * elevator. The only thing that can <strong> really </strong> be controlled
 * here is the speed of intake and outake.
 */
public class EndEffector extends SubsystemBase {
  private static EndEffector m_EndEffector = null;

  public static final synchronized EndEffector get() {
    if (m_EndEffector == null) {
      m_EndEffector = new EndEffector();
    }
    return m_EndEffector;
  }

  /*
   * <p> Creates a new EndEffector. </p>
   */
  TalonFX motor = new TalonFX(BotConstants.EndEffector.motorID, BotConstants.rioBus);
  VoltageOut VoltageRequest = new VoltageOut(0);
  MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);
  StatusSignal<AngularVelocity> velocitySignal;
  StatusSignal<Angle> positionSignal;
  StatusSignal<Voltage> voltageSignal;
  StatusSignal<Current> currentSignal;

  public EndEffector() {
    motor.getConfigurator().apply(BotConstants.EndEffector.cfg);
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    SmartDashboard.putData("Intake", spin(0.6));
    SmartDashboard.putData("Eject", spin(-0.6));
  }

  public void stop() {
    motor.stopMotor();
  }

  public void run(double output) {
    motor.setControl(VoltageRequest.withOutput(output * 12));
  }

  /**
   * spins the roller at a fraction of it's power <i> power </i?
   * 
   * @param power [-1,1] the fraction of the power that the roller can exert at
   *              12V
   * @return the command
   */
  public Command spin(double power) {
    return this.startEnd(() -> motor.setControl(VoltageRequest.withOutput(power * 12)), () -> motor.set(0));
  }

  public Command spin(DoubleSupplier power) {
    return this.runEnd(() -> motor.setControl(VoltageRequest.withOutput(power.getAsDouble() * 12)), () -> motor.set(0));
  }

  public Command spinVelocity(double velocity) {
    return this.startEnd(() -> motor.setControl(velocityOut.withVelocity(velocity)), () -> motor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BackupLogger.addToQueue("EndEffector/VelocityRpsSquared",
        velocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("EndEffector/CurrentAmps", currentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("EndEffector/VoltageVolts", voltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("EndEffector/PositionRot", positionSignal.refresh().getValue().in(Rotations));
  }
}
