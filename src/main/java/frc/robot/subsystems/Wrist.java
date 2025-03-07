// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
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
import frc.robot.util.GameInfo.CoralScoringPosition;
import frc.team696.lib.Logging.BackupLogger;

public class Wrist extends SubsystemBase {
  private static Wrist m_Wrist = null;

  public static final synchronized Wrist get() {
    if (m_Wrist == null) {
      m_Wrist = new Wrist();
    }
    return m_Wrist;
  }

  public TalonFX motor = new TalonFX(BotConstants.Wrist.motorID, BotConstants.rioBus);

  StatusSignal<AngularVelocity> velocitySignal;
  StatusSignal<Angle> positionSignal;
  StatusSignal<Voltage> voltageSignal;
  StatusSignal<Current> currentSignal;

  MotionMagicVoltage WristPoistionRequest = new MotionMagicVoltage(0);
  VoltageOut WristVoltageRequest = new VoltageOut(0);

  /** Creates a new Wrist. */
  private Wrist() {
    motor.getConfigurator().apply(BotConstants.Wrist.cfg);
    zero();
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();
    voltageSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
    SmartDashboard.putData("Zero Wrist", this.runOnce(() -> zero()).ignoringDisable(true));
  }

  public void resetPosition(double newPosition) {
    motor.setPosition(newPosition);
  }

  public void zero() {
    resetPosition(0);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  public Command Position(double position){
    return this.runEnd(()->goToPosition(position), ()->motor.set(0.0));
  }

  public void goToPosition(double position) {
    motor.setControl(WristPoistionRequest.withPosition(position));

  }

  public void goToPosition(CoralScoringPosition position){
    goToPosition(position.wristRot.in(Rotation));
  }


  @Override
  public void periodic() {
    

    // This method will be called once per scheduler run
    BackupLogger.addToQueue("Wrist/VelocityRpsSquared", velocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("Wrist/CurrentAmps", currentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("Wrist/VoltageVolts", voltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("Wrist/PositionRot", positionSignal.refresh().getValue().in(Rotation));
  }
}
