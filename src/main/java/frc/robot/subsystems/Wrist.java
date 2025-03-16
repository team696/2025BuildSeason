// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.robot.util.GameInfo.CoralScoringPosition;

public class Wrist extends SubsystemBase {
  private static Wrist m_Wrist = null;

  public static final synchronized Wrist get() {
    if (m_Wrist == null) {
      m_Wrist = new Wrist();
    }
    return m_Wrist;
  }

  private TalonFX motor = new TalonFX(BotConstants.Wrist.motorID, BotConstants.rioBus);

  MotionMagicVoltage WristPoistionRequest = new MotionMagicVoltage(0);
  VoltageOut WristVoltageRequest = new VoltageOut(0);

  /** Creates a new Wrist. */
  private Wrist() {
    motor.getConfigurator().apply(BotConstants.Wrist.cfg);
    zero();
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
    return motor.getPosition().getValueAsDouble();
  }

  public Command Position(double position) {
    return this.runEnd(() -> goToPosition(position), () -> motor.set(0.0));
  }

  public void goToPosition(double position) {
    motor.setControl(WristPoistionRequest.withPosition(position));

  }

  public void goToPosition(CoralScoringPosition position) {
    goToPosition(position.wristRot.in(Rotation));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist/Position", getPosition());
    // This method will be called once per scheduler run
  }
}
