// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.team696.lib.Logging.BackupLogger;

/**
 * represents the other coral system that picks up from ground and can score L1
 */
public class GroundCoral extends SubsystemBase {
  private static GroundCoral m_GroundCoral = null;

  public static synchronized final GroundCoral get() {
    if (m_GroundCoral == null) {
      m_GroundCoral = new GroundCoral();
    }
    return m_GroundCoral;
  }

  TalonFX angleMotor = new TalonFX(BotConstants.GroundCoral.angleId, BotConstants.rioBus);
  public TalonFX rollerMotor = new TalonFX(BotConstants.GroundCoral.rollerId, BotConstants.rioBus);
  MotionMagicDutyCycle positionRequest = new MotionMagicDutyCycle(0);

  private GroundCoral() {
    angleMotor.getConfigurator().apply(BotConstants.GroundCoral.angleCfg);
    rollerMotor.getConfigurator().apply(BotConstants.GroundCoral.rollerCfg);
    zero();
    this.setDefaultCommand(this.runEnd(() -> {
      angleMotor.setControl(positionRequest.withPosition(0));
    }, () -> {
      angleMotor.stopMotor();
    }));
  }

  public void resetPosition(double newPosition) {
    angleMotor.setPosition(newPosition);
  }

  public void zero() {
    resetPosition(0);
  }

  public void stop() {
    angleMotor.stopMotor();
    rollerMotor.stopMotor();
  }

  public boolean isStalling() {
    return rollerMotor.getStatorCurrent().getValue()
        .in(Amp) > (BotConstants.GroundCoral.rollerCfg.CurrentLimits.StatorCurrentLimit - 20);
  }

  public double getPosition() {
    return angleMotor.getPosition().getValue().in(Rotation);
  }

  public void position(double position) {
    if (Elevator.get().getPosition() >= 16) {
      angleMotor.setControl(positionRequest.withPosition(position));
    } else {
      stop();
    }
  }

  public Command Intake() {
    return this.startEnd(() -> {
      position(14);
      rollerMotor.set(1.);
    }, () -> {
      angleMotor.stopMotor();
      rollerMotor.stopMotor();
    });
  }

  public Command Stowed() {
    return this.runEnd(
        () -> {
          position(0);
          rollerMotor.stopMotor();
        },
        this::stop);
  }

  public Command Ready() {
    return this.runEnd(
        () -> {
          position(6.4);
          rollerMotor.set(0.3);
        },
        this::stop);
  }

  public Command Spit() {
    return this.runEnd(() -> {
      position(6.4);
      rollerMotor.set(-0.6);
    }, this::stop);
  }

  @Override
  public void periodic() {
    BackupLogger.addToQueue("GroundCoral/Position", angleMotor.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
