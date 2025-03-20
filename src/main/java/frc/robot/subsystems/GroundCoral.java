// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;

/**
 * represents the other coral system that picks up from ground and can score L1
 */
public class GroundCoral extends SubsystemBase {
  private static GroundCoral m_GroundCoral = null;

  public static enum Positions {
    Stowed(0),
    Ready(6.18),
    Spit(6.18),
    Intake(12.5);

    Positions(double value) {
      this.value = value;
    }

    double value = 0;
  }

  public static synchronized final GroundCoral get() {
    if (m_GroundCoral == null) {
      m_GroundCoral = new GroundCoral();
    }
    return m_GroundCoral;
  }

  TalonFX angleMotor = new TalonFX(BotConstants.GroundCoral.angleId, BotConstants.rioBus);
  TalonFX rollerMotor = new TalonFX(BotConstants.GroundCoral.rollerId, BotConstants.rioBus);
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
    angleMotor.setControl(positionRequest.withPosition(position));
  }

  public Command Intake() {
    return this.startEnd(() -> {
      position(Positions.Intake.value);
      rollerMotor.set(0.8);
    }, () -> {
      angleMotor.stopMotor();
      //rollerMotor.stopMotor();
    });
  }

  public Command Stowed() {
    return this.runEnd(
        () -> {
          if (Elevator.get().getPosition() > 20
              || this.getPosition() < 4) {
            position(Positions.Stowed.value);
          } else {
            position(Positions.Ready.value);
          }
          rollerMotor.stopMotor();
        },
        this::stop);
  }



  public Command Ready() {
    return this.runEnd(
        () -> {
          if (Elevator.get().getPosition() > 20 || this.getPosition() > 6.) {
            position(Positions.Ready.value);
          } else {
            position(Positions.Stowed.value);
          }
          rollerMotor.set(0.4);
        },
        this::stop);
  }

  public Command Spit() {
    return this.runEnd(() -> {
      position(Positions.Spit.value);
      rollerMotor.set(-0.3);
    }, this::stop);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("GroundCoral/Position", getPosition());
    // This method will be called once per scheduler run
  }
}
