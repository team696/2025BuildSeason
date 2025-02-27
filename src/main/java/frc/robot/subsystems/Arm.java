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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.team696.lib.Logging.BackupLogger;

public class Arm extends SubsystemBase {
  private static Arm arm=null;
  public static synchronized Arm get(){
    if(arm==null){
      arm=new Arm();
    }
    return arm;
  }
  TalonFX armMotor= new TalonFX(BotConstants.Arm.motorID,BotConstants.rioBus);
  StatusSignal<AngularVelocity> velocitySignal;
  StatusSignal<Angle> positionSignal;
  StatusSignal<Voltage> voltageSignal;
  StatusSignal<Current> currentSignal;
  MotionMagicVoltage positionRequest=new MotionMagicVoltage(0);
  VoltageOut voltageRequest=new VoltageOut(0);
  
  /** Creates a new Arm. */
  private Arm() {
    armMotor.getConfigurator().apply(BotConstants.Arm.cfg);
    velocitySignal=armMotor.getVelocity();
    positionSignal=armMotor.getPosition();
    voltageSignal=armMotor.getMotorVoltage();
    currentSignal=armMotor.getStatorCurrent();

  }


  public Command Position(DoubleSupplier position){
    return this.runEnd(()->armMotor.setControl(positionRequest.withPosition(position.getAsDouble())), ()->armMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BackupLogger.addToQueue("Arm/VelocityRpsSquared", velocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("Arm/CurrentAmps", currentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("Arm/VoltageVolts", voltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("Arm/PositionRot", positionSignal.getValue().in(Rotations));
  }
}
