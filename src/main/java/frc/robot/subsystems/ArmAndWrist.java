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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.robot.util.GameInfo;
import frc.team696.lib.Logging.BackupLogger;

public class ArmAndWrist extends SubsystemBase {
  private static ArmAndWrist arm=null;
  public static synchronized ArmAndWrist get(){
    if(arm==null){
      arm=new ArmAndWrist();
    }
    return arm;
  }
  TalonFX armMotor= new TalonFX(BotConstants.Arm.motorID,BotConstants.rioBus);
  TalonFX wristMotor=new TalonFX(BotConstants.Wrist.motorID, BotConstants.rioBus);
  StatusSignal<AngularVelocity> armVelocitySignal;
  StatusSignal<Angle> armPositionSignal;
  StatusSignal<Voltage> armVoltageSignal;
  StatusSignal<Current> armCurrentSignal;
  StatusSignal<AngularVelocity> wristVelocitySignal;
  StatusSignal<Angle> wristPositionSignal;
  StatusSignal<Voltage> wristVoltageSignal;
  StatusSignal<Current> wristCurrentSignal;
  MotionMagicVoltage positionRequest=new MotionMagicVoltage(0);
  VoltageOut voltageRequest=new VoltageOut(0);
  
  /** Creates a new Arm. */
  private ArmAndWrist() {
    armMotor.getConfigurator().apply(BotConstants.Arm.cfg);
    wristMotor.getConfigurator().apply(BotConstants.Wrist.cfg);
    armVelocitySignal=armMotor.getVelocity();
    armPositionSignal=armMotor.getPosition();
    armVoltageSignal=armMotor.getMotorVoltage();
    armCurrentSignal=armMotor.getStatorCurrent();

    wristVelocitySignal=wristMotor.getVelocity();
    wristPositionSignal=wristMotor.getPosition();
    wristVoltageSignal=wristMotor.getMotorVoltage();
    wristCurrentSignal=wristMotor.getStatorCurrent();
    SmartDashboard.putData("ArmPlus", Spin(0.1));
    SmartDashboard.putData("ArmMinus", Spin(-0.1));
  }


  public Command Position(DoubleSupplier position){
    return this.runEnd(()->armMotor.setControl(positionRequest.withPosition(position.getAsDouble())), ()->armMotor.set(0));
  }
  public Command Position(GameInfo.CoralScoringPosition position){
    return this.runOnce(()->System.out.println("going to "+GameInfo.L3.rot.in(Rotations))).andThen(this.startEnd(()->armMotor.setControl(positionRequest.withPosition(position.rot.in(Rotations))), ()->armMotor.set(0)));
  }

  /**
   * Spins the arm at a certain fraction of the motors
   * @param speed [-1, 1] 
   * @return the command that spins the arm 
   */
  public Command Spin(double speed){
    return this.runEnd(()->armMotor.setControl(voltageRequest.withOutput(speed*12)), ()->armMotor.set(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BackupLogger.addToQueue("Arm/VelocityRpsSquared", armVelocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("Arm/CurrentAmps", armCurrentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("Arm/VoltageVolts", armVoltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("Arm/PositionRot", armPositionSignal.getValue().in(Rotations));

    BackupLogger.addToQueue("Wrist/VelocityRpsSquared", wristVelocitySignal.refresh().getValue().in(RotationsPerSecond));
    BackupLogger.addToQueue("Wrist/CurrentAmps", wristCurrentSignal.refresh().getValue().in(Amps));
    BackupLogger.addToQueue("Wrist/VoltageVolts", wristVoltageSignal.refresh().getValue().in(Volts));
    BackupLogger.addToQueue("Wrist/PositionRot", wristPositionSignal.getValue().in(Rotations));
  }
}
