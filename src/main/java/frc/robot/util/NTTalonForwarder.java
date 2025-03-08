// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.team696.lib.Logging.BackupLogger;

/** Add your docs here. */
public class NTTalonForwarder {
    String name="";
    public StatusSignal<AngularVelocity> velocitySignal;
    public StatusSignal<Angle> positionSignal;
    public StatusSignal<Voltage> voltageSignal;
    public StatusSignal<Current> currentSignal;
    public NTTalonForwarder(String name, TalonFX motor){
        this.name=name;
        velocitySignal = motor.getVelocity();
        positionSignal = motor.getPosition();
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
    }

    public double getCurrentAmps(){
        return currentSignal.getValue().in(Amps);
    }

    public void update(){
        BackupLogger.addToQueue(name+"/VelocityRpsSquared", velocitySignal.refresh().getValue().in(RotationsPerSecond));
        BackupLogger.addToQueue(name+"/CurrentAmps", currentSignal.refresh().getValue().in(Amps));
        BackupLogger.addToQueue(name+"/VoltageVolts", voltageSignal.refresh().getValue().in(Volts));
        BackupLogger.addToQueue(name+"/PositionRot", positionSignal.refresh().getValue().in(Rotations));
    }
}
