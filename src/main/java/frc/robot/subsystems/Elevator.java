// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.BotConstants;
import frc.robot.util.GameInfo;
import frc.team696.lib.HardwareDevices.TalonFactory;
import frc.team696.lib.Logging.BackupLogger;

public class Elevator extends SubsystemBase {
  private static Elevator m_Elevator=null;
  public static synchronized Elevator get(){
    if(m_Elevator==null){
      m_Elevator=new Elevator();
    }
    return m_Elevator;
  }
  private TalonFactory m_master, m_slave;
  private MotionMagicVoltage positionReq;
  public SysIdRoutine identificationRoutine;
  // controls the angle of the scoring arm
  public TalonFactory m_angle;

  private Elevator() {
    m_master=new TalonFactory(BotConstants.Elevator.masterID, BotConstants.rioBus, BotConstants.Elevator.cfg, "Elevator Master");
    m_slave=new TalonFactory(BotConstants.Elevator.slaveId,BotConstants.rioBus, BotConstants.Elevator.cfg, "Elevator Slave");

    m_slave.Follow(m_master, false);
    
    positionReq=new MotionMagicVoltage(0).withSlot(0);
    
    identificationRoutine=new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(0.4),Seconds.of(3)), 
    
    new SysIdRoutine.Mechanism(this::DriveVoltage, (log)->{
      log.motor(m_master.getName()).voltage(m_master.get().getMotorVoltage().getValue())
      .linearPosition(Meters.of(m_master.getPosition()))
      .linearVelocity(MetersPerSecond.of(m_master.getVelocity()));
    }, this));
    this.setDefaultCommand(positionCommand(0));
  }

  /**
   * Use ONLY for SysID. Sets the elevator motors to a specific voltage
   */
  public void DriveVoltage(Voltage v){
    m_master.VoltageOut(v);
  }
  /**
   * Moves to and holds a position
   * @param position The scoring position to hold
   * @return the command which holds the elevator at position (requires this subsystem)
   */
  public Command positionCommand(GameInfo.CoralScoringPosition position){
    return this.startEnd(()->m_master.setControl(positionReq.withPosition(position.height)), ()->m_master.VoltageOut(Volts.of(0)));
  }

  public Command positionCommand(double position){
    return this.startEnd(()->m_master.setControl(positionReq.withPosition(position)), ()->m_master.VoltageOut(Volts.of(0)));
  }
  /**
   * Holds the current position
   * @return a command that holds the elevator at the position it was in at schedluing time
   */
  public Command holdPosition(){
    return this.startEnd(()->m_master.setControl(positionReq.withPosition(m_master.getPosition())), ()->m_master.VoltageOut(Volts.of(0)));
  }
  public void zeroPosition(){
    resetPosition(0);
  }
  public void resetPosition(double newPosition){
    m_master.setPosition(newPosition);
    m_slave.setPosition(newPosition);
  }
  @Override
  public void periodic() {
    BackupLogger.addToQueue("Elevator/MasterCurrent", m_master.getCurrent());
    BackupLogger.addToQueue("Elevator/SlaveCurrent", m_slave.getCurrent());
    BackupLogger.addToQueue("Elevator/MasterPosition", m_master.getPosition());
    BackupLogger.addToQueue("Elevator/SlavePosition", m_slave.getPosition());
    BackupLogger.addToQueue("Elevator/SlaveVelocity", m_slave.getVelocity());
    BackupLogger.addToQueue("Elevator/MasterVelocity", m_master.getVelocity());

  }
}
