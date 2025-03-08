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
  private TalonFactory master, slave;
  private MotionMagicVoltage positionReq;
  public SysIdRoutine identificationRoutine;

  private Elevator() {
    master=new TalonFactory(BotConstants.Elevator.masterID, BotConstants.rioBus, BotConstants.Elevator.cfg, "Elevator Master");
    slave=new TalonFactory(BotConstants.Elevator.slaveId,BotConstants.rioBus, BotConstants.Elevator.cfg, "Elevator Slave");

    slave.Follow(master, true);
    
    positionReq=new MotionMagicVoltage(0).withSlot(0);
    zero();
    identificationRoutine=new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(0.4),Seconds.of(3)), 
    new SysIdRoutine.Mechanism(this::DriveVoltage, (log)->{
      log.motor(master.getName()).voltage(master.get().getMotorVoltage().getValue())
      .linearPosition(Meters.of(master.getPosition()))
      .linearVelocity(MetersPerSecond.of(master.getVelocity()));
    }, this));
  }

  public void stop() {
    master.stop();
  }
  /**
   *
   * Use ONLY for SysID. Sets the elevator motors to a specific voltage
   */
  public void DriveVoltage(Voltage v){
    master.VoltageOut(v);
  }

  public void goToPosition(double position) {
    if (GroundCoral.get().getPosition() > .1) {
      position = Math.max(position, 30);
    }
    master.get().setControl(positionReq.withPosition(position));
  }

  public void goToPosition(GameInfo.CoralScoringPosition position) {
    goToPosition(position.height);
  } 

  public double getPosition() {
    return master.getPosition();
  }
  /**
   * 
   * Moves to and holds a position
   * @param position The scoring position to hold
   * @return the command which holds the elevator at position (requires this subsystem)
   */
  public Command positionCommand(GameInfo.CoralScoringPosition position){
    return this.runEnd(()->goToPosition(position), ()->master.get().set(0));
  }
  public Command positionCommand(double position){
    return this.runEnd(()->goToPosition(position), ()->master.VoltageOut(Volts.of(0)));
  }
  /**
   * Holds the current position
   * @return a command that holds the elevator at the position it was in at schedluing time
   */
  public Command holdPosition(){
    return this.startEnd(()->goToPosition(master.getPosition()), ()->master.VoltageOut(Volts.of(0)));
  }

  public void zero(){
    resetPosition(0);
  }

  public void resetPosition(double newPosition){
    master.setPosition(newPosition);
    slave.setPosition(newPosition);
  }

  @Override
  public void periodic() {
    BackupLogger.addToQueue("Elevator/MasterCurrent", master.getCurrent());
    BackupLogger.addToQueue("Elevator/SlaveCurrent", slave.getCurrent());
    BackupLogger.addToQueue("Elevator/MasterPosition", master.getPosition());
    BackupLogger.addToQueue("Elevator/SlavePosition", slave.getPosition());
    BackupLogger.addToQueue("Elevator/SlaveVelocity", slave.getVelocity());
    BackupLogger.addToQueue("Elevator/MasterVelocity", master.getVelocity());

  }
}
