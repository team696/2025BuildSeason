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
    m_slave=new TalonFactory(BotConstants.Elevator.slaveId,BotConstants.canivoreBus, BotConstants.Elevator.cfg, "Elevator Slave");
    //m_angle=new TalonFactory();
    m_slave.Follow(m_master, true);// TODO: determine of the slave needs to go in the same or in the opposite direction to the master
    
    positionReq=new MotionMagicVoltage(0);
    
    identificationRoutine=new SysIdRoutine(new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(0.4),Seconds.of(3)), 
    
    new SysIdRoutine.Mechanism(this::DriveVoltage, (log)->{
      log.motor(m_master.getName()).voltage(m_master.get().getMotorVoltage().getValue())
      .linearPosition(Meters.of(m_master.getPosition()))
      .linearVelocity(MetersPerSecond.of(m_master.getVelocity()));
    }, m_Elevator));
  }

  /**
   * Use ONLY for SysID. Sets the elevator motors to a specific voltage
   */
  public void DriveVoltage(Voltage v){
    m_master.VoltageOut(v);
  }
  /**
   * Moves to and holds a position
   * @param position
   * @return 
   */
  public Command positionCommand(GameInfo.CoralScoringPosition position){
    // TODO: unfuck this before we actually practice
    return this.startEnd(()->m_master.setControl(positionReq.withPosition(position.height)), ()->m_master.VoltageOut(Volts.of(0)));
  }
  /**
   * Holds the current position
   * @return a command that holds the elevator at the position it was in at schedluing time
   */
  public Command holdPosition(){
    return this.startEnd(()->m_master.setControl(positionReq.withPosition(m_master.getPosition())), ()->m_master.VoltageOut(Volts.of(0)));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
