// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.team696.lib.Logging.BackupLogger;

public class Climber extends SubsystemBase {
  private static Climber m_Climber=null;
  public static synchronized Climber get(){
    if(m_Climber==null){
      m_Climber=new Climber();
    }
    return m_Climber;
  }

  private TalonFX motor;
  private MotionMagicVoltage positionReq;

  private Climber() {
    motor=new TalonFX(BotConstants.Climber.motorID, BotConstants.canivoreBus);
    positionReq=new MotionMagicVoltage(0);
    zero();
    this.setDefaultCommand(In());
  }

  public void stop(){
    motor.stopMotor();
  }

  public Command Out(){
    return this.runEnd(
      ()->{motor.setControl(positionReq.withPosition(14));},
      ()->{stop();}
    );
  }
  public Command In(){
    return this.runEnd(
      ()->{motor.setControl(positionReq.withPosition(0));},
      ()->{stop();}
    );
  }
  public void resetPosition(double newPosition){
    motor.setPosition(newPosition);
  }

  public void zero(){
    resetPosition(0);
  }

  @Override
  public void periodic() {
    BackupLogger.addToQueue("climber/Postion", motor.getPosition().getValueAsDouble());
  }
}
