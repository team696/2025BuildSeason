// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BotConstants;
import frc.robot.util.NTTalonForwarder;


/**
 * represents the other coral system that picks up from ground and can score L1
 */
public class GroundCoral extends SubsystemBase {
  private static GroundCoral m_GroundCoral=null;
  public static synchronized final GroundCoral get(){
    if(m_GroundCoral==null){
      m_GroundCoral=new GroundCoral();
    }
    return m_GroundCoral;
  }

  TalonFX angleMotor=new TalonFX(BotConstants.GroundCoral.angleId, BotConstants.rioBus);
  TalonFX rollerMotor=new TalonFX(BotConstants.GroundCoral.rollerId, BotConstants.rioBus);
  NTTalonForwarder angleForwarder=new NTTalonForwarder("GroundCoralAngle", angleMotor);
  NTTalonForwarder rollerForwarder=new NTTalonForwarder("GroundCoralRoller", rollerMotor);
  MotionMagicVoltage positionRequest=new MotionMagicVoltage(0);

  private GroundCoral() {
    angleMotor.getConfigurator().apply(BotConstants.GroundCoral.angleCfg);
    rollerMotor.getConfigurator().apply(BotConstants.GroundCoral.rollerCfg);
    zero();
  }

  public void resetPosition(double newPosition){
    angleMotor.setPosition(newPosition);
  }
  public void zero(){
    resetPosition(0);
  }

  public Command Intake(){
    return this.startEnd(()->{
      angleMotor.setControl(positionRequest.withPosition(2.5));
      rollerMotor.set(0.6);
    }, ()->{
      angleMotor.stopMotor();
      rollerMotor.stopMotor();
    });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleForwarder.update();
    rollerForwarder.update();
  }
}
