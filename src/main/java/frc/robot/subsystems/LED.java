// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase { 
  private static LED m_LED=null;
  public static synchronized LED get(){
    if(m_LED==null){
      m_LED=new LED();
    }
    return m_LED;
  }



  
  /** Creates a new LED. */
  private LED() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
