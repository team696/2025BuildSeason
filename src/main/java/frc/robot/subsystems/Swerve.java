// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team696.lib.Swerve.SwerveDriveSubsystem;

public class Swerve extends SwerveDriveSubsystem {
  private static Swerve m_Swerve;
  public static synchronized Swerve get(){
    if(m_Swerve==null){
      m_Swerve=new Swerve();
    }
    return m_Swerve;
  }
  private Swerve() {}

  public void Drive(SwerveModuleState[] states){
    setModuleStates(states);
  }

  public void Drive(ChassisSpeeds speeds, boolean fieldRelative){
    if(fieldRelative)
      Drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw()));
    else
      Drive(speeds);
  }

  public void onUpdate(){

  }
}
