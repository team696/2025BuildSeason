// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team696.lib.Camera.LimeLightCam;
import frc.team696.lib.Swerve.SwerveDriveSubsystem;

public class Swerve extends SwerveDriveSubsystem {
  private static Swerve m_Swerve;
  public static synchronized Swerve get(){
    if(m_Swerve==null){
      m_Swerve=new Swerve();
    }
    return m_Swerve;
  }
  private LimeLightCam m_frontCam;

  private Swerve() {
    m_frontCam=new LimeLightCam("limelight-corner", true);
  }

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
    m_frontCam.SetRobotOrientation(getYaw());
    // TODO: verify pose estimation actually works, maybe look at what other teams do to improve this 
    m_frontCam.addVisionEstimate(this::addVisionMeasurement, (result)->{
        // AprilTag reads while the robot is rotating quickly may be inaccurate
        //if(result.ambiguity>0.4||getState().angularVelocity()>2.0){
        //  return false;
        //}
        //if(result.pose.getX()<0||result.pose.getX()>8.3||result.pose.getY()<0||result.pose.getY()>16.3)
        //  return false;// robot is literally off the field
        double deviationRatio=(Math.pow((result.distToTag),2)/60);
        m_frontCam.setStdDeviations(deviationRatio, deviationRatio, deviationRatio);
        return true;
    });
  }
}
