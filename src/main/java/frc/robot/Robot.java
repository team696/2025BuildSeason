// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import frc.team696.lib.Logging.BackupLogger;
import frc.team696.lib.Swerve.Commands.TeleopSwerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;


  public Robot() {
    
    SmartDashboard.putData(CommandScheduler.getInstance());
    configureDriverStationBinds();
    Swerve.get().setDefaultCommand(TeleopSwerve.New());
  }

  private void configureDriverStationBinds(){
    TeleopSwerve.config(Swerve.get(), HumanControls.DriverStation.leftJoyX, HumanControls.DriverStation.leftJoyY, HumanControls.DriverStation.rightJoyX, null, 0.07);  
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    BackupLogger.logSystemInformation();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
