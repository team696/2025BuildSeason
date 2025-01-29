// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team696.lib.Swerve.Commands.TeleopSwerve;

/** Add your docs here. */
public final class HumanControls {
    public final class DriverStation{
        public static final Joystick DriverPanel=new Joystick(0);
        public static final Joystick OperatorPanelA=new Joystick(1);
        public static final Joystick OperatorPanelB=new Joystick(2);
        public static final Joystick OperatorPanelC=new Joystick(3);
        public static final JoystickButton resetGyro=new JoystickButton(DriverPanel, 1);
        public static final DoubleSupplier leftJoyY = ()->-DriverPanel.getRawAxis(1);
        public static final DoubleSupplier leftJoyX = ()->DriverPanel.getRawAxis(0);
        public static final DoubleSupplier rightJoyX = ()->-DriverPanel.getRawAxis(2);
        
    }
    public final class SingleXboxController{

    }
    public final class SinglePS4Controller{

    }
}
