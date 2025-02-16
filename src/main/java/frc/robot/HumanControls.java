// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team696.lib.Swerve.Commands.TeleopSwerve;

/** Add your docs here. */
public final class HumanControls {
    // comment out or remove this class when the 2025 operator panel comes out
    public final class OperatorPanel2024{
        public static final Joystick operatorPanel = new Joystick(1);
        public static final Joystick operatorPanelB = new Joystick(2);

        public static final JoystickButton Shoot = new JoystickButton(operatorPanel, 1);
        public static final JoystickButton Amp = new JoystickButton(operatorPanel, 2);
        public static final JoystickButton ExtraA = new JoystickButton(operatorPanel, 3);
        public static final JoystickButton Trap = new JoystickButton(operatorPanel, 4);
        public static final JoystickButton ExtraB = new JoystickButton(operatorPanel, 6);
        public static final JoystickButton Ground = new JoystickButton(operatorPanel, 7);
        public static final JoystickButton Source = new JoystickButton(operatorPanel, 8);
        public static final JoystickButton Rollers = new JoystickButton(operatorPanel, 9);
        public static final JoystickButton Drop = new JoystickButton(operatorPanel, 10);
        public static final JoystickButton ExtraC = new JoystickButton(operatorPanel,11);

        public static final JoystickButton Climb = new JoystickButton(operatorPanelB, 2);
        public static final JoystickButton OhShit = new JoystickButton(operatorPanelB,   3);
        public static final JoystickButton Rightest = new JoystickButton(operatorPanelB, 4);
        public static final JoystickButton Right = new JoystickButton(operatorPanelB, 5);
        public static final JoystickButton Left = new JoystickButton(operatorPanelB, 6);
        public static final JoystickButton Gyro = new JoystickButton(operatorPanelB, 7);
        public static final JoystickButton Leftest = new JoystickButton(operatorPanelB, 8);
    }
    public final class OperatorPanel2025{
        public static final JoystickButton lockScoringPose=null;
        public static final JoystickButton scoreL1=null;
        public static final JoystickButton scoreL2=null;
        public static final JoystickButton scoreL3=null;
        public static final JoystickButton scoreL4=null;
        public static final JoystickButton leftOrRight=null;
        // Use one of the switches on the driver station for this
        public static final BooleanSupplier manualOverride=null;
    }
    public final class DriverStation{
        public static final Joystick DriverPanel=new Joystick(0);
        public static final Joystick OperatorPanelA=new Joystick(1);
        public static final Joystick OperatorPanelB=new Joystick(2);
        public static final Joystick OperatorPanelC=new Joystick(3);
        public static final JoystickButton resetGyro=new JoystickButton(DriverPanel, 1);

        public static final DoubleSupplier leftJoyY = ()->-DriverPanel.getRawAxis(1);
        public static final DoubleSupplier leftJoyX = ()->DriverPanel.getRawAxis(0);
        public static final DoubleSupplier rightJoyX = ()->-DriverPanel.getRawAxis(2)/3;
        
    }
    public final class SingleXboxController{
        public static final CommandXboxController controller = new CommandXboxController(5);

        public static final DoubleSupplier leftJoyY =  ()->-controller.getRawAxis(1);
        public static final DoubleSupplier leftJoyX =  ()->-controller.getRawAxis(0);
        public static final DoubleSupplier rightJoyX = ()->-controller.getRawAxis(4);

        public static final Trigger A = controller.a();
        public static final Trigger B = controller.b();
        public static final Trigger X = controller.x();
        public static final Trigger Y = controller.y();

        public static final Trigger RB = controller.rightBumper();
        public static final Trigger LB = controller.leftBumper();

        public static final Trigger LT = controller.leftTrigger(0.5);
        public static final Trigger RT = controller.rightTrigger(0.5);

    }
    public final class SinglePS4Controller{

    }
    
}
