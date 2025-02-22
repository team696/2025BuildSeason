// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** Add your docs here. */
public class BotConstants {
    // The CAN Bus attached directly to the RoboRIO, used for devices controlling non-drivetrain mechanisms
    public static CANBus rioBus;
    // Used for drivetrain devices (motors, encoders, IMU, etc.). Supports CAN FD.
    public static CANBus canivoreBus;
    static{
        rioBus=new CANBus("rio");
        canivoreBus=new CANBus("cv");
    }
    public static class Elevator{
        public static int masterID=10;
        public static int slaveId=11;
        public static TalonFXConfiguration cfg;
        static{
            cfg=new TalonFXConfiguration();
            // TODO: determine all these constants (Feedback first, feedforward if needed)
            cfg.Slot0.kP=1.0/20.0;
            cfg.Slot0.kG=0.06;
            cfg.Slot0.kS=0.02;
            cfg.Slot0.GravityType=GravityTypeValue.Elevator_Static;
            cfg.Slot0.StaticFeedforwardSign=StaticFeedforwardSignValue.UseVelocitySign;

            cfg.MotionMagic.MotionMagicCruiseVelocity=4.;
            cfg.MotionMagic.MotionMagicAcceleration=8.;
            cfg.MotionMagic.MotionMagicJerk=5.;
            cfg.MotorOutput.NeutralMode=NeutralModeValue.Coast;
            cfg.CurrentLimits.StatorCurrentLimit=60.;
            cfg.CurrentLimits.StatorCurrentLimitEnable=true;

        }

    }
}
