// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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
        public static int masterID=0;
        public static int slaveId=1;
        public static TalonFXConfiguration cfg;
        static{
            cfg=new TalonFXConfiguration();
            // TODO: determine all these constants (Feedback first, feedforward if needed)
            cfg.Slot0.kP=0;
            cfg.Slot0.kV=0;
            cfg.Slot0.kV=0;
            cfg.Slot0.kG=0;
            cfg.Slot0.kA=0;
            cfg.MotionMagic.MotionMagicCruiseVelocity=0;
            cfg.MotionMagic.MotionMagicAcceleration=0;
            cfg.Feedback.SensorToMechanismRatio=0;
        }

    }
}
