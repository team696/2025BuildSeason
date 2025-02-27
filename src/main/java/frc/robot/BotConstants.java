// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * Constants for robot mechanisms unrelated to the drive train
*/
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
        public static int masterID=11;
        public static int slaveId=10;
        public static TalonFXConfiguration cfg;
        static{
            cfg=new TalonFXConfiguration();
            cfg.Slot0.kP=(1.0/20.0)*20;
            cfg.Slot0.kG=0.05;
            cfg.Slot0.kS=0.02;
            cfg.Slot0.GravityType=GravityTypeValue.Elevator_Static;
            cfg.Slot0.StaticFeedforwardSign=StaticFeedforwardSignValue.UseVelocitySign;

            cfg.MotionMagic.MotionMagicCruiseVelocity=128.;
            cfg.MotionMagic.MotionMagicAcceleration=128.;
            cfg.MotionMagic.MotionMagicJerk=128.;
            cfg.MotorOutput.NeutralMode=NeutralModeValue.Coast;
            cfg.CurrentLimits.StatorCurrentLimit=60.;
            cfg.CurrentLimits.StatorCurrentLimitEnable=true;

        }

    }
    public static class Arm{
        public static int motorID=13;
        public static TalonFXConfiguration cfg=new TalonFXConfiguration();
        static{
            cfg.MotorOutput.NeutralMode=NeutralModeValue.Brake;
            cfg.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
            cfg.Slot0.kP=1/10;
            cfg.CurrentLimits.StatorCurrentLimitEnable=true;
            cfg.CurrentLimits.StatorCurrentLimit=120;
            cfg.MotionMagic.MotionMagicCruiseVelocity=0;
            cfg.MotionMagic.MotionMagicAcceleration=0;
            cfg.MotionMagic.MotionMagicJerk=0;
            // TODO: figure out the ratios   
        }
    }
    public static class ClimberIntake{
        public static int masterID=0;
        public static int slaveID=1;
        public static TalonFXConfiguration cfg=new TalonFXConfiguration();
        static{
            cfg.MotorOutput.NeutralMode=NeutralModeValue.Coast;
            cfg.CurrentLimits.StatorCurrentLimitEnable=true;
            cfg.CurrentLimits.StatorCurrentLimit=120;
            cfg.MotionMagic.MotionMagicCruiseVelocity=20;
            cfg.MotionMagic.MotionMagicAcceleration=15;
            cfg.MotionMagic.MotionMagicJerk=30;
        }
    }

}
