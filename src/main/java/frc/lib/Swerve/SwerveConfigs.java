package frc.team696.lib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * 
 * Device Configs of the Swerve Drive
 * 
 * <p> Only Update Device Configs if you know what you are doing.
 */
public final class SwerveConfigs {
        public final static TalonFXConfiguration angle;
        public final static TalonFXConfiguration drive;
        public final static CANcoderConfiguration canCoder;
        public final static Pigeon2Configuration pigeon;
        public final static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT;
        public final static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT;
        public final static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT;
        public final static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT;
        static {
                angle = new TalonFXConfiguration();
                drive = new TalonFXConfiguration();
                canCoder = new CANcoderConfiguration();  
                pigeon = new Pigeon2Configuration();

                FRONT_LEFT = new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
                FRONT_RIGHT = new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
                BACK_LEFT = new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
                BACK_RIGHT = new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
                
                /** Swerve CANCoder Configuration */
                canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                canCoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

                /** Swerve Angle Motor Configuration */
                angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                angle.Feedback.SensorToMechanismRatio = SwerveConstants.ANGLE_GEAR_RATIO;
                angle.ClosedLoopGeneral.ContinuousWrap = true;
                angle.CurrentLimits.SupplyCurrentLimitEnable = true;
                angle.CurrentLimits.SupplyCurrentLimit = 25;
                angle.CurrentLimits.SupplyCurrentLowerLimit = 60;
                angle.CurrentLimits.SupplyCurrentLowerTime = 0.1;
                angle.CurrentLimits.StatorCurrentLimitEnable = true;
                angle.CurrentLimits.StatorCurrentLimit = 80;
                angle.Slot0.kP = 150.0;
                angle.Slot0.kI = 0.0;
                angle.Slot0.kD = 0.0;

                angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
                angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
                angle.Voltage.PeakForwardVoltage = 12.;
                angle.Voltage.PeakReverseVoltage = -12.;

                /** Swerve Drive Motor Configuration */
                drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drive.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
                drive.CurrentLimits.SupplyCurrentLimit = 25;
                drive.CurrentLimits.SupplyCurrentLowerLimit = 90;
                drive.CurrentLimits.SupplyCurrentLowerTime = 0.2;
                drive.CurrentLimits.StatorCurrentLimitEnable = true;
                drive.CurrentLimits.StatorCurrentLimit = 110;
                drive.Voltage.PeakForwardVoltage = 12.;
                drive.Voltage.PeakReverseVoltage = -12.;

                drive.Slot0.kP = 2.;
                drive.Slot0.kI = 0.0;
                drive.Slot0.kD = 0.0;
                drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
                drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
                drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02;
                drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

                /** Individual Swerve Module Configurations */ 
                FRONT_LEFT.EncoderId = 0;  
                FRONT_LEFT.DriveMotorId = 1; 
                FRONT_LEFT.SteerMotorId = 0;
                FRONT_LEFT.EncoderOffset = -0.24-.25;
        
                FRONT_RIGHT.EncoderId = 1; 
                FRONT_RIGHT.DriveMotorId = 3;
                FRONT_RIGHT.SteerMotorId = 2;
                FRONT_RIGHT.EncoderOffset = -0.393-.25;

                BACK_LEFT.EncoderId = 2; 
                BACK_LEFT.DriveMotorId = 5;
                BACK_LEFT.SteerMotorId = 4;
          
                BACK_LEFT.EncoderOffset = -0.456-.25;
                
                BACK_RIGHT.EncoderId = 3; 
                BACK_RIGHT.DriveMotorId = 7;
                BACK_RIGHT.SteerMotorId = 6;
                BACK_RIGHT.EncoderOffset = -0.03-.25;

                /** Pigeon Configuration */ 
                pigeon.MountPose.MountPoseYaw = 0;
        }
}
