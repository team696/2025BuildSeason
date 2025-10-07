package frc.team696.lib.HardwareDevices;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Base interface for switching between different Gyro Devices
 */
public interface GyroInterface {
    public abstract Rotation2d getYaw();

    public abstract void resetYaw();

    public abstract AngularVelocity getAngularVelocity();
}
