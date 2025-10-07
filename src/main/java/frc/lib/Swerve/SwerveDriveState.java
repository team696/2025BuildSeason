// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Swerve;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

/** 
 * Holds the current State of the robot and last update.
 * 
 * <p> Should Not be manually updated
 */
public class SwerveDriveState implements StructSerializable {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    public ChassisSpeeds robotAcceleration = new ChassisSpeeds();
    public double timeStamp = 0;
    public double timeSinceLastUpdate = -1;

    /** Use This to Scale trust of camera measurements, decreasing it as you get updates, up to you how you want to change it. */
    public double trust = 1e-9; // Never Set To 0

    public SwerveDriveState(Pose2d pose, ChassisSpeeds speeds, ChassisSpeeds accelerations, double time, double trust) {
        update(pose, speeds, accelerations, time, trust);
    }

    public SwerveDriveState(Pose2d pose, ChassisSpeeds speeds, ChassisSpeeds acceleration, double time, double timeSinceLastUpdate, double trust) {
        this.pose = pose;
        this.robotRelativeSpeeds = speeds;
        this.robotAcceleration = acceleration;
        this.timeStamp = time;
        this.timeSinceLastUpdate = timeSinceLastUpdate;
        this.trust = trust;
    }

    public SwerveDriveState(SwerveDriveState other) {
        this(other.pose, other.robotRelativeSpeeds, other.robotAcceleration, other.timeStamp, other.trust);
        this.timeSinceLastUpdate = other.timeSinceLastUpdate;
    }

    public SwerveDriveState() {
        this(new Pose2d(), new ChassisSpeeds(), new ChassisSpeeds(), 0, 1e-9);
    }

    /**
     * 
     * @return Current X,Y velocity of the robot in M/S
     */
    public double speed() {
        return Math.sqrt(robotRelativeSpeeds.vxMetersPerSecond * robotRelativeSpeeds.vxMetersPerSecond + robotRelativeSpeeds.vyMetersPerSecond * robotRelativeSpeeds.vyMetersPerSecond);
    }

    /**
     * 
     * @return Current X,Y velocity of robot in M/S as a Vector
     */
    public Translation2d velocityXY() {
        return new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    }

    public Translation2d accelerationXY() {
        return new Translation2d(robotAcceleration.vxMetersPerSecond, robotAcceleration.vyMetersPerSecond);
    }

    /**
     * 
     * @return Current Rotational Velocity of the robot in Rad/s
     */
    public double angularVelocity() {
        return Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Used interally for updated estimation
     * 
     * @param pose
     * @param speeds
     * @param time
     * @return
     */
    public SwerveDriveState update(Pose2d pose, ChassisSpeeds speeds, ChassisSpeeds accelerations, double time, double slippage) {
        this.timeSinceLastUpdate = time - this.timeStamp;
        if (timeSinceLastUpdate == 0) timeSinceLastUpdate = 1e-6;

        this.pose = pose;
        this.robotRelativeSpeeds = speeds;
        this.robotAcceleration = accelerations;
        this.timeStamp = time;

        this.trust += slippage;

        return this;
    }

    public static final SwerveDriveStateStruct struct = new SwerveDriveStateStruct();
    public static class SwerveDriveStateStruct implements Struct<SwerveDriveState> {
    @Override
    public Class<SwerveDriveState> getTypeClass() {
        return SwerveDriveState.class;
    }

    @Override
    public String getTypeName() {
        return "SwerveDriveState";
    }

    @Override
    public int getSize() {
        return Pose2d.struct.getSize() + ChassisSpeeds.struct.getSize() + ChassisSpeeds.struct.getSize() + Double.BYTES * 3;
    }

    @Override
    public String getSchema() {
        return "Pose2d pose;ChassisSpeeds velocity;ChassisSpeeds acceleration;double timestamp;double timeSinceLastUpdate;double trust";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {Pose2d.struct, ChassisSpeeds.struct};
    }

    @Override
    public SwerveDriveState unpack(ByteBuffer bb) {
        Pose2d pose = Pose2d.struct.unpack(bb);
        ChassisSpeeds velocity = ChassisSpeeds.struct.unpack(bb);
        ChassisSpeeds acceleration = ChassisSpeeds.struct.unpack(bb);
        double time = bb.getDouble();
        double timeSinceLastUpdate = bb.getDouble();
        double trust = bb.getDouble();
        return new SwerveDriveState(pose, velocity, acceleration, time, timeSinceLastUpdate, trust);
    }

    @Override
    public void pack(ByteBuffer bb, SwerveDriveState value) {
        Pose2d.struct.pack(bb, value.pose);
        ChassisSpeeds.struct.pack(bb, value.robotRelativeSpeeds);
        ChassisSpeeds.struct.pack(bb, value.robotAcceleration);
        bb.putDouble(value.timeStamp);
        bb.putDouble(value.timeSinceLastUpdate);
        bb.putDouble(value.trust);
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
    }
}
 

/**
 *  Potential Future work -> Incorporate Gyro Readings into Current Acceleration and Store Jerk, Might be cool
 *
 *  Add system for keeping track of confidence of current pose (1690). Overtime accumulate error from odometry -> reset down when getting vision readings ect.
 * 
 * Will help update vision faster and more reliably, don't need to trust shit vision poses when we have gotten a good one and barely moved! 
 */