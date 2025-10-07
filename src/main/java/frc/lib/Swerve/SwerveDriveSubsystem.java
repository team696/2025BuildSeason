package frc.team696.lib.Swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team696.lib.Util;
import frc.team696.lib.HardwareDevices.PigeonFactory;
import frc.team696.lib.Logging.BackupLogger;

public abstract class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModulePosition[] _swervePositions = new SwerveModulePosition[4];
  private final SwerveDrivePoseEstimator _poseEstimator;

  protected final SwerveModule[] _modules;
  protected final SwerveDriveKinematics _kinematics;

  protected final PigeonFactory _pigeon;

  protected Rotation2d yawOffset = new Rotation2d(0);

  protected final ReadWriteLock _stateLock;
  protected final SwerveDriveState _cachedState;

  private final odometryThread _odometryThread;

  private SwerveModuleState[] swerveModuleDesiredStates = { new SwerveModuleState(), new SwerveModuleState(),
      new SwerveModuleState(), new SwerveModuleState() };

  public SwerveDriveSubsystem() {
    this._stateLock = new ReentrantReadWriteLock();
    this._cachedState = new SwerveDriveState();

    SwerveModule frontLeft = new SwerveModule(SwerveConfigs.FRONT_LEFT);
    SwerveModule frontRight = new SwerveModule(SwerveConfigs.FRONT_RIGHT);
    SwerveModule backLeft = new SwerveModule(SwerveConfigs.BACK_LEFT);
    SwerveModule backRight = new SwerveModule(SwerveConfigs.BACK_RIGHT);
    _modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

    _kinematics = new SwerveDriveKinematics(SwerveConstants.modPositions);

    for (int i = 0; i < 4; ++i) {
      _swervePositions[i] = _modules[i].getPosition();
    }

    _pigeon = new PigeonFactory(0, SwerveConstants.canBus, SwerveConfigs.pigeon, "Pigeon");

    _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getYaw(), _swervePositions,
        new Pose2d(0, 0, new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6));

    zeroYaw();

    _odometryThread = new odometryThread(this);
    _odometryThread.start();
  }

  /**
   * 
   * @return Current Estimated State Of The Robot
   * 
   * @see SwerveDriveState.java
   */
  public SwerveDriveState getState() {
    try {
      this._stateLock.readLock().lock();
      return this._cachedState;
    } finally {
      this._stateLock.readLock().unlock();
    }
  }

  /**
   * 
   * @return Current Estimated Pose2d Of The Robot
   */
  public Pose2d getPose() {
    return getState().pose;
  }

  /**
   * Resets The Pose2d Of The Robot To newPose
   * 
   * @param newPose new Pose2d Of The Robot
   */
  public void resetPose(Pose2d newPose) {
    try {
      this._stateLock.writeLock().lock();
      _poseEstimator.resetPosition(getYaw(), _swervePositions, newPose);
    } finally {
      this._stateLock.writeLock().unlock();
    }
  }

  /**
   * Resets The Pose Of the Robot to 0,0,0
   */
  public void resetPose() {
    resetPose(new Pose2d());
  }

  /**
   * Updated The Yaw Offset Of The Robot
   * <p>
   * Used To 0 out the Gyro When Driving
   */
  public void updateYawOffset() {
    yawOffset = getPose().getRotation().minus(getYaw());
  }

  /**
   * Periodically called 50 Hz
   * <p>
   * Replaces the regular periodic function
   */
  public abstract void onUpdate();

  /**
   * 
   * @return Current Yaw Of the Gyro
   * 
   * @see latencyAdjustedYaw()
   */
  public Rotation2d getYaw() {
    return _pigeon.getYaw();
  }

  /**
   * 
   * @return Latency Adjusted Yaw Of the Gyro
   */
  public Rotation2d latencyAdjustedYaw() {
    return _pigeon.getLatencyAdjustedYaw();
  }

  /**
   * Zeros the gyro
   */
  public void zeroYaw() {
    yawOffset = getYaw();
    resetPose(new Pose2d(getPose().getTranslation(),
        Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180 : 0)));
  }

  /**
   * 
   * @return Robot Relatives Speeds
   * 
   *         <ul>
   *         <li>x is forward</li>
   *         <li>y is left</li>
   *         <li>r is counterclock wise</li>
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getState().robotRelativeSpeeds;
  }

  /**
   * 
   * @return Array Of Modules States
   */
  protected SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : _modules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * 
   * @return Array of Module Positions
   */
  protected SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : _modules) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  /**
   * Used To drive the robot during teleop
   * 
   * @param translation   in X, Y
   * @param rotation      in R
   * @param fieldRelative should forward be based on the field or robot
   * @param isOpenLoop    closed or open loop control of the drive wheels.
   *                      Typically openLoop for teleop and closed for autonomous
   */
  public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds desiredRobotSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    if (fieldRelative)
      desiredRobotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(desiredRobotSpeeds,
          getYaw().plus(yawOffset).rotateBy(Rotation2d.fromDegrees((Util.getAlliance() == Alliance.Red ? 180 : 0))));

    SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(desiredRobotSpeeds);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Used to drive during auto
   * 
   * @param c Chassis Speeds object containing the desired speeds of the robot
   */
  public void Drive(ChassisSpeeds c) {
    SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(c);
    setModuleStates(swerveModuleStates);
  }

  public void voltageDriveForward(Voltage volts) {
    for (SwerveModule mod : _modules)
      mod.setDesiredVoltageForward(volts);
  }

  public void voltageRotateForward(Voltage volts) {
    for (SwerveModule mod : _modules)
      mod.setDesiredVoltageRotation(volts);
  }

  /**
   * Stops driving
   */
  public void doNothing() {
    Drive(new ChassisSpeeds());
  }

  /**
   * @param desiredStates the desired states of each module, in the order of the
   *                      kinematics (FL, FR, BL, BR)
   * @param openLoop      Open or closed loop control
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_VELOCITY);

    for (SwerveModule mod : _modules)
      mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);

    this.swerveModuleDesiredStates = desiredStates;
  }

  /**
   * Using closed loop control
   * 
   * @param desiredStates the desired states of each module, in the order of the
   *                      kinematics (FL, FR, BL, BR)
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates, false);
  }

  /**
   * 
   * @return List Of Swerve Modules
   */
  public SwerveModule[] getModules() {
    return _modules;
  }

  /**
   * 
   * @param position Position for distance from
   * @return distance from position argument
   */
  public double distTo(Translation2d position) {
    return getPose().getTranslation().getDistance(position);
  }

  /**
   * 
   * @param position Position for distance from
   * @return distance from position argument
   */
  public double distTo(Pose2d position) {
    return distTo(position.getTranslation());
  }

  /**
   * 
   * @param position Position for angle to
   * @return Angle to Position
   */
  public Rotation2d angleTo(Translation2d position) {
    Translation2d delta = getPose().getTranslation().minus(position);
    Rotation2d rot = Rotation2d.fromRadians(Math.atan2(delta.getY(), delta.getX()));
    return rot;
  }

  /**
   * 
   * @param position Position for angle to
   * @return Angle to Position
   */
  public Rotation2d angleTo(Pose2d position) {
    return angleTo(position.getTranslation());
  }

  /**
   * Don't override periodic, override onUpdate()
   * 
   */
  @Override
  public final void periodic() {
    if (DriverStation.isDisabled()) {
      this.updateYawOffset();
    }

    BackupLogger.addToQueue("Swerve/ModuleStates", getModuleStates());

    BackupLogger.addToQueue("Swerve/DesiredModuleStates", swerveModuleDesiredStates);

    BackupLogger.addToQueue("Swerve/RobotState", getState());

    onUpdate();
  }

  /**
   * Adds a vision measurement to the current estimation
   * 
   * @param visionPose      Vision Pose
   * @param visionTimestamp Vision Timestamp
   * @param stdDeviations   Standard Deviations of vision measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double visionTimestamp, Vector<N3> stdDeviations) {
    try {
      this._stateLock.writeLock().lock();

      _poseEstimator.addVisionMeasurement(visionPose, visionTimestamp, stdDeviations);
    } finally {
      this._stateLock.writeLock().unlock();
    }
  }

  class odometryThread extends Thread {
    private SwerveDriveSubsystem this0;

    private BaseStatusSignal[] _allSignals;

    odometryThread(SwerveDriveSubsystem this0) {
      this.this0 = this0;

      this.setName("OdometryThread");
      this.setDaemon(true);
      this.setPriority(MIN_PRIORITY + 1); // Slightly Above Minimum
    }

    public void run() {
      _allSignals = new BaseStatusSignal[this0._modules.length * SwerveModule.SIGNAL_COUNT + 2 + 3];
      for (SwerveModule mod : this0._modules) {

        BaseStatusSignal[] signals = mod.getSignals();

        for (int i = 0; i < SwerveModule.SIGNAL_COUNT; i++) {
          _allSignals[mod.moduleNumber * SwerveModule.SIGNAL_COUNT + i] = signals[i];
        }
      }
      _allSignals[_allSignals.length - 2] = this0._pigeon._yawSignal;
      _allSignals[_allSignals.length - 1] = this0._pigeon._yawVelocitySignal;

      _allSignals[_allSignals.length - 5] = this0._pigeon.get().getAccelerationX();
      _allSignals[_allSignals.length - 4] = this0._pigeon.get().getAccelerationY();
      _allSignals[_allSignals.length - 3] = this0._pigeon.get().getAccelerationZ();

      BaseStatusSignal.setUpdateFrequencyForAll(250, _allSignals);
      ChassisSpeeds acceleration = new ChassisSpeeds();

      while (true) {
        try {
          BaseStatusSignal.refreshAll(_allSignals);

          this.this0._stateLock.writeLock().lock();

          for (int i = 0; i < 4; ++i) {
            _swervePositions[i] = _modules[i].getPosition();
          }

          ChassisSpeeds speeds = _kinematics.toChassisSpeeds(getModuleStates());

          double time = System.currentTimeMillis();
          double timeSinceLastUpdate = time - _cachedState.timeStamp;
          if (timeSinceLastUpdate == 0)
            timeSinceLastUpdate = 1e-9;

          Pose2d newPose = _poseEstimator.updateWithTime(Utils.getCurrentTimeSeconds(), latencyAdjustedYaw(),
              _swervePositions);

          double slippage = Math.abs(speeds.omegaRadiansPerSecond - _pigeon.getAngularVelocity().in(RadiansPerSecond))
              * Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                  + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);

          double measuredSensorAcceleration = Math
              .sqrt(Math.pow(_allSignals[_allSignals.length - 3].getValueAsDouble(), 2)
                  + Math.pow(_allSignals[_allSignals.length - 4].getValueAsDouble(), 2)
                  + Math.pow(_allSignals[_allSignals.length - 5].getValueAsDouble(), 2));

          acceleration.vxMetersPerSecond = (speeds.vxMetersPerSecond
              - _cachedState.robotRelativeSpeeds.vxMetersPerSecond) / (timeSinceLastUpdate / 1000);
          acceleration.vyMetersPerSecond = (speeds.vyMetersPerSecond
              - _cachedState.robotRelativeSpeeds.vyMetersPerSecond) / (timeSinceLastUpdate / 1000);
          acceleration.omegaRadiansPerSecond = (speeds.omegaRadiansPerSecond
              - _cachedState.robotRelativeSpeeds.omegaRadiansPerSecond) / (timeSinceLastUpdate / 1000);

          double accelerationMagnitude = acceleration.vxMetersPerSecond * acceleration.vxMetersPerSecond
              + acceleration.vyMetersPerSecond * acceleration.vyMetersPerSecond;

          BackupLogger.addToQueue("sensorAcceleration",
              Units.Gs.of(measuredSensorAcceleration - 1).in(Units.MetersPerSecondPerSecond));
          BackupLogger.addToQueue("acceleration", accelerationMagnitude);

          this.this0._cachedState.update(
              newPose,
              speeds,
              acceleration,
              time,
              slippage);
        } finally {
          this.this0._stateLock.writeLock().unlock();
        }
        Timer.delay(1.0 / 250.0); // Limits To 250 Hz
      }
    }
  }
}
