package frc.team696.lib.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.team696.lib.Util;
import frc.team696.lib.HardwareDevices.CANCoderFactory;
import frc.team696.lib.HardwareDevices.TalonFactory;

public class SwerveModule{
    public static int s_moduleCount = 0;
    public static final String[] moduleNames = {"FL", "FR", "BL", "BR"}; 

    private final StatusSignal<Angle> _encoderSignal;
    private final StatusSignal<Angle> _anglePositionSignal;
    private final StatusSignal<AngularVelocity> _angleVelocitySignal;
    private final StatusSignal<Angle> _drivePositionSignal;
    private final StatusSignal<AngularVelocity> _driveVelocitySignal;

    public static final int SIGNAL_COUNT = 4; 

    public final int moduleNumber;
    private Rotation2d _angleOffset;
    
    private TalonFactory _angleMotor;
    private TalonFactory _driveMotor;
    private CANCoderFactory _encoder;

    private Rotation2d _lastAngle;

    private final SimpleMotorFeedforward _driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.drivekS.in(Volts), SwerveConstants.drivekV.in(PerUnit.combine(Volts, MetersPerSecond)), SwerveConstants.drivekA.in(PerUnit.combine(Volts,MetersPerSecondPerSecond)));
    
    private final VelocityVoltage _driveVelocity = new VelocityVoltage(0);

    private final PositionVoltage _anglePosition = new PositionVoltage(0);

    private final double[] _sysidRotationPositions = new double[] {135, -135, -45, 45};


    public SwerveModule(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstants){
        this.moduleNumber = s_moduleCount++;
        this._angleOffset = Rotation2d.fromRotations(moduleConstants.EncoderOffset); 
        /* Angle Encoder Config */
        _encoder = new CANCoderFactory(moduleConstants.EncoderId, SwerveConstants.canBus, SwerveConfigs.canCoder, "Swerve Encoder " + moduleNumber);

        _encoderSignal = ( _encoder.get().getAbsolutePosition()); 

        /* Angle Motor Config */
        _angleMotor = new TalonFactory(moduleConstants.SteerMotorId, SwerveConstants.canBus, SwerveConfigs.angle, "Swerve Angle " + moduleNumber);
        resetToAbsolute();

        /* Drive Motor Config */
        _driveMotor = new TalonFactory(moduleConstants.DriveMotorId, SwerveConstants.canBus, SwerveConfigs.drive, "Swerve Drive " + moduleNumber);

        _anglePositionSignal = (_angleMotor.get().getPosition());
        _angleVelocitySignal = (_angleMotor.get().getVelocity());
        _drivePositionSignal = (_driveMotor.get().getPosition());
        _driveVelocitySignal = (_driveMotor.get().getVelocity());
  
        BaseStatusSignal.setUpdateFrequencyForAll(100, _encoderSignal, _anglePositionSignal, _angleVelocitySignal, _drivePositionSignal, _driveVelocitySignal);

        ParentDevice.optimizeBusUtilizationForAll(_encoder.get(), _angleMotor.get(), _driveMotor.get());

        _lastAngle = getState().angle;
    }
    
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState.optimize(getState().angle);
        desiredState.cosineScale(getState().angle); 
        Rotation2d angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond) * 0.01)) ? _lastAngle : desiredState.angle);
        _angleMotor.setControl(_anglePosition.withPosition(angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
        _lastAngle = angle;
    }

    // For Use In SysID
    public void setDesiredVoltageForward(Voltage volts) {
        _driveMotor.VoltageOut(volts);
        _angleMotor.setControl(_anglePosition.withPosition(0));
    }
    public void setDesiredVoltageRotation(Voltage volts) {
        _driveMotor.VoltageOut(volts);
        _angleMotor.setControl(_anglePosition.withPosition(_sysidRotationPositions[moduleNumber]));
    }
    public Voltage getDriveOutputVoltage() {
        return Volts.of(_driveMotor.get().get() * RobotController.getBatteryVoltage());
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            _driveMotor.VoltageOut(Volts.of(desiredState.speedMetersPerSecond / SwerveConstants.MAX_VELOCITY.in(Units.MetersPerSecond)*12));
        } else {
            _driveVelocity.Velocity = Util.MPSToRPS(desiredState.speedMetersPerSecond, SwerveConstants.WHEEL_CIRCUM.in(Units.Meters));
            _driveVelocity.FeedForward = _driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            _driveMotor.setControl(_driveVelocity);
        }
    }

    public Rotation2d getCANCoderAngle(boolean refresh){
        if (refresh) {
            _encoderSignal.refresh();
        }

        return Rotation2d.fromRotations(_encoderSignal.getValueAsDouble());
    }

    public Rotation2d getCANCoderAngle(){
        return getCANCoderAngle(true);
    }

    public void resetToAbsolute(){
        Rotation2d absolutePosition = getCANCoderAngle(true).minus(_angleOffset);
        _angleMotor.setPosition(absolutePosition.getRotations());
    }

    public SwerveModuleState getState(boolean refresh){
        if (refresh) {
            _driveVelocitySignal.refresh();
        }
        return new SwerveModuleState(
            Util.RPSToMPS(_driveVelocitySignal.getValueAsDouble(), SwerveConstants.WHEEL_CIRCUM.in(Units.Meters)), 
            Rotation2d.fromRotations(getAngleMotorPosition(refresh).in(Rotations))
        );
    }

    public SwerveModuleState getState(){
        return getState(false);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Util.rotationsToMeters(getDriveMotorPosition().in(Rotations), SwerveConstants.WHEEL_CIRCUM.in(Units.Meters)), 
            Rotation2d.fromRotations(getAngleMotorPosition().in(Rotations))
        );
    }

    public Angle getDriveMotorPosition(boolean refresh) {
        if (refresh) {
            _drivePositionSignal.refresh();
            _driveVelocitySignal.refresh();
        }
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValue(_drivePositionSignal, _driveVelocitySignal).magnitude());
    }

    public Angle getDriveMotorPosition() {
        return getDriveMotorPosition(false);
    }

    public Angle getAngleMotorPosition(boolean refresh) {
        if (refresh) {
            _anglePositionSignal.refresh();
            _angleVelocitySignal.refresh();
        }
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValue(_anglePositionSignal, _angleVelocitySignal).magnitude());
    }

    public Angle getAngleMotorPosition() {
        return getAngleMotorPosition(false);
    }

    public BaseStatusSignal[] getSignals(){
        return new BaseStatusSignal[] {_anglePositionSignal, _angleVelocitySignal, _drivePositionSignal, _driveVelocitySignal, _encoderSignal};
    }
}