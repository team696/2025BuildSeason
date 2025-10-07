package frc.team696.lib.HardwareDevices;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.team696.lib.Logging.PLog;

/**
 * Class For creating and keeping track of talon device 
 * 
 * <p> Automatically logs name of the device if it fails to config
 * 
 * <p> Resets device parameters if it drops out
 */
public class TalonFactory { 
    private final double TIMEOUT = 0.05;
    private TalonFX _motor;
    private TalonFXConfiguration _config;
    private String _name;
    private DutyCycleOut _DutyCycleControl;
    private VoltageOut _VoltageControl;

    private int _followID = -1;
    private boolean _opposeDirection = false;

    private boolean _configured = false;
    private double _lastConfiguration = -100;

    private Alert configurationAlert;

    public TalonFactory(int id, String canBus, TalonFXConfiguration config, String name) {
        this._motor = new TalonFX(id, canBus);
        this._name = name;
        this._config = config;
        _DutyCycleControl = new DutyCycleOut(0);
        _VoltageControl = new VoltageOut(0);
        configurationAlert = new Alert(String.format("Failed to configure %s", this._name), AlertType.kError);
        configure();
    }

    public TalonFactory(int id, CANBus canBus, TalonFXConfiguration config, String name) {
        this._motor = new TalonFX(id, canBus);
        this._name = name;
        this._config = config;
        _DutyCycleControl = new DutyCycleOut(0);
        _VoltageControl = new VoltageOut(0);
        configurationAlert = new Alert(String.format("Failed to configure %s", this._name), AlertType.kError);
        configure();
    }

    public TalonFactory(int id, TalonFXConfiguration config, String name) {
        this(id, "rio", config, name);
    }

    private boolean configure() {
        return configure(false);
    }

    public boolean configure(boolean force) {
        if (!force && _configured) return true;
        if (!force && Timer.getFPGATimestamp() - _lastConfiguration < 3) return false;

        _lastConfiguration = Timer.getFPGATimestamp();
        StatusCode configCode = _motor.getConfigurator().apply(this._config, TIMEOUT);

        if(configCode.isError()) {
            PLog.unusual(_name, "Failed to configure");
        } else {
            PLog.info(_name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(_name, "Config Warning: " + configCode.toString());
            }

            if (_followID != -1)
                _motor.setControl(new Follower(_followID, _opposeDirection));

            setPosition(0);

            _configured = true;
        }

        configurationAlert.set(!_configured);

        return _configured;
    }

    public void Follow(int id, boolean opposeDirection) {
        _followID = id;
        this._opposeDirection = opposeDirection;
        if (configure())
            _motor.setControl(new Follower(id, opposeDirection)); 
    }

    public void Follow(TalonFX master, boolean opposeDirection) {
        Follow(master.getDeviceID(), opposeDirection);
    }

    public void Follow(TalonFactory master, boolean opposeDirection) {
        Follow(master.getID(), opposeDirection);
    }

    public int getID() {
        return _motor.getDeviceID();
    }
    
    public String getName(){
        return _name;
    }

    public void setPosition(double newPosition) {
        if (configure())
            if(!_motor.setPosition(newPosition).isOK()) {
                PLog.info(this._name, "Failed To Set Position");
                _configured = false;
            }
    }

    public void setPosition() {
        setPosition(0);
    }

    public double getPosition() {
        if (configure()) {
            StatusSignal<Angle> positionCode = _motor.getPosition();
            if(!positionCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return positionCode.getValueAsDouble();
        } else 
            return 0;
    }

    public double getVelocity() {
        if (configure()) {
            StatusSignal<AngularVelocity> velocityCode = _motor.getVelocity();
            if(!velocityCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return velocityCode.getValueAsDouble();
        } else 
            return 0;
    }

    public void PercentOutput(double percent) {
        setControl(_DutyCycleControl.withOutput(percent));
    }

    /** @param voltage 0-12 input */
    public void VoltageOut(Voltage voltage) {
        setControl(_VoltageControl.withOutput(voltage.in(Volts)));
    }

    public void stop() {
        _motor.stopMotor();
    }
    
    public void setControl(ControlRequest request) {
        if (configure())
            if (!_motor.setControl(request).isOK()) {
                _configured = false;
            }
    }

    public double getCurrent() {
        if (configure()) {
            StatusSignal<Current> currentCode = _motor.getStatorCurrent();
            if(!currentCode.getStatus().isOK()) {
                _configured = false;
                return 0;
            }
            return currentCode.getValueAsDouble();
        } else 
            return 0;
    }

    public TalonFX get() {
        return _motor;
    }
}
