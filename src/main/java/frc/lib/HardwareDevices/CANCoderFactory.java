package frc.team696.lib.HardwareDevices;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.team696.lib.Logging.PLog;

/**
 * Class For creating and keeping track of cancoder device 
 * 
 * <p> Automatically logs name of the device if it fails to config
 * 
 * <p> Resets device parameters if it drops out
 */
public class CANCoderFactory {
    private final double TIMEOUT = 0.05;
    private CANcoder _encoder;
    private CANcoderConfiguration _config;
    private String _name;

    private boolean _configured = false;
    private double _lastConfiguration = -100;

    private Alert configurationAlert;

    public CANCoderFactory(int id, String canBus, CANcoderConfiguration config, String name) {
        this._encoder = new CANcoder(id, canBus);
        this._name = name;
        this._config = config;
        configurationAlert = new Alert(String.format("Failed to configure %s", this._name), AlertType.kError);
        configure();
    }

    public CANCoderFactory(int id, CANcoderConfiguration config, String name) {
        this(id, "rio", config, name);
    }

    private boolean configure() {
        return configure(false);
    }

    public boolean configure(boolean force) {
        if (!force && _configured) return true;
        if (!force && Timer.getFPGATimestamp() - _lastConfiguration < 3) return false;

        _lastConfiguration = Timer.getFPGATimestamp();
        StatusCode configCode = _encoder.getConfigurator().apply(this._config, TIMEOUT);

        if(configCode.isError()) {
            PLog.unusual(_name, "Failed to configure");
        } else {
            PLog.info(_name, "Configured");
            if (configCode.isWarning()) {
                PLog.unusual(_name, "Config Warning: " + configCode.toString());
            }

            _configured = true;
        }

        configurationAlert.set(!_configured);

        return _configured;
    }


    public Rotation2d getPosition() {
        if (configure()) {
            StatusSignal<Angle> positionCode = _encoder.getAbsolutePosition();
            if(!positionCode.getStatus().isOK()) {
                _configured = false;
                return new Rotation2d();
            }
            return Rotation2d.fromRotations(positionCode.getValueAsDouble());
        } else 
            return new Rotation2d();
    }


    public CANcoder get() {
        return _encoder;
    }
}
