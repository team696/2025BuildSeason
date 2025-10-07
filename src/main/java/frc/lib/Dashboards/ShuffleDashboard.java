package frc.team696.lib.Dashboards;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ShuffleDashboard {
    private static ShuffleboardTab _mainTab; 

    public static final Field2d field = new Field2d();

    static {
        _mainTab = Shuffleboard.getTab("Telemetry");

        _mainTab.add(field).withPosition(0, 0).withSize(7, 6);
    }

    public static ComplexWidget addObject(Sendable object) {
        return _mainTab.add(object);
    }

    public static ComplexWidget addObject(String name, Sendable object) {
        return _mainTab.add(name, object);
    }

    public static ShuffleboardTab Tab() {
        return _mainTab;
    }

    public static void addAutoChooser(Sendable chooser) {
        _mainTab.add("Autos", chooser).withPosition(7, 0).withSize(2,1);
    }
}
