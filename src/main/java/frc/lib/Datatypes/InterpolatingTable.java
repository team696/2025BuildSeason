package frc.team696.lib.Datatypes;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;
import frc.team696.lib.Util;

/**
 * Class For creating a table to interpolate between values automatically
 * 
 * <p> To use must create your custom class that implements Interpolatable to handle how the table will interpolate between nearest datapoints.
 */
public class InterpolatingTable<T extends Interpolatable<T>> {
    public final TreeMap<Double, T> map = new TreeMap<Double, T>();

    @SafeVarargs
    public InterpolatingTable(Map.Entry<Double, T>... entries) {
        for (Map.Entry<Double, T> entry : entries) {
            map.putIfAbsent(entry.getKey(), entry.getValue());
        }
    }
    
    /**
     * 
     * @param key Gets the value from the map at this point
     * @return Interpolated Value
     */
    public T getValue(double key) {
        double index = Util.clamp(key, map.firstKey() + 0.0000000001, map.lastKey() - 0.0000000001);
        Map.Entry<Double, T> lower = map.floorEntry(index);
        Map.Entry<Double, T> higher = map.ceilingEntry(index);

        double t = (index - lower.getKey()) / (higher.getKey() - lower.getKey());

        return lower.getValue().interpolate(higher.getValue(), t);
    }

}
