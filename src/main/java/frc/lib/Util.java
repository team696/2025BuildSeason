package frc.team696.lib;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team696.lib.Logging.PLog;

/**
 *  Useful functions which don't fit anywhere specific
 */
public class Util {

    /**
     * Linear Interpolates between two numbers
     * 
     * @param t Value from [0,1]
     * @param min Lower value --> 0
     * @param max Upper value --> 1
     * @return Interpolated value between min and max
     */
    public static double lerp(double t, double min, double max) {
        return (max - min) * t + min;
    }

    /**
     * @param val Value to be clamped
     * @param min Minimum Value
     * @param max Maximum Value
     * @return Value Clamped between min and max
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(Math.min(max, val), min);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * @return Current Alliance through driverstation or FMS. Defaults to Blue.
     */
    public static Alliance getAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get();
        } else {
            return Alliance.Blue;
        }
    }

    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
    }

    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

    /**
     * 
     * @return List of connected MAC Addresses
     * @throws IOException
     */
    public static List<byte[]> getMacAddresses() throws IOException {
		List<byte[]> macAddresses = new ArrayList<>();

		Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
		NetworkInterface networkInterface;
		while (networkInterfaces.hasMoreElements()) {
			networkInterface = networkInterfaces.nextElement();

			byte[] address = networkInterface.getHardwareAddress();
			if (address == null) {
				continue;
			}

			macAddresses.add(address);
		}
		return macAddresses;
	}

    public static String macToString(byte[] address) {
		StringBuilder builder = new StringBuilder();
		for (int i = 0; i < address.length; i++) {
			if (i != 0) {
				builder.append(':');
			}
			builder.append(String.format("%02X", address[i]));
		}
		return builder.toString();
	}

    /** Returns the current detected robot based on map of robot name to mac address
     * <p>
     *  <ul>
     *  <li> -1 is simulation </li>
     *  <li> 0 is unknown </li>
     *  <li> 1 is first item in map... etc. </li>
     *  </ul>
     * </p>
     *  verbose will print to console, which robot connected, or the MAC if unknown
     * 
     * <p>
     *  ex: (RobotInit()):  Util.setRobotType(new LinkedHashMap<>() { }, true);
     */
    public static int setRobotType(LinkedHashMap<String, byte[]> nameToMac, boolean verbose) {
        if (RobotBase.isSimulation()) {
            if (verbose) {
                PLog.info("Robot", "Simulation Detected");
            }
            return -1;
        }

        List<byte[]> macAddresses;
		try {
			macAddresses = Util.getMacAddresses();
		} catch (IOException e) {
            PLog.fatalException("Robot", "Mac Address Attempt Unsuccessful", e);
			macAddresses = List.of();
            return 0;
		}

        for (byte[] macAddress : macAddresses) {
            int i = 1;
            for (Map.Entry<String, byte[]> robotEntry : nameToMac.entrySet()) {
                if (Arrays.compare(robotEntry.getValue(), macAddress) == 0) {
                    if (verbose) {
                        PLog.info("Robot", String.format("%s Detected", robotEntry.getKey()));
                    }
                    return i;
                }
                i++;
            }
        }

		if (verbose) {
            PLog.info("Robot", "Unknown Robot Detected");
            for (byte[] macAddress : macAddresses) {
                PLog.info("    ", Util.macToString(macAddress));
            }		
        }     
        return 0;
    }
    public static <K, V, R> Map<K, R> transformMap(Map<K, V> originalMap, Function<V, R> transformer) {
        Map<K, R> resultMap = new HashMap<>();
        for (Map.Entry<K, V> entry : originalMap.entrySet()) {
            resultMap.put(entry.getKey(), transformer.apply(entry.getValue()));
        }
        return resultMap;
    }
    
    public static <K, K2, V, R> Map<K, Map<K2, R>> transformNestedMap(
            Map<K, Map<K2, V>> originalMap, Function<V, R> transformer) {
        Map<K, Map<K2, R>> resultMap = new HashMap<>();
        
        for (Map.Entry<K, Map<K2, V>> outerEntry : originalMap.entrySet()) {
            Map<K2, R> innerResultMap = new HashMap<>();
            for (Map.Entry<K2, V> innerEntry : outerEntry.getValue().entrySet()) {
                innerResultMap.put(innerEntry.getKey(), transformer.apply(innerEntry.getValue()));
            }
            resultMap.put(outerEntry.getKey(), innerResultMap);
        }
        
        return resultMap;
    }
}
