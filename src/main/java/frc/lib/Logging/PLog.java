package frc.team696.lib.Logging;

import edu.wpi.first.wpilibj.DriverStation;

/** Class for printing out to the console */
public class PLog {
    /**
	 * Log a FATAL error, after which the robot cannot (properly) function. <br>
	 * 
	 * @param category
	 * @param message
	 */
	public static <T> void fatal(String category, T message) {
		log("Fatal", category, String.valueOf(message));

		// make it show up on the DS as well
		DriverStation.reportError("Fatal Error: " + String.valueOf(message), true);
	}

	/**
	 * Log a FATAL error due to an exception, after which the robot cannot
	 * (properly) function. <br>
	 * Prints your message, and the exception's name, message, and stacktrace.
	 * 
	 */
	public static void fatalException(String category, String userMessage, Exception exception) {
		String exceptionMessage = String.format("%s -- %s: %s", userMessage, exception.getClass().getSimpleName(), exception.getMessage());
        for (StackTraceElement element : exception.getStackTrace()) {
            exceptionMessage += "\n    " + element;
        }
		log("Fatal", category, exceptionMessage);
		
	}

    public static void fatalException(String category, String userMessage, StackTraceElement[] elements) {
		String exceptionMessage = String.format("%s ", userMessage);
        for (StackTraceElement element : elements) {
            exceptionMessage += "\n    " + element;
        }
		log("Fatal", category, exceptionMessage);
	}

	/**
	 * Log a failure which may kill one function or one thread, however the robot as
	 * a whole can keep functioning.
	 * 
	 * @param category
	 * @param message
	 */
	public static <T> void recoverable(String category, T message) {
		log("Recoverable", category, String.valueOf(message));

		DriverStation.reportError("Error: " + String.valueOf(message), true);
	}

	/**
	 * Log something which should not happen under normal circumstances and probably
	 * is a bug, but does not cause anything to crash.
	 * 
	 * @param category
	 * @param message
	 */
	public static <T> void unusual(String category, T message) {
		log("Unusual", category, String.valueOf(message));
	}

	/**
	 * Log a semi-important message which the user should probably see, but does not
	 * indicate anything is broken.
	 */
	public static <T> void info(String category, T message) {
		log("Info", category, String.valueOf(message));
	}

	/**
	 * Log a message which is not important during normal operation, but is useful
	 * if you're trying to debug the robot.
	 * 
	 * @param category
	 * @param message 
	 */
	public static <T> void debug(String category, T message) {
		log("Debug", category, String.valueOf(message));
	}

	private static void log(String severity, String category, String message) {
		String output = String.format("[%s] [%s] %s", severity, category, message);
		System.out.println(output);
		BackupLogger.addToQueue("prints", output);
	}
}
