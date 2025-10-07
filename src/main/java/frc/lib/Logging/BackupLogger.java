// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Logging;

import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * NOT DONE
 * 
 * 
 * Sick Logger that uses generics to fuck shit up. Who knows how performant it
 * is, who cares, it's 1 file!
 * 
 * May Switch to this internally -> less reliance on other libraries, more
 * freedom, doesn't hurt anything anyway
 * 
 * Really Meant as a backup/alternative to other loggers, use it if you want,
 * idgaf
 */
public class BackupLogger {
  public static DataLog log;

  public static final boolean PUBLISH_TO_NT = true;
  public static final boolean PUBLISH_TO_LOG = true;

  private static ConcurrentLinkedQueue<LogEntry<?>> logQueue = new ConcurrentLinkedQueue<LogEntry<?>>();

  private static final CANStatus status = new CANStatus();
  private static PowerDistribution pdh;

  private static logThread lThread;

  static {
    DataLogManager.start("/U/logs");

    log = DataLogManager.getLog();
    
    DataLogManager.logConsoleOutput(true);
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(log, true);

    lThread = new logThread();
    lThread.start();
  }

  public static void stop() {
    lThread.isRunning = false;
    DataLogManager.stop();

    for (LogEntry.GenericPublisher<?> p : LogEntry.publishers.values()) {
      p.publisher.close();
    }

    for (LogEntry.GenericDataLog<?> p : LogEntry.logEntries.values()) {
      p.dataLog.finish();
    }
  }

  public static void setPdh(PowerDistribution newPdh) {
    pdh = newPdh;
  }

  /**
   * Call Periodically If Desired
   */
  public static void logSystemInformation() {
    addToQueue("SystemStats/FPGAVersion", HALUtil.getFPGAVersion());
    addToQueue("SystemStats/FPGARevision", HALUtil.getFPGARevision());
    addToQueue("SystemStats/SerialNumber", HALUtil.getSerialNumber());
    addToQueue("SystemStats/Comments", HALUtil.getComments());
    addToQueue("SystemStats/TeamNumber", HALUtil.getTeamNumber());
    addToQueue("SystemStats/FPGAButton", HALUtil.getFPGAButton());
    addToQueue("SystemStats/SystemActive", HAL.getSystemActive());
    addToQueue("SystemStats/BrownedOut", HAL.getBrownedOut());
    addToQueue("SystemStats/RSLState", HAL.getRSLState());
    addToQueue("SystemStats/SystemTimeValid", HAL.getSystemTimeValid());
    addToQueue("SystemStats/BatteryVoltage", PowerJNI.getVinVoltage());
    addToQueue("SystemStats/BatteryCurrent", PowerJNI.getVinCurrent());
    addToQueue("SystemStats/3v3Rail/Voltage", PowerJNI.getUserVoltage3V3());
    addToQueue("SystemStats/3v3Rail/Current", PowerJNI.getUserCurrent3V3());
    addToQueue("SystemStats/3v3Rail/Active", PowerJNI.getUserActive3V3());
    addToQueue("SystemStats/3v3Rail/CurrentFaults", PowerJNI.getUserCurrentFaults3V3());
    addToQueue("SystemStats/5vRail/Voltage", PowerJNI.getUserVoltage5V());
    addToQueue("SystemStats/5vRail/Current", PowerJNI.getUserCurrent5V());
    addToQueue("SystemStats/5vRail/Active", PowerJNI.getUserActive5V());
    addToQueue("SystemStats/5vRail/CurrentFaults", PowerJNI.getUserCurrentFaults5V());
    addToQueue("SystemStats/6vRail/Voltage", PowerJNI.getUserVoltage6V());
    addToQueue("SystemStats/6vRail/Current", PowerJNI.getUserCurrent6V());
    addToQueue("SystemStats/6vRail/Active", PowerJNI.getUserActive6V());
    addToQueue("SystemStats/6vRail/CurrentFaults", PowerJNI.getUserCurrentFaults6V());
    addToQueue("SystemStats/BrownoutVoltage", PowerJNI.getBrownoutVoltage());
    addToQueue("SystemStats/CPUTempCelcius", PowerJNI.getCPUTemp());

    CANJNI.getCANStatus(status);
    addToQueue("SystemStats/CANBus/Utilization", status.percentBusUtilization);
    addToQueue("SystemStats/CANBus/OffCount", status.busOffCount);
    addToQueue("SystemStats/CANBus/TxFullCount", status.txFullCount);
    addToQueue("SystemStats/CANBus/ReceiveErrorCount", status.receiveErrorCount);
    addToQueue("SystemStats/CANBus/TransmitErrorCount", status.transmitErrorCount);
    addToQueue("SystemStats/EpochTimeMicros", HALUtil.getFPGATime());

    if (pdh == null)
      return;

    addToQueue("SystemStats/PowerDistribution/Temperature", pdh.getTemperature());
    addToQueue("SystemStats/PowerDistribution/Voltage", pdh.getVoltage());
    addToQueue("SystemStats/PowerDistribution/ChannelCurrent", pdh.getAllCurrents());
    addToQueue("SystemStats/PowerDistribution/TotalCurrent", pdh.getTotalCurrent());
    addToQueue("SystemStats/PowerDistribution/TotalPower", pdh.getTotalPower());
    addToQueue("SystemStats/PowerDistribution/TotalEnergy", pdh.getTotalEnergy());
    addToQueue("SystemStats/PowerDistribution/ChannelCount", pdh.getNumChannels());
  }

  public static <T> void addToQueue(String name, T value) {
    if (lThread.isRunning)
      logQueue.add(new LogEntry<T>(name, value));
  }

  public static class logThread extends Thread {
    public logThread() {
      this.setName("LoggerThread");
      this.setDaemon(true);
      this.setPriority(MIN_PRIORITY);
    }

    public volatile boolean isRunning = true;

    @Override
    public void run() {
      while (isRunning) {
        LogEntry<?> entry = BackupLogger.logQueue.poll();
        if (entry == null)
          continue;
        try {
          if (BackupLogger.PUBLISH_TO_NT) {
            LogEntry.GenericPublisher.classToPublisher.get(entry.value.getClass())
                .invoke(LogEntry.publishers.get(entry.name).publisher, entry.value);
          }
          if (BackupLogger.PUBLISH_TO_LOG) {
            LogEntry.GenericDataLog.classToDatalog.get(entry.value.getClass())
                .invoke(LogEntry.logEntries.get(entry.name).dataLog, entry.value);
          }
        } catch (Exception e) {
          PLog.info("Logger", "Publisher Failed to accept value: " + entry.name);
        }
      }
    }
  }
}
