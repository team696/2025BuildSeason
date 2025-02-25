// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase { 
  private static LED m_LED=null;
  private final int _ledOffset = 8;
  private final int _numLed = 36 + 8;
  public static synchronized LED get(){
    if(m_LED==null){
      m_LED=new LED();
    }
    return m_LED;
  }
  CANdle candle=new CANdle(0);

  public double getVbat() { return candle.getBusVoltage(); }
  public double get5V() { return candle.get5VRailVoltage(); }
  public double getCurrent() { return candle.getCurrent(); }
  public double getTemperature() { return candle.getTemperature(); }

  private void setColor(int r, int g, int b) {
    candle.clearAnimation(0);

    candle.setLEDs(r,g,b, 255, _ledOffset, _numLed);
  }

  private Command Color(int r, int g, int b){
    return this.startEnd(()->setColor(r,g,b), ()->setColor(0,0,0));
  }
  /** Creates a new LED. */
  private LED() {
    CANdleConfiguration _candleConfiguration = new CANdleConfiguration();
    _candleConfiguration.statusLedOffWhenActive = true;
    _candleConfiguration.disableWhenLOS = false;
    _candleConfiguration.stripType = LEDStripType.RGB;
    _candleConfiguration.brightnessScalar = 1;
    _candleConfiguration.vBatOutputMode = VBatOutputMode.On;
    _candleConfiguration.enableOptimizations = true;
    _candleConfiguration.v5Enabled = true;
    candle.configAllSettings(_candleConfiguration);
    this.setDefaultCommand(Color(255,0,0).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
