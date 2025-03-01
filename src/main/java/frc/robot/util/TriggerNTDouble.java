// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TriggerNTDouble {
    String name;
    double val;
    Trigger trigger;
    DoubleEntry ntEntry;
    DoubleConsumer update;

    private boolean hasChanged() {
        double prev = val;
        val = ntEntry.getAsDouble();
        return prev != val;
    }

    public TriggerNTDouble(String name, double val, DoubleConsumer update) {
        this.name = name;
        this.val = val;
        this.update = update;
        this.ntEntry = NetworkTableInstance.getDefault().getDoubleTopic(name).getEntry(0);
        this.ntEntry.set(val);
        trigger = new Trigger(() -> hasChanged());
        trigger.onTrue(new InstantCommand(() -> update.accept(this.val)).ignoringDisable(true));
    }
}
