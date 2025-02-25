// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class DynamicNTDouble {
    String name;
    double val;
    DoubleEntry ntEntry;
    DoubleConsumer update;
    public static Map<String, DynamicNTDouble> avail=new HashMap<String, DynamicNTDouble>();
    public static void periodic(){
        for(DynamicNTDouble v:avail.values()){
            double prev=v.val;
            v.val=v.ntEntry.get(0);
            if(v.val!=prev){
                v.update.accept(v.val);
            } 
            
        }
    }
    public void Register(){
        if(avail.containsKey(name)){
            throw new IllegalArgumentException("DynamicNTDouble with name "+name+" already exists");
        }
        avail.put(name, this);
    }
    public DynamicNTDouble(String name, double val, DoubleConsumer update){
        this.name=name;
        this.val=val;
        this.update=update;
        this.ntEntry=NetworkTableInstance.getDefault().getDoubleTopic(name).getEntry(0);
        this.ntEntry.set(val);
    }
}
