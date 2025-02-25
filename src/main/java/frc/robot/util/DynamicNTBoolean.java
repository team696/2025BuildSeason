// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;

/** Add your docs here. */
public class DynamicNTBoolean {
    String name;
    boolean val;
    BooleanEntry ntEntry;
    BooleanConsumer update;
    public static Map<String, DynamicNTBoolean> avail=new HashMap<String, DynamicNTBoolean>();
    public static void periodic(){
        for(DynamicNTBoolean v:avail.values()){
            boolean prev=v.val;
            v.val=v.ntEntry.get(false);
            if(v.val!=prev){
                v.update.accept(v.val);
            } 
            
        }
    }
    public void Register(){
        if(avail.containsKey(name)){
            throw new IllegalArgumentException("DynamicNTBoolean with name "+name+" already exists");
        }
        avail.put(name, this);
    }
    public DynamicNTBoolean(String name, boolean val, BooleanConsumer update){
        this.name=name;
        this.val=val;
        this.update=update;
        this.ntEntry=NetworkTableInstance.getDefault().getBooleanTopic(name).getEntry(false);
        this.ntEntry.set(val);
    }
}
