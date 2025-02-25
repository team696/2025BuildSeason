// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;

/** Add your docs here. */
public class DynamicNTVal<T> {
    String name;
    T val;
    T prev;
    NetworkTableEntry ntEntry;
    Consumer<T> update;
    public static Map<String, DynamicNTVal> avail=new HashMap<String, DynamicNTVal>();
    public static void periodic(){
        for(DynamicNTVal v:avail.values()){
            v.prev=v.val;
            v.val=v.MagicPull();
            if(!v.val.equals(v.prev)){
                v.update.accept(v.val);
            } 
            
        }
    }
    public void Register(){
        if(avail.containsKey(name)){
            throw new IllegalArgumentException("DynamicNTVal with name "+name+" already exists");
        }
        avail.put(name, this);
    }
    public T MagicPull(){
        switch(ntEntry.getType()){
            case kDouble:
                return (T)Double.valueOf(ntEntry.getDouble(0));
            case kBoolean:
                return (T)Boolean.valueOf(ntEntry.getBoolean(false));
            case kString:
                return (T)ntEntry.getString("");
            default:
                throw new IllegalArgumentException("DynamicNTVal with name "+name+" has invalid type");
        }
    }
    // this is a magic function that allows you to put any type of value into the network table
    public void MagicPut(T val){
        if(val instanceof Double)
            ntEntry.setDouble((double)val);
        else if(val instanceof Boolean)
            ntEntry.setBoolean((boolean)val);
        else if(val instanceof String)
            ntEntry.setString((String)val);
        else
            throw new IllegalArgumentException("DynamicNTVal with name "+name+" has invalid type");

    }
    public DynamicNTVal(String name, T val, Consumer<T> update){
        this.name=name;
        this.val=val;
        this.update=update;
        this.ntEntry=NetworkTableInstance.getDefault().getEntry(name);
        MagicPut(val);
    }
}
