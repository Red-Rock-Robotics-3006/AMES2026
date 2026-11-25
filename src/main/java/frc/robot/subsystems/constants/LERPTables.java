package frc.robot.subsystems.constants;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LERPTables {
    //first entry is distance from target, second entry is end effector rpm
    public static InterpolatingDoubleTreeMap l1RPSLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 200.0), 
        Map.entry(0.59, 600.0), 
        Map.entry(0.74, 770.0), 
        Map.entry(0.95, 900.0)
    );
    
    public static InterpolatingDoubleTreeMap l2RPSLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 650.0), 
        Map.entry(0.59, 900.0), 
        Map.entry(0.74, 970.0), 
        Map.entry(0.95, 110.0)
    );

    public static InterpolatingDoubleTreeMap l3RPSLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 1100.0), 
        Map.entry(0.59, 1220.0), 
        Map.entry(0.74, 1280.0), 
        Map.entry(0.95, 1460.0)
    );
    
    //first entry is distance from target, second entry is arm angle
    public static InterpolatingDoubleTreeMap l1ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 0.15), 
        Map.entry(0.59, 0.15), 
        Map.entry(0.74, 0.15), 
        Map.entry(0.95, 0.15)                                   
    );

    public static InterpolatingDoubleTreeMap l2ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 0.22), 
        Map.entry(0.59, 0.22), 
        Map.entry(0.74, 0.22), 
        Map.entry(0.95, 0.22)                                     
    );

    public static InterpolatingDoubleTreeMap l3ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.29, 0.255), 
        Map.entry(0.59, 0.253), 
        Map.entry(0.74, 0.252), 
        Map.entry(0.95, 0.251)                                     
    );
}
