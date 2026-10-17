package frc.robot.subsystems.constants;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LERPTables {
    //first entry is distance from target, second entry is end effector rpm
    public static InterpolatingDoubleTreeMap l1RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    
    public static InterpolatingDoubleTreeMap l2RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );

    public static InterpolatingDoubleTreeMap l3RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    
    //first entry is distance from target, second entry is arm angle
    public static InterpolatingDoubleTreeMap l1ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 0.0), 
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );

    public static InterpolatingDoubleTreeMap l2ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 0.0), 
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );

    public static InterpolatingDoubleTreeMap l3ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 0.0), 
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
}
