package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.wrappers.RedRockTalon;

public class EndEffector extends SubsystemBase{
    private static EndEffector instance = null;

    private final RedRockTalon endEffectorMotor = new RedRockTalon(0, "EndEffector/endEffector-motor");
    
    public EndEffector(){
        super("EndEffector");
    }
}
