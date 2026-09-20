package frc.robot.subsystems;

import redrocklib.wrappers.RedRockTalon;

public class EndEffector {
    private static EndEffector instance = null;

    private final RedRockTalon endEfMotor = new RedRockTalon(0, "EndEffector/endEffector-motor");
    
}
