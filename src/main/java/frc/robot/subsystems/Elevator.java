package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.wrappers.RedRockTalon;

public class Elevator extends SubsystemBase{
    private static Elevator instance = null;

    private final RedRockTalon elevatorMotor1 = new RedRockTalon(0, "Elevator/elevator-motor-1");
    private final RedRockTalon elevatorMotor2 = new RedRockTalon(0, "Elevator/elevator-motor-2");

    public Elevator(){
        super("Elevator");
    }

    @Override
    public void periodic(){

    }

    public Elevator getInstance(){
        if(instance == null) instance = new Elevator();
        return instance;
    }
}
