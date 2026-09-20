package frc.robot.subsystems;

import redrocklib.wrappers.RedRockTalon;

public class Elevator {
    private static Elevator instance = null;

    private final RedRockTalon elevatorMotor1 = new RedRockTalon(0, "Elevator/elevator-motor-1");
    private final RedRockTalon elevatorMotor2 = new RedRockTalon(0, "Elevator/elevator-motor-2");

}
