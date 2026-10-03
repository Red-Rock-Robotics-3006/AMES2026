package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class Superstructure {
    private static Superstructure instance = null;

    private Intake intake = Intake.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private Arm arm = Arm.getInstance();
    private LED led = LED.getInstance();
    private Elevator elevator = Elevator.getInstance();

    public static enum RobotState{
        L1,
        L2,
        L3,
        INTAKING,
        SOURCE_CELL,
        SOURCE_SHELL,
        SHELF,
        STOPPED,
        IDLE,
        STOW
    }

/* For auto align
    public static enum FieldPos{
        LEFT,
        LEFTMID,
        RIGHT,
        RIGHTMID
    }
*/


    private RobotState wantedState = RobotState.IDLE;
    private RobotState currentState = RobotState.IDLE;

    public RobotState setCurrentRobotState(RobotState pos, double dist){
        this.intake.goToPos(pos);
        this.arm.goToArmPosCommand(pos, dist);
        this.endEffector.goToPos(pos);
    }

    public RobotState getWantedState(){
        return wantedState;
    }

    public RobotState getCurrentState(){
        return currentState;
    }

    public static Superstructure getInstance(){
        if(instance == null) return instance = new Superstructure();
        return instance;
    }
}
