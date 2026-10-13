package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class Superstructure {
    private static Superstructure instance = null;

    private Intake intake = Intake.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    private Arm arm = Arm.getInstance();
    // private LED led = LED.getInstance();
    // private Elevator elevator = Elevator.getInstance();

    public static enum RobotState{
        L1,
        L2,
        L3,
        INTAKING_CELL,
        INTAKING_SHELL,
        SOURCE_CELL,
        SOURCE_SHELL,
        SHELF,
        IDLE,
        STOW,
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

    private void setCurrentRobotState(RobotState state){
        currentState = state;
    }

    private void setWantedRobotState(RobotState state){
        wantedState = state;
    }


    public RobotState getWantedState(){
        return wantedState;
    }

    public RobotState getCurrentState(){
        return currentState;
    }

    public void initializeBot(){
        this.arm.zeroArmCommand();
    }
    
    public Command goToIntakeCellCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.INTAKING_CELL),
            () -> arm.goToStaticPosCommand(RobotState.INTAKING_CELL),
            (interrupted) -> this.setCurrentRobotState(RobotState.INTAKING_CELL),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }

    public Command goToIntakeShellCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.INTAKING_SHELL),
            () -> arm.goToStaticPosCommand(RobotState.INTAKING_SHELL),
            (interrupted) -> this.setCurrentRobotState(RobotState.INTAKING_SHELL),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }

    public Command goToSourceShellCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.SOURCE_SHELL),
            () -> arm.goToStaticPosCommand(RobotState.SOURCE_SHELL),
            (interrupted) -> this.setCurrentRobotState(RobotState.SOURCE_SHELL),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }

    public Command goToSourceCellCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.SOURCE_CELL),
            () -> arm.goToStaticPosCommand(RobotState.SOURCE_CELL),
            (interrupted) -> this.setCurrentRobotState(RobotState.SOURCE_CELL),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }

    public Command goToShelfCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.SHELF),
            () -> arm.goToStaticPosCommand(RobotState.SHELF),
            (interrupted) -> this.setCurrentRobotState(RobotState.SHELF),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }

    public Command goToStowCommand(){
        return new FunctionalCommand(
            () -> this.setWantedRobotState(RobotState.STOW),
            () -> arm.goToStaticPosCommand(RobotState.STOW),
            (interrupted) -> this.setCurrentRobotState(RobotState.STOW),
            () -> {
                return arm.atWantedArmPosStatic();
            }
        );
    }
    
    public Command idleCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setWantedRobotState(RobotState.IDLE)),
            Commands.runOnce(() -> arm.stopArmCommand(), arm),
            Commands.runOnce(() -> endEffector.stopEndEffectorCommand(), endEffector),
            Commands.runOnce(() -> intake.stopIntakeCommand(), intake),
            Commands.runOnce(() -> this.setCurrentRobotState(RobotState.IDLE))
        );
    }

    public static Superstructure getInstance(){
        if(instance == null) instance = new Superstructure();
        return instance;
    }
}
