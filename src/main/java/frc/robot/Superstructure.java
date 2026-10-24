package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.vision.Odometry;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.constants.LERPTables;
import redrocklib.logging.SmartDashboardNumber;

public class Superstructure extends SubsystemBase{
    private static Superstructure instance = null;

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

    
    
    private RobotState wantedState = RobotState.IDLE;
    private RobotState currentState = RobotState.IDLE;

    
    private EndEffector endEffector = EndEffector.getInstance();
    private Arm arm = Arm.getInstance();
    private Intake intake = Intake.getInstance();
    private Odometry odometry = Odometry.getInstance();

    // private CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance(); 
    // private LED led = LED.getInstance();

    //ARM

    //LERP disable poses
    private SmartDashboardNumber l1ArmPos = new SmartDashboardNumber("Arm/Positions/L1", 0); //TODO
    private SmartDashboardNumber l2ArmPos = new SmartDashboardNumber("Arm/Positions/L2", 0); //TODO
    private SmartDashboardNumber l3ArmPos = new SmartDashboardNumber("Arm/Positions/L3", 0); //TODO
    
    //Static arm locations
    private SmartDashboardNumber shelfArmPos = new SmartDashboardNumber("Arm/Positions/shelf", 0); //TODO
    private SmartDashboardNumber sourceShellArmPos = new SmartDashboardNumber("Arm/Positions/source-shell", 0); //TODO
    private SmartDashboardNumber sourceCellArmPos = new SmartDashboardNumber("Arm/Positions/source-cell", 0); //TODO
    private SmartDashboardNumber stowArmPos = new SmartDashboardNumber("Arm/Positions/stow", 0); //TODO
    private SmartDashboardNumber stoppedArmPos = new SmartDashboardNumber("Arm/Positions/stopped", 0); //TODO

    //EndEffector

    //Static Velocities
    SmartDashboardNumber intakingVel = new SmartDashboardNumber("EndEffector/intakingVel", -1200);
    SmartDashboardNumber intakingCellVel = new SmartDashboardNumber("EndEffector/intakingCellVel", 0);//TODO
    SmartDashboardNumber intakingShellVel = new SmartDashboardNumber("EndEffector/intakingShellVel", 0);//TODO
    SmartDashboardNumber scoreShelfVel = new SmartDashboardNumber("EndEffector/scoreShelfVel", 0);//TODO
    SmartDashboardNumber intakeCellSourceVel = new SmartDashboardNumber("EndEffector/intakeCellSourceVel", 0);//TODO
    SmartDashboardNumber intakeShellSourceVel = new SmartDashboardNumber("EndEffector/intakeShellSourceVel", 0);//TODO


    //LERP disable velocities
    SmartDashboardNumber staticL1Vel = new SmartDashboardNumber("EndEffector/staticL1Vel", 0);//TODO
    SmartDashboardNumber staticL2Vel = new SmartDashboardNumber("EndEffector/staticL2Vel", 0);//TODO
    SmartDashboardNumber staticL3Vel = new SmartDashboardNumber("EndEffector/staticL3Vel", 0);//TODO

    //Intake

    //Odometry
    private DockColumns dockColumnTarget = DockColumns.NONE;
    public static enum DockColumns{
        LEFT,
        LEFT_MID,
        RIGHT_MID,
        RIGHT,
        NONE
    }
    private double distanceFromTarget;

    //Statemachine to subsystem comms
    //Arm
    
    private double stateToArmPos(Superstructure.RobotState state, double dist){
        switch(state){
            case L1:
            return LERPTables.l1ArmPosLerpTable.get(dist);
            case L2:
            return LERPTables.l2ArmPosLerpTable.get(dist);
            case L3:
                return LERPTables.l3ArmPosLerpTable.get(dist);
            case INTAKING_CELL:
            return this.sourceCellArmPos.getNumber();
            case INTAKING_SHELL:
            return this.sourceShellArmPos.getNumber();
            case SOURCE_CELL:
            return this.sourceCellArmPos.getNumber();
            case SOURCE_SHELL:
                return this.sourceShellArmPos.getNumber();
                case SHELF:
                return this.shelfArmPos.getNumber();
                case STOW:
                return this.stowArmPos.getNumber();
                case IDLE:
                return this.stoppedArmPos.getNumber();
                default:
                return 0.0;
            }
    }    
    
    private double stateToArmPosStatic(Superstructure.RobotState state){
        switch(state){
            case L1:
            return this.l1ArmPos.getNumber();
            case L2:
            return this.l2ArmPos.getNumber();
            case L3:
            return this.l3ArmPos.getNumber();
            case INTAKING_CELL:
            return this.sourceCellArmPos.getNumber();
            case INTAKING_SHELL:
            return this.sourceShellArmPos.getNumber();
            case SOURCE_CELL:
            return this.sourceCellArmPos.getNumber();
            case SOURCE_SHELL:
            return this.sourceShellArmPos.getNumber();
            case SHELF:
            return this.shelfArmPos.getNumber();
            case IDLE:
            return this.stoppedArmPos.getNumber();
            case STOW:
            return this.stowArmPos.getNumber();
            default:
                return 0.0;
        }
    }
    
    //EndEffector
    private double stateToVelocity(Superstructure.RobotState state, double dist){
        switch(state){
            case L1:
                return LERPTables.l1RPMLerpTable.get(dist);
            case L2:
                return LERPTables.l2RPMLerpTable.get(dist);
            case L3:
                return LERPTables.l2RPMLerpTable.get(dist);
            case INTAKING_CELL:
                return this.intakingCellVel.getNumber();
            case INTAKING_SHELL:
            return this.intakingShellVel.getNumber();
            case SOURCE_CELL:
            return this.intakeCellSourceVel.getNumber();
            case SOURCE_SHELL:
                return this.intakeShellSourceVel.getNumber();
            case SHELF:
                return this.scoreShelfVel.getNumber();
            case IDLE:
            return 0;
            case STOW:
                return 0;
            default:
            return 0.0;
        }
    }
    private double stateToVelocitySTATIC(Superstructure.RobotState state){
        switch(state){
            case L1:
                return this.staticL1Vel.getNumber();
            case L2: 
            return this.staticL1Vel.getNumber();
            case L3:
                return this.staticL3Vel.getNumber();
                case INTAKING_CELL:
                return this.intakingCellVel.getNumber();
            case INTAKING_SHELL:
            return this.intakingShellVel.getNumber();
            case SOURCE_CELL:
            return this.intakeCellSourceVel.getNumber();
            case SOURCE_SHELL:
                return this.intakeShellSourceVel.getNumber();
            case SHELF:
                return this.scoreShelfVel.getNumber();
            case IDLE:
                return 0;
            case STOW:
                return 0;
                default:
                return 0.0;
            }
    }
    public Command endEffectCommand(){
        return Commands.runOnce(() -> this.endEffector.shootCommand(stateToVelocity(this.wantedState, this.distanceFromTarget)), endEffector);
    }

    //Superstructure-specific commands
    private void setCurrentRobotState(RobotState state){
        currentState = state;
    }
    
    public void setWantedRobotState(RobotState state){
        wantedState = state;
    }

    public void setDockTarget(DockColumns targetCol){ 
        this.dockColumnTarget = targetCol;
    }
    
    public RobotState getWantedState(){
        return wantedState;
    }
    
    public RobotState getCurrentState(){
        return currentState;
    }

    public DockColumns getDockColumns(){
        return dockColumnTarget;
    }

    public int getIntDockCol(DockColumns dockCol){
        switch(dockCol){
            case LEFT:
                return 0;
            case LEFT_MID:
                return 1;
            case RIGHT_MID:
                return 2;
            case RIGHT:
                return 3;
            case NONE:
                return -1;
        }
    }
    public void initializeBot(){
        this.arm.zeroArmCommand();
    }
    
    //Arm commands
    private Command goToArmPosStaticCommand(RobotState state){
        return new FunctionalCommand( 
            () -> this.setWantedRobotState(state),
            () -> arm.goToStaticPosCommand(this.stateToArmPosStatic(wantedState)),
            (interrupted) -> this.setCurrentRobotState(state),
            () -> {
                return arm.atWantedArmPos(this.stateToArmPosStatic(wantedState));
            }
        );
    }

    private Command goToArmPosCommand(RobotState state, double dist){
        return new FunctionalCommand( 
            () -> this.setWantedRobotState(state),
            () -> arm.goToStaticPosCommand(this.stateToArmPos(wantedState, dist)),
            (interrupted) -> this.setCurrentRobotState(state),
            () -> {
                return arm.atWantedArmPos(this.stateToArmPosStatic(wantedState));
            }
        );
    }
    
    public Command idleCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setWantedRobotState(RobotState.IDLE)),
            Commands.runOnce(() -> arm.stopArmCommand(), arm),
            Commands.runOnce(() -> endEffector.stopEndEffectorCommand(), endEffector),
            // Commands.runOnce(() -> intake.stopIntakeCommand(), intake),
            Commands.runOnce(() -> this.setCurrentRobotState(RobotState.IDLE))
        );
    }

    public static Superstructure getInstance(){
        if(instance == null) instance = new Superstructure();
        return instance;
    }

    //since cases are very similar let similar cases fall through to the next, it will end at the break
    private void goToWantedState(){
        switch(this.wantedState){
            case L1:
            case L2:
            case L3:
                if(this.dockColumnTarget != DockColumns.NONE){
                    Commands.sequence(
                        this.goToArmPosCommand(wantedState, distanceFromTarget)
                        // this.endEffector.shootCommand(this.stateToVelocity(RobotState.L1, distanceFromTarget))
                    );
                }
                else
                    Commands.sequence(
                        this.goToArmPosStaticCommand(wantedState)
                        // this.endEffector.shootCommand(this.stateToVelocitySTATIC(wantedState))
                    );
                break;
            case SOURCE_SHELL:
            case SOURCE_CELL:
            case INTAKING_CELL:
            case INTAKING_SHELL:
                Commands.sequence(
                    this.goToArmPosStaticCommand(wantedState)
                );
                break;
            case SHELF:
            case STOW:
            case IDLE:
                this.goToArmPosStaticCommand(wantedState);
                break;
            default:
                return;
        }
    }

    @Override
    public void periodic(){
        distanceFromTarget = dockColumnTarget!=DockColumns.NONE?odometry.get2dDistToTargetAllCams(this.getIntDockCol(dockColumnTarget)):-1;
        this.goToWantedState();


        // SmartDashboard.putData("Superstructure/current-state", (Sendable)currentState);
    }
}
