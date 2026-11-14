package frc.robot;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AutoAlign;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.vision.Odometry;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.EndEffector.IntakeState;
import frc.robot.subsystems.constants.LERPTables;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardBoolean;
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
        INTERMEDIATE,
        STOW,
        TEST
    }

    
    
    private RobotState wantedState = RobotState.STOW;
    private RobotState lastState = null;

    
    private EndEffector endEffector = EndEffector.getInstance();
    private Arm arm = Arm.getInstance();
    private Intake intake = Intake.getInstance();
    private Odometry odometry = Odometry.getInstance();
    private AutoAlign autoAlign = AutoAlign.getInstance();

    private Command activeCommand = null;

    private CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance(); 
    // private LED led = LED.getInstance();

    //ARM

    //LERP disable poses
    private SmartDashboardNumber l1ArmPos = new SmartDashboardNumber("ArmPoses/Positions/L1", 0); //TODO
    private SmartDashboardNumber l2ArmPos = new SmartDashboardNumber("ArmPoses/Positions/L2", 0); //TODO
    private SmartDashboardNumber l3ArmPos = new SmartDashboardNumber("ArmPoses/Positions/L3", 0); //TODO
    
    //Static arm locations
    private SmartDashboardNumber shelfArmPos = new SmartDashboardNumber("ArmPoses/Positions/shelf", 0.1); 
    private SmartDashboardNumber sourceShellArmPos = new SmartDashboardNumber("ArmPoses/Positions/source-shell", 0); //TODO
    private SmartDashboardNumber sourceCellArmPos = new SmartDashboardNumber("ArmPoses/Positions/source-cell", 0); //TODO
    private SmartDashboardNumber stowArmPos = new SmartDashboardNumber("ArmPoses/Positions/stow", 0); 
    private SmartDashboardNumber intakeShellPos = new SmartDashboardNumber("ArmPoses/Positions/intake-shell-pos", 0.64); 
    private SmartDashboardNumber intakeCellPos = new SmartDashboardNumber("ArmPoses/Positions/intake-cell-pos", 0.58);
    private SmartDashboardNumber intermediatePos = new SmartDashboardNumber("ArmPoses/Positions/intermediate", 0.3);
    private SmartDashboardNumber test = new SmartDashboardNumber("ArmPoses/Positions/test", 0.28);
    //EndEffector

    //Static Velocities
    SmartDashboardNumber intakingVel = new SmartDashboardNumber("EndEffectorSpeeds/intakingVel", -1200);
    SmartDashboardNumber intakingCellVel = new SmartDashboardNumber("EndEffectorSpeeds/intakingCellVel", 0);//TODO
    SmartDashboardNumber intakingShellVel = new SmartDashboardNumber("EndEffectorSpeeds/intakingShellVel", 0);//TODO
    SmartDashboardNumber scoreShelfVel = new SmartDashboardNumber("EndEffectorSpeeds/scoreShelfVel", 0);//TODO
    SmartDashboardNumber intakeCellSourceVel = new SmartDashboardNumber("EndEffectorSpeeds/intakeCellSourceVel", 0);//TODO
    SmartDashboardNumber intakeShellSourceVel = new SmartDashboardNumber("EndEffectorSpeeds/intakeShellSourceVel", 0);//TODO

    //LERP disable velocities
    SmartDashboardNumber staticL1Vel = new SmartDashboardNumber("EndEffectorSpeeds/staticL1Vel", 0);//TODO
    SmartDashboardNumber staticL2Vel = new SmartDashboardNumber("EndEffectorSpeeds/staticL2Vel", 0);//TODO
    SmartDashboardNumber staticL3Vel = new SmartDashboardNumber("EndEffectorSpeeds/staticL3Vel", 0);//TODO

    //Intake

    //Odometry
    public static enum DockColumns{
        LEFT,
        LEFT_MID,
        RIGHT_MID,
        RIGHT,
        NONE
    }

    private DockColumns dockColumnTarget = DockColumns.NONE;

    public DockColumns getDockColumn(){
        return dockColumnTarget;
    }

    public static int getDockColumnAsInt(){
        return getInstance().getIntDockCol(getInstance().getDockColumn());
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
            default:
                return -1;
        }
        
    }
    //TODO what does this do?
    public Command fullAutoAlignCommand(){
        if(!(this.dockColumnTarget != DockColumns.NONE || odometry.get2dDistToTargetAllCams(getIntDockCol(dockColumnTarget)) != null)){
            // Logger.recordOutput("Superstructure/ Auto Aligning", false);
            return Commands.runOnce(()->{});
        }
        // Logger.recordOutput("Superstructure/ Auto Aligning", true);
        return autoAlign.autoAlignToTarget(true);
                
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
                return this.intakeCellPos.getNumber();
            case INTAKING_SHELL:
                return this.intakeShellPos.getNumber();
            case SOURCE_CELL:
                return this.sourceCellArmPos.getNumber();
            case SOURCE_SHELL:
                return this.sourceShellArmPos.getNumber();
            case SHELF:
                return this.shelfArmPos.getNumber();
            case STOW:
                return this.stowArmPos.getNumber();
            case INTERMEDIATE:
                return this.intermediatePos.getNumber();
            case TEST:
                return this.test.getNumber();
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
            case SOURCE_CELL:
                return this.sourceCellArmPos.getNumber();
            case SOURCE_SHELL:
                return this.sourceShellArmPos.getNumber();
            case SHELF:
                return this.shelfArmPos.getNumber();
            case INTAKING_CELL:     //Currently we want all these poses to be the same as stow but are keeping them incase they need to be different values after testing
                return this.intakeCellPos.getNumber();
            case INTAKING_SHELL:
                return this.intakeShellPos.getNumber();
            case STOW:
                return this.stowArmPos.getNumber();
            case INTERMEDIATE:
                return this.intermediatePos.getNumber();
            case TEST:
                return this.test.getNumber();
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
            case TEST:
                return 0;
            case INTERMEDIATE:
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
            case INTERMEDIATE:
                return 0;
            case STOW:
                return 0;
            case TEST:
                return 0;
            default:
                return 0.0;
            }
    }

    public boolean atWantedArmPosSuperstructure(){
        return RobotState.L1 == wantedState || RobotState.L2 == wantedState || RobotState.L3 == wantedState
            ? arm.atWantedArmPos(stateToArmPos(wantedState, distanceFromTarget))
            : arm.atWantedArmPos(stateToArmPosStatic(wantedState));
    }

    public Command endEffectCommand(){
        return Commands.runOnce(() -> this.endEffector.shootCommand(stateToVelocity(this.wantedState, this.distanceFromTarget)), endEffector);
    }

    //Superstructure-specific commands
    
    public void setWantedRobotState(RobotState state){
        wantedState = state;
    }

    public void setDockTarget(DockColumns targetCol){ 
        this.dockColumnTarget = targetCol;
    }
    
    public RobotState getWantedState(){
        return wantedState;
    }
    
    public RobotState getlastState(){
        return lastState;
    }

    public double getDistToTarget(){
        return this.distanceFromTarget;
    }
    
    public void initializeBot(){
        this.arm.zeroArmCommand();
    }
    
    //Arm commands

    //when arm is moving endeffector slowly pushes piece out so compensate with low reversing speed 
    public Command goToArmPosStaticCommand(RobotState state){
        return new FunctionalCommand( 
            () -> {
                arm.armToPos(this.stateToArmPosStatic(state));
                this.setWantedRobotState(state);
            },
            () -> endEffector.setSpeed(-arm.getPivotVelocity()*(3.0/64.0)),
            (interrupted) -> {
                endEffector.setSpeed(0);
            },
            () -> {
                return arm.atWantedArmPos(this.stateToArmPosStatic(state));
            },
            this, arm, endEffector
        );
    }

    public Command goToArmPosCommand(RobotState state, double dist){
        return new FunctionalCommand( 
            () -> {
                arm.armToPos(this.stateToArmPosStatic(state));
                this.setWantedRobotState(state);
            },
            () -> endEffector.setSpeed(-arm.getPivotVelocity()*(3.0/64.0)),
            (interrupted) -> {
                endEffector.setSpeed(0);
            },
            () -> {
                return arm.atWantedArmPos(this.stateToArmPos(state, dist));
            },
            this, arm, endEffector
        );
    }

    public Command goToIntakePos(){
        if(endEffector.getIntakeState() == IntakeState.NOTHING)
            return this.goToArmPosStaticCommand(RobotState.INTAKING_SHELL);
        else
            return this.goToArmPosStaticCommand(RobotState.INTAKING_CELL);
    }
    
    public Command idleCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> arm.idle()),
            Commands.runOnce(() -> endEffector.stopEndEffectorCommand()),
            Commands.runOnce(() -> intake.stopIntake())
        );
    }

    public Command intakeEndEffectCommand(){
        return endEffector.intakeCommand();
    }

    //since cases are very similar let similar cases fall through to the next, it will end at the break
    // private Command goToWantedState(){
    //     switch(this.wantedState){
    //         case L1:
    //         case L2:
    //         case L3:
    //             if(this.dockColumnTarget != DockColumns.NONE){
    //                 // Commands.sequence(
    //                     return this.goToArmPosCommand(wantedState, distanceFromTarget);
    //                     // this.endEffector.shootCommand(this.stateToVelocity(RobotState.L1, distanceFromTarget))
    //                     // );
    //                 }
    //             else
    //             // Commands.sequence(
    //                     return this.goToArmPosStaticCommand(wantedState);
    //                     // this.endEffector.shootCommand(this.stateToVelocitySTATIC(wantedState))
    //                 // );
    //         case SOURCE_SHELL: //could also make it so when wantedState is source start end effector intaking
    //         case SOURCE_CELL:
    //             // Logger.recordOutput("Superstructure/ Auto Aligning", false);
    //             return Commands.sequence(
    //                 this.goToArmPosStaticCommand(wantedState),
    //                 endEffector.intakeCommand()
    //             );
    //         case INTAKING_CELL:
    //         case INTAKING_SHELL:
    //         case INTERMEDIATE:
    //         case STOW:
    //         case TEST:
    //         case SHELF:
    //             // Logger.recordOutput("Superstructure/ Auto Aligning", false);
    //             return this.goToArmPosStaticCommand(wantedState);
    //         default:
    //             return null;
    //     }
    // }

    @Override
    public void periodic(){
        distanceFromTarget = dockColumnTarget!=DockColumns.NONE
            ?odometry.get2dDistToTargetAllCams(this.getIntDockCol(dockColumnTarget)) == null
                ? -1
                : odometry.get2dDistToTargetAllCams(this.getIntDockCol(dockColumnTarget))
            :-1;
        SmartDashboard.putNumber("Superstructure/distanceFromTarget", distanceFromTarget);
        //used to check when new button has been pressed and runs a command scheduler
        // if(lastState == null || wantedState != lastState){
        //     lastState = wantedState;
        //     //cancel old command and run new command
        //     if(activeCommand != null) 
        //         activeCommand.cancel();
        //     activeCommand = goToWantedState();
        //     if(activeCommand != null){
        //         CommandScheduler.getInstance().schedule(activeCommand);
        //         SmartDashboard.putData("Superstructure/activeStateCommand", activeCommand);
        //     }
        // }

        SmartDashboard.putBoolean("Arm/at-wanted-pos", this.atWantedArmPosSuperstructure()); //TODO add distance from peg board when have odometry
        // Logger.recordOutput("Arm/at-wanted-pos", arm.atWantedArmPos(distanceFromTarget)); //TODO add distance from peg board when have odometry
        
        // if(this.dockColumnTarget != DockColumns.NONE){
        //     autoAlign.autoAlignToTarget(this.getIntDockCol(dockColumnTarget));
        //     SmartDashboard.putBoolean("Superstructure/aligning-to-target", this.dockColumnTarget != DockColumns.NONE);
        //     SmartDashboard.putBoolean("Superstructure/aligned-to-target", autoAlign.alignedToTarget());
        // }
        // SmartDashboard.putData("Superstructure/current-state", (Sendable)currentState);
    }

    public Superstructure(){
        // AutoLogOutputManager.addObject(this);
    }
    public static Superstructure getInstance(){
        if(instance == null) instance = new Superstructure();
        return instance;
    }
}
