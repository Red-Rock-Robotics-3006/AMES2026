package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;
import frc.robot.Superstructure;
import frc.robot.subsystems.constants.LERPTables;

public class EndEffector extends SubsystemBase{
    private static EndEffector instance = null;

    private final Superstructure superstructure = new Superstructure();

    private final RedRockTalon endEffectMotor = new RedRockTalon(0, "EndEffector/endEffectMotor"); //TODO


    //Constants
    SmartDashboardNumber intakingVel = new SmartDashboardNumber("EndEffector/intakingVel", -1200);
    SmartDashboardNumber intakingCellVel = new SmartDashboardNumber("EndEffector/intakingCellVel", 0);//TODO
    SmartDashboardNumber intakingShellVel = new SmartDashboardNumber("EndEffector/intakingShellVel", 0);//TODO
    SmartDashboardNumber scoreShelfVel = new SmartDashboardNumber("EndEffector/scoreShelfVel", 0);//TODO
    SmartDashboardNumber intakeCellSourceVel = new SmartDashboardNumber("EndEffector/intakeCellSourceVel", 0);//TODO
    SmartDashboardNumber intakeShellSourceVel = new SmartDashboardNumber("EndEffector/intakeShellSourceVel", 0);//TODO


    //LERP disabled values
    SmartDashboardNumber staticL1Vel = new SmartDashboardNumber("EndEffector/staticL1Vel", 0);//TODO
    SmartDashboardNumber staticL2Vel = new SmartDashboardNumber("EndEffector/staticL2Vel", 0);//TODO
    SmartDashboardNumber staticL3Vel = new SmartDashboardNumber("EndEffector/staticL3Vel", 0);//TODO


    //Thresholds and Limits
    SmartDashboardNumber velocityThreshold = new SmartDashboardNumber("EndEffector/velocityThreshold", 4);

    private SmartDashboardNumber currentLimit = new SmartDashboardNumber("EndEffector/current-limit", 45);

    public EndEffector(){
        this.endEffectMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) 
            .withKV(0)
            .withKP(0)
            .withKI(0) 
            .withKD(0) 
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
    }


    //Movement methods
    private void setSpeed(double velocity){
        endEffectMotor.setMotionMagicVelocity(velocity);
    }

    private void stop(){
        endEffectMotor.motor.setControl(new CoastOut());
    }


    //Other Methods
    public boolean atWantedVelocity(double dist){
        return Math.abs(
            this.endEffectMotor.motor.getVelocity().getValueAsDouble()*60 - 
            this.stateToVelocity(Superstructure.getInstance().getWantedState(), dist)
        ) <= this.velocityThreshold.getNumber();
    }

    //Superstructure conversion
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
    
    //Commands
    public Command shootCommand(double dist){
        return Commands.runOnce(
            () -> this.setSpeed(stateToVelocity(superstructure.getWantedState(), dist)), 
            this
        );
    }
    
    public Command shootStaticCommand(){
        return Commands.runOnce(
            () -> this.setSpeed(stateToVelocitySTATIC(superstructure.getWantedState())), 
            this
        );
    }

    public Command stopEndEffectorCommand(){
        return Commands.runOnce(
            () -> this.stop(), 
            this
        );
    }

    public Command ejectPieceCommand(){
        return Commands.runOnce(
            () -> this.setSpeed(-this.intakingShellVel.getNumber()), this
        );
    }
    
    public Command intakeShellCommand(){
        return new FunctionalCommand(
            () -> this.setSpeed(intakeShellSourceVel.getNumber()), 
            () -> {}, 
            (interrupted) -> this.stop(), 
            () -> {
                return this.endEffectMotor.motor.getSupplyCurrent().getValueAsDouble() > this.currentLimit.getNumber();
            }, 
            this
        );
    }

    public Command intakeCellCommand(){
        return new FunctionalCommand(
            () -> this.setSpeed(intakeCellSourceVel.getNumber()), 
            () -> {}, 
            (interrupted) -> this.stop(), 
            () -> {
                return this.endEffectMotor.motor.getSupplyCurrent().getValueAsDouble() > this.currentLimit.getNumber();
            }, 
            this
        );
    }


    
    @Override
    public void periodic(){
        this.endEffectMotor.update();

        
        SmartDashboard.putBoolean("EndEffector/at-wanted-velocity", this.atWantedVelocity(0)); //TODO add distance from peg board when have odometry
    
        SmartDashboard.putNumber("EndEffector/supply-current", this.endEffectMotor.motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("EndEffector/velocity", this.endEffectMotor.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("EndEffector/acceleration", this.endEffectMotor.motor.getAcceleration().getValueAsDouble());
    }

    public static EndEffector getInstance(){
        if(instance == null) instance = new EndEffector();
        return instance;
    }
}