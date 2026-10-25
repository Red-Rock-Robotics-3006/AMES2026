package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;
import frc.robot.Superstructure;

public class EndEffector extends SubsystemBase{
    private static EndEffector instance = null;

    private final Superstructure superstructure = new Superstructure();

    private final RedRockTalon endEffectMotor = new RedRockTalon(0, "EndEffector/endEffectMotor"); //TODO

    //Thresholds and Limits
    SmartDashboardNumber velocityThreshold = new SmartDashboardNumber("EndEffector/velocityThreshold", 70);
    SmartDashboardNumber intakeVelocity = new SmartDashboardNumber("EndEffector/intaking-velocity", -1200);
    SmartDashboardNumber outakeVelocity = new SmartDashboardNumber("EndEffector/outaking-velocity", 500);
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
    public boolean atWantedVelocity(double velocity){
        return Math.abs(
            this.endEffectMotor.motor.getVelocity().getValueAsDouble()*60 - 
            velocity
        ) <= this.velocityThreshold.getNumber();
    }

    
    //Commands
    public Command shootCommand(double velocity){
        return Commands.runOnce(
            () -> this.setSpeed(velocity), this
        );
    }
    
    public Command stopEndEffectorCommand(){
        return Commands.runOnce(
            () -> this.stop(), 
            this
        );
    }

    public Command outakeCommand(){
        return Commands.runOnce(
            () -> this.setSpeed(this.outakeVelocity.getNumber()), this
        );
    }
    

    public Command intakeCommand(){
        return new FunctionalCommand(
            () -> this.setSpeed(intakeVelocity.getNumber()), 
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