package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    // private final Superstructure superstructure = Superstructure.getInstance();
    private final Arm arm = Arm.getInstance();

    private final RedRockTalon endEffectMotor = new RedRockTalon(31, "EndEffector/endEffectMotor"); //TODO

    //Thresholds and Limits
    private SmartDashboardNumber velocityThreshold = new SmartDashboardNumber("EndEffector/velocityThreshold", 70);
    private SmartDashboardNumber intakeVelocity = new SmartDashboardNumber("EndEffector/intaking-velocity", -1200);
    private SmartDashboardNumber outakeVelocity = new SmartDashboardNumber("EndEffector/outaking-velocity", 500);
    private SmartDashboardNumber keepPieceInSpeed = new SmartDashboardNumber("EndEffector/keep-pieces-in-end-effector-velocity", 10);
    private SmartDashboardNumber currentLimit = new SmartDashboardNumber("EndEffector/current-limit", 45);
    private SmartDashboardNumber testSpeed = new SmartDashboardNumber("EndEffector/test-speed", 2500);
    private SmartDashboardNumber testIntakeSpeed = new SmartDashboardNumber("EndEffector/test-intake-speed", -200);

    public EndEffector(){
        this.endEffectMotor
        .withFeedbackConfigs(
            new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(3)
        )
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) 
            .withKS(0.4) 
            .withKV(0.37)
            .withKP(0.6)
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
    public void setSpeed(double rpm){
        endEffectMotor.motor.setControl(new VelocityVoltage(rpm/60).withSlot(0).withOverrideBrakeDurNeutral(true));
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

    public Command shootTestCommand(){
        return Commands.runOnce(() -> this.setSpeed(this.testSpeed.getNumber()), this);
    }
    public Command runIntakeTestCommand(){
        return Commands.runOnce(() -> this.setSpeed(this.testIntakeSpeed.getNumber()), this);
    }
    public Command runIntakeTestCommandDutyCycle(){
        return Commands.runOnce(() -> endEffectMotor.motor.set(0.1), this);
    }
    public Command stopIntakeTestCommandDutyCycle(){
        return Commands.runOnce(() -> endEffectMotor.motor.set(0), this);
    }


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

    public Command keepGamePiecesInConstantCommand(){
        return Commands.runOnce(
            () -> this.setSpeed(this.keepPieceInSpeed.getNumber()), 
            this
        );
    }

    public Command keepGamePiecesInCommand(){
        return Commands.runOnce(
            () -> this.setSpeed(arm.getPivotVelocity()*(3.0/64.0)), 
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