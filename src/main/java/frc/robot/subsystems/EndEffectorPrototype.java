package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class EndEffectorPrototype extends SubsystemBase{
    private static EndEffectorPrototype instance = null;

    private final RedRockTalon wheelMotor = new RedRockTalon(0, "EndEffectorPrototype/wheel-motor"); //TODO

    private SmartDashboardNumber wheelSpeedRPM = new SmartDashboardNumber("EndEffectorPrototype/wheel-speed-rpm", 200);

    public EndEffectorPrototype(){
        this.wheelMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
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

    private void setSpeed(){
        wheelMotor.setMotionMagicVelocity(wheelSpeedRPM.getNumber());
    }

    private void stop(){
        wheelMotor.motor.setControl(new CoastOut());
    }

    private void increaseSpeed(){
        wheelSpeedRPM.putNumber(wheelSpeedRPM.getNumber()+10);
    }
    
    private void decreaseSpeed(){
        wheelSpeedRPM.putNumber(wheelSpeedRPM.getNumber()-10);
    }

    public Command startEndEffectorTestCommand(){
        return Commands.runOnce(() -> this.setSpeed(), this);
    }

    public Command stopEndEffectorTestCommand(){
        return Commands.runOnce(() -> this.stop(), this);
    }

    public Command increaseEndEffectorSpeedTestCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.increaseSpeed(), this),
            Commands.runOnce(() -> this.setSpeed(), this)
        );
    }

    public Command decreaseEndEffectorSpeedTestCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.decreaseSpeed(), this),
            Commands.runOnce(() -> this.setSpeed(), this)
        );
    }

    @Override
    public void periodic(){
        this.wheelMotor.update();
    }

    public EndEffectorPrototype getInstance(){
        if(instance == null) return new EndEffectorPrototype();
        return instance;
    }
}