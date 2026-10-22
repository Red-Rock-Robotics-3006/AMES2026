package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    public Intake(){

    }

    @Override
    public void periodic(){

    }

    public static Intake getInstance(){
        if(instance == null) instance = new Intake();
        return instance;
    }
=======
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase {
    private static Intake instance = null;
    public static final boolean kEnableTuning = true;

    private RedRockTalon intakeMotorOne = new RedRockTalon(0, "intake-motor-one", "*"); //TODO: Change the ID to the correct one
    private RedRockTalon intakeMotorTwo = new RedRockTalon(0, "intake-motor-two", "*"); //TODO: Change the ID to the correct one

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake speed", 0, kEnableTuning && true); //TODO: Change the default value to the correct one
    private SmartDashboardNumber intakeSpeedRatio = new SmartDashboardNumber("intake/intake speed ratio", 0, kEnableTuning && true); //TODO: Change the default value to the correct one //ratio of the speed of motor 1 to motor 2
    private SmartDashboardNumber intakeReverseSpeed = new SmartDashboardNumber("intake/intake reverse speed", 0, kEnableTuning && true); //TODO: Change the default value to the correct one

    private Intake() {
        super();

        this.intakeMotorOne.withTuningEnabled(kEnableTuning && true)
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimitConfigs(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        );

        this.intakeMotorTwo.withTuningEnabled(kEnableTuning && true)
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimitConfigs(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        );
    }

    public void setIntakeSpeed(double speed) {
        this.intakeMotorOne.motor.setControl(new DutyCycleOut(speed).withEnableFOC(false));
        this.intakeMotorTwo.motor.setControl(new DutyCycleOut(speed * intakeSpeedRatio.getNumber()).withEnableFOC(false));
    }

    public void stopIntake() {
        this.setIntakeSpeed(0);
    }

    public void startIntake() {
        this.setIntakeSpeed(intakeSpeed.getNumber());
    }

    public void reverseIntake() {
        this.setIntakeSpeed(intakeReverseSpeed.getNumber());
    }

    public Command startIntakeCommand() {
        return Commands.runOnce(() -> this.startIntake(), this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> this.stopIntake(), this);
    }

    public Command reverseIntakeCommand() {
        return Commands.runOnce(() -> this.reverseIntake(), this);
    }


    @Override
    public void periodic() {
        this.intakeMotorOne.update();
        this.intakeMotorTwo.update();
    }

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

>>>>>>> 1d6aa763e8acef9d0ddc10921c3b41d925c9f198
}
