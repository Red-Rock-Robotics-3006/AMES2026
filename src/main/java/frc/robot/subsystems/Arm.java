package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Superstructure;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Arm extends SubsystemBase{
    private static Arm instance = null;

    private final RedRockTalon pivotMotor = new RedRockTalon(0, "Arm/pivot-motor", "*");
    private final CANcoder encoder = new CANcoder(0, "*");

    private SmartDashboardNumber kRotorToSensorRatio = new SmartDashboardNumber("Arm/CANCoder/k-rotor-to-sensor-ratio", 0); //TODO
    private SmartDashboardNumber kSensorToMechRatio = new SmartDashboardNumber("Arm/CANCoder/k-sensor-to-mech-ratio", 0); //TODO
    private SmartDashboardNumber l1ArmPos = new SmartDashboardNumber("Arm/Positions/L1", 0); //TODO
    private SmartDashboardNumber l2ArmPos = new SmartDashboardNumber("Arm/Positions/L2", 0); //TODO
    private SmartDashboardNumber l3ArmPos = new SmartDashboardNumber("Arm/Positions/L3", 0); //TODO
    private SmartDashboardNumber shelfArmPos = new SmartDashboardNumber("Arm/Positions/shelf", 0); //TODO
    private SmartDashboardNumber intakingArmPos = new SmartDashboardNumber("Arm/Positions/intaking", 0); //TODO
    private SmartDashboardNumber sourceShellArmPos = new SmartDashboardNumber("Arm/Positions/source-shell", 0); //TODO
    private SmartDashboardNumber sourceCellArmPos = new SmartDashboardNumber("Arm/Positions/source-cell", 0); //TODO
    private SmartDashboardNumber stowArmPos = new SmartDashboardNumber("Arm/Positions/stow", 0); //TODO
    private SmartDashboardNumber stoppedArmPos = new SmartDashboardNumber("Arm/Positions/stopped", 0); //TODO

    public Arm(){
        super("Arm");

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(encoder.getDeviceID())
        .withRotorToSensorRatio(kRotorToSensorRatio.getNumber()) //How many motor rotations are there for every rotation of the CANCoder?
        .withSensorToMechanismRatio(kSensorToMechRatio.getNumber()); //How many CANCoder rotations are there to each mechanism rotation

        this.pivotMotor.withMotorOutputConfigs(
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

    private double stateToPos(Superstructure.RobotState state){
        switch(state){
            case L1:
                return this.l1ArmPos.getNumber();
            case L2:
                return this.l2ArmPos.getNumber();
            case L3:
                return this.l3ArmPos.getNumber();
            case INTAKING:
                return this.intakingArmPos.getNumber();
            case SOURCE_CELL:
                return this.sourceCellArmPos.getNumber();
            case SOURCE_SHELL:
                return this.sourceShellArmPos.getNumber();
            case SHELF:
                return this.shelfArmPos.getNumber();
            case STOPPED:
                return this.stoppedArmPos.getNumber();
            case STOW:
                return this.stowArmPos.getNumber();
            default:
                return 0.0;
        }
    }     

    private void armToPos(Superstructure.RobotState state){
        pivotMotor.setMotionMagicPosition(0);
    }
    public static Arm getInstance(){
        if(instance == null) return Arm.instance = new Arm();
        return instance;
    }
}
