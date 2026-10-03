package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Superstructure;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Arm extends SubsystemBase{
    private static Arm instance = null;

    private final Superstructure  superstructure= new Superstructure();

    private final RedRockTalon pivotMotor = new RedRockTalon(0, "Arm/pivot-motor", "*");
    private final CANcoder encoder = new CANcoder(0, "*");

    private SmartDashboardNumber kRotorToSensorRatio = new SmartDashboardNumber("Arm/CANCoder/k-rotor-to-sensor-ratio", 0); //TODO
    private SmartDashboardNumber kSensorToMechRatio = new SmartDashboardNumber("Arm/CANCoder/k-sensor-to-mech-ratio", 0); //TODO
    private SmartDashboardNumber kDiscontinuityPoint = new SmartDashboardNumber("Arm/CANCoder/k-discontinuity-point", 0.5); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc
    private SmartDashboardNumber kCANCoderOffset = new SmartDashboardNumber("Arm/CANCoder/k-cancoder-offset", 0); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc

    private SmartDashboardNumber l1ArmPos = new SmartDashboardNumber("Arm/Positions/L1", 0); //TODO
    private SmartDashboardNumber l2ArmPos = new SmartDashboardNumber("Arm/Positions/L2", 0); //TODO
    private SmartDashboardNumber l3ArmPos = new SmartDashboardNumber("Arm/Positions/L3", 0); //TODO
    private SmartDashboardNumber shelfArmPos = new SmartDashboardNumber("Arm/Positions/shelf", 0); //TODO
    private SmartDashboardNumber intakingArmPos = new SmartDashboardNumber("Arm/Positions/intaking", 0); //TODO
    private SmartDashboardNumber sourceShellArmPos = new SmartDashboardNumber("Arm/Positions/source-shell", 0); //TODO
    private SmartDashboardNumber sourceCellArmPos = new SmartDashboardNumber("Arm/Positions/source-cell", 0); //TODO
    private SmartDashboardNumber stowArmPos = new SmartDashboardNumber("Arm/Positions/stow", 0); //TODO
    private SmartDashboardNumber stoppedArmPos = new SmartDashboardNumber("Arm/Positions/stopped", 0); //TODO
    private SmartDashboardNumber minArmPos = new SmartDashboardNumber("Arm/Positions/min", 0); //TODO
    private SmartDashboardNumber maxArmPos = new SmartDashboardNumber("Arm/Positions/max", 0); //TODO
    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("Arm/Tolerance", 0); //TODO //In rotations

    //first entry is distance from target, second entry is end effector rpm
    private InterpolatingDoubleTreeMap l1RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    private InterpolatingDoubleTreeMap l2RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    private InterpolatingDoubleTreeMap l3RPMLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    
    //first entry is distance from target, second entry is arm angle
    private InterpolatingDoubleTreeMap l1ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    private InterpolatingDoubleTreeMap l2ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );
    private InterpolatingDoubleTreeMap l3ArmPosLerpTable = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.5, 0.0), 
        Map.entry(1.5, 0.0), 
        Map.entry(2.0, 0.0), 
        Map.entry(2.5, 0.0), 
        Map.entry(3.0, 0.0)                                     
    );

    public Arm(){
        super("Arm");

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(encoder.getDeviceID())
        .withRotorToSensorRatio(kRotorToSensorRatio.getNumber()) //How many motor rotations are there for every rotation of the CANCoder?
        .withSensorToMechanismRatio(kSensorToMechRatio.getNumber()); //How many CANCoder rotations are there to each mechanism rotation

        this.encoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(kDiscontinuityPoint.getNumber())
            .withMagnetOffset(kCANCoderOffset.getNumber())
        );

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
    
    private void armToPos(double pos){
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(pos, minArmPos.getNumber(), maxArmPos.getNumber()));
    }

    public boolean atWantedArmPos(double dist){
        return Math.abs(
            this.pivotMotor.motor.getPosition().getValueAsDouble() - 
            stateToArmPos(superstructure.getWantedState(), dist)
        ) > armTolerance.getNumber();
    }

    private double stateToArmPos(Superstructure.RobotState state){
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

    private double stateToArmPos(Superstructure.RobotState state, double dist){
        switch(state){
            case L1:
                return this.l1ArmPosLerpTable.get(dist);
            case L2:
                return this.l2ArmPosLerpTable.get(dist);
            case L3:
                return this.l3ArmPosLerpTable.get(dist);
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

    public Command goToArmPosCommand(Superstructure.RobotState pos, double dist){
        return Commands.runOnce(
            () -> armToPos(stateToArmPos(superstructure.getWantedState(), dist)), this
        );
    }

    @Override
    public void periodic(){
        this.pivotMotor.update();

        SmartDashboard.putBoolean("Arm/at-wanted-pos", this.atWantedArmPos(0)); //TODO add distance from peg board when have odometry
        
        StatusSignal<Angle> encoderPos = encoder.getPosition();
        encoderPos.refresh();
        SmartDashboard.putNumber("Arm/encoder/posistion-rotations", encoderPos.getValueAsDouble());

    }

    public static Arm getInstance(){
        if(instance == null) return instance = new Arm();
        return instance;
    }
}
