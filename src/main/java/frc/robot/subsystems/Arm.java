package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.subsystems.constants.LERPTables;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;
import frc.robot.subsystems.constants.*;

public class Arm extends SubsystemBase{
    private static Arm instance = null;

    private final Superstructure superstructure= new Superstructure();

    private final RedRockTalon pivotMotor = new RedRockTalon(0, "Arm/pivot-motor", "*");
    private final CANcoder encoder = new CANcoder(0, "*");


    //CANCoder values
    private SmartDashboardNumber kRotorToSensorRatio = new SmartDashboardNumber("Arm/CANCoder/k-rotor-to-sensor-ratio", 0); //TODO
    private SmartDashboardNumber kSensorToMechRatio = new SmartDashboardNumber("Arm/CANCoder/k-sensor-to-mech-ratio", 0); //TODO
    private SmartDashboardNumber kDiscontinuityPoint = new SmartDashboardNumber("Arm/CANCoder/k-discontinuity-point", 0.5); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc
    private SmartDashboardNumber kCANCoderOffset = new SmartDashboardNumber("Arm/CANCoder/k-cancoder-offset", 0); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc

    //Static arm poses
    private SmartDashboardNumber l1ArmPos = new SmartDashboardNumber("Arm/Positions/L1", 0); //TODO
    private SmartDashboardNumber l2ArmPos = new SmartDashboardNumber("Arm/Positions/L2", 0); //TODO
    private SmartDashboardNumber l3ArmPos = new SmartDashboardNumber("Arm/Positions/L3", 0); //TODO
    
    //Static locations
    private SmartDashboardNumber shelfArmPos = new SmartDashboardNumber("Arm/Positions/shelf", 0); //TODO
    private SmartDashboardNumber sourceShellArmPos = new SmartDashboardNumber("Arm/Positions/source-shell", 0); //TODO
    private SmartDashboardNumber sourceCellArmPos = new SmartDashboardNumber("Arm/Positions/source-cell", 0); //TODO
    private SmartDashboardNumber stowArmPos = new SmartDashboardNumber("Arm/Positions/stow", 0); //TODO
    private SmartDashboardNumber stoppedArmPos = new SmartDashboardNumber("Arm/Positions/stopped", 0); //TODO

    //Limits and Thresholds
    private SmartDashboardNumber minArmPos = new SmartDashboardNumber("Arm/Positions/min", 0); //TODO
    private SmartDashboardNumber maxArmPos = new SmartDashboardNumber("Arm/Positions/max", 0); //TODO
    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("Arm/Tolerance", 0); //TODO //In rotations
    private SmartDashboardNumber currentLimit = new SmartDashboardNumber("Arm/current-limit", 45);
    
    //Testing
    private SmartDashboardNumber testPos = new SmartDashboardNumber("Arm/test-pos", 0);
    private SmartDashboardNumber testPosChangeAmount = new SmartDashboardNumber("Arm/test-pos-change-amount", 1); 
    
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

        this.pivotMotor.motor.getConfigurator().apply(feedbackConfigs);
    }
    
    
    //Movement Methods
    private void stopArm(){
        this.pivotMotor.motor.setControl(new VoltageOut(0));
    }

    private void normalizeArm(){
        this.pivotMotor.motor.set(0.1);
    }

    private void resetArm(){
        this.pivotMotor.motor.setControl(new VoltageOut(0));
        this.encoder.setPosition(0);
    }

    private void armToPos(double pos){
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(pos, minArmPos.getNumber(), maxArmPos.getNumber()));
    }


    //Testing
    private void increaseTestPos(double amount){
        this.testPos.putNumber(this.testPos.getNumber() + amount);
    }

    private void decreaseTestPos(double amount){
        this.testPos.putNumber(this.testPos.getNumber() - amount);
    }


    //Other Methods
    public boolean atWantedArmPos(double dist){
        return Math.abs(
            this.pivotMotor.motor.getPosition().getValueAsDouble() - 
            stateToArmPos(superstructure.getWantedState(), dist)
        ) <= armTolerance.getNumber();
    }
    
    public boolean atWantedArmPosStatic(){
        return Math.abs(
            this.pivotMotor.motor.getPosition().getValueAsDouble() - 
            stateToArmPosStatic(superstructure.getWantedState())
        ) <= armTolerance.getNumber();
    }

    //Superstructure Methods
    //static arm pos for not lerping l1, l2, l3
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


    //Commands
    public Command goToPosCommand(Superstructure.RobotState pos, double dist){
        return Commands.runOnce(
            () -> armToPos(stateToArmPos(superstructure.getWantedState(), dist)), this
        );
    }

    public Command goToStaticPosCommand(Superstructure.RobotState pos){
        return Commands.runOnce(
            () -> armToPos(stateToArmPosStatic(superstructure.getWantedState())), this
        );
    }


    public Command zeroArmCommand(){
        return new FunctionalCommand(
            () -> this.normalizeArm(),
            () -> {},
            (interrupted) -> this.resetArm(),
            () -> {
                return this.pivotMotor.motor.getSupplyCurrent().getValueAsDouble() > this.currentLimit.getNumber();
            }
        );
    }


    //Testing
    public Command goToTestPosCommand(){
        return Commands.runOnce(
            () -> armToPos(testPos.getNumber()), 
            this
        );
    }
    
    public Command decreaseTestPosCommand(){
        return Commands.runOnce(
            () -> this.decreaseTestPos(testPosChangeAmount.getNumber()),
            this
        );
    }

    public Command increaseTestPosCommand(){
        return Commands.runOnce(
            () -> this.increaseTestPos(testPosChangeAmount.getNumber()),
            this
        );
    }

    public Command stopArmCommand(){
        return Commands.runOnce(
            () -> this.stopArm(), 
            this
        );
    }

    @Override
    public void periodic(){
        this.pivotMotor.update();

        SmartDashboard.putBoolean("Arm/at-wanted-pos", this.atWantedArmPos(0)); //TODO add distance from peg board when have odometry
        
        StatusSignal<Angle> encoderPos = encoder.getPosition();
        encoderPos.refresh();
        SmartDashboard.putNumber("Arm/encoder/posistion-rotations", encoderPos.getValueAsDouble());
    
        SmartDashboard.putNumber("Arm/supply-current", this.pivotMotor.motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/velocity", this.pivotMotor.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Arm/acceleration", this.pivotMotor.motor.getAcceleration().getValueAsDouble());
    }

    public static Arm getInstance(){
        if(instance == null) return instance = new Arm();
        return instance;
    }
}
