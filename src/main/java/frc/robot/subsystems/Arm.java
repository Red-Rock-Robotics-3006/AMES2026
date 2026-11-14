package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Arm extends SubsystemBase{
    private static Arm instance = null;

    private static EndEffector endeffector = EndEffector.getInstance();
    // private final Superstructure superstructure= new Superstructure();

    private final RedRockTalon pivotMotor = new RedRockTalon(21, "Arm/pivot-motor");
    // private final CANcoder encoder = new CANcoder(25, "*");


    //CANCoder values
    // private SmartDashboardNumber kRotorToSensorRatio = new SmartDashboardNumber("Arm/CANCoder/k-rotor-to-sensor-ratio", 0); //TODO
    // private SmartDashboardNumber kSensorToMechRatio = new SmartDashboardNumber("Arm/CANCoder/k-sensor-to-mech-ratio", 0); //TODO
    // private SmartDashboardNumber kDiscontinuityPoint = new SmartDashboardNumber("Arm/CANCoder/k-discontinuity-point", 0.5); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc
    // private SmartDashboardNumber kCANCoderOffset = new SmartDashboardNumber("Arm/CANCoder/k-cancoder-offset", 0); //TODO gives range of rotation, so 0.7 would be -0.3, 0.7, 1 would be 0, 1 etc

    //Limits and Thresholds
    private SmartDashboardNumber minArmPos = new SmartDashboardNumber("Arm/Positions/min", 0); 
    private SmartDashboardNumber maxArmPos = new SmartDashboardNumber("Arm/Positions/max", 0.64); 
    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("Arm/Tolerance", 0.01); 
    private SmartDashboardNumber currentLimit = new SmartDashboardNumber("Arm/current-limit", 30);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("Arm/normalize-speed", -0.2);
    
    //Testing
    private SmartDashboardNumber testPosChangeAmount = new SmartDashboardNumber("Arm/test-pos-change-amount", 0.01); 
    private SmartDashboardNumber testPosNoSuperstructure = new SmartDashboardNumber("Arm/test-pos-no-superstructure", 0.25); 
    
    public Arm(){
        super("Arm");

        // AutoLogOutputManager.addObject(this);
        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        // .withFeedbackRemoteSensorID(encoder.getDeviceID())
        // .withRotorToSensorRatio(kRotorToSensorRatio.getNumber()) //How many motor rotations are there for every rotation of the CANCoder?
        // .withSensorToMechanismRatio(kSensorToMechRatio.getNumber()); //How many CANCoder rotations are there to each mechanism rotation

        // this.encoder.getConfigurator().apply(
        //     new MagnetSensorConfigs()
        //     .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        //     .withAbsoluteSensorDiscontinuityPoint(kDiscontinuityPoint.getNumber())
        //     .withMagnetOffset(kCANCoderOffset.getNumber())
        // );

        this.pivotMotor
        .withFeedbackConfigs(
            new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(64)
        )
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) 
            .withKS(0.4) 
            .withKV(0)
            .withKP(150)
            .withKI(0) 
            .withKD(3) 
            .withKG(0.2)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(500)
            .withMotionMagicCruiseVelocity(0.6)
            .withMotionMagicJerk(0)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(30)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);

        // this.pivotMotor.motor.getConfigurator().apply(feedbackConfigs);
    }
    
    
    //Movement Methods
    private void stopArm(){
        this.pivotMotor.motor.setControl(new VoltageOut(0));
    }

    private void normalizeArm(){
        this.pivotMotor.motor.set(this.normalizeSpeed.getNumber());
    }

    private void resetArm(){
        this.pivotMotor.motor.setControl(new VoltageOut(0));
        this.pivotMotor.motor.setPosition(0);
    }

    public void armToPos(double pos){
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(pos, minArmPos.getNumber(), maxArmPos.getNumber()));
        // Logger.recordOutput("Arm/desiredArmPos", pos);
    }

    public double getPivotVelocity(){
        return this.pivotMotor.motor.getVelocity().getValueAsDouble();
    }

    //Testing
    // private void increaseTestPos(double amount){
    //     this.testPos.putNumber(this.testPos.getNumber() + amount);
    // }

    // private void decreaseTestPos(double amount){
    //     this.testPos.putNumber(this.testPos.getNumber() - amount);
    // }


    //Other Methods
    public boolean atWantedArmPos(double pos){ 
        return Math.abs(
            this.pivotMotor.motor.getPosition().getValueAsDouble() - 
            pos
        ) <= armTolerance.getNumber();
    }

    //Commands

    public Command zeroArmCommandMove(){
        return new FunctionalCommand(
            () -> this.normalizeArm(),
            () -> {},
            (interrupted) -> this.resetArm(),
            () -> {
                return this.pivotMotor.motor.getSupplyCurrent().getValueAsDouble() > this.currentLimit.getNumber();
            },
            this
        );
    }

    public Command zeroArmCommand(){
        return Commands.runOnce(() -> this.resetArm(), this);
    }


    //Testing
    public Command goToTestPosCommand(){
        return Commands.runOnce(
            () -> armToPos(this.testPosNoSuperstructure.getNumber()), 
            this
        );
    }
    
    // public Command decreaseTestPosCommand(){
    //     return Commands.runOnce(
    //         () -> this.decreaseTestPos(testPosChangeAmount.getNumber()),
    //         this
    //     );
    // }

    // public Command increaseTestPosCommand(){
    //     return Commands.runOnce(
    //         () -> this.increaseTestPos(testPosChangeAmount.getNumber()),
    //         this
    //     );
    // }

    public Command stopArmCommand(){
        return Commands.runOnce(
            () -> this.stopArm(), 
            this
        );
    }

    @Override
    public void periodic(){
        this.pivotMotor.update();

        

        // StatusSignal<Angle> encoderPos = encoder.getPosition();
        // encoderPos.refresh();
        // SmartDashboard.putNumber("Arm/encoder/posistion-rotations", encoderPos.getValueAsDouble());
        // Logger.recordOutput("Arm/encoder/posistion-rotations", encoderPos.getValueAsDouble());
    
        SmartDashboard.putNumber("Arm/supply-current", this.pivotMotor.motor.getSupplyCurrent().getValueAsDouble());
        // Logger.recordOutput("Arm/supply-current", this.pivotMotor.motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/velocity", this.pivotMotor.motor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Arm/currentPose", this.pivotMotor.motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm/currentPos", this.pivotMotor.motor.getPosition().getValueAsDouble());
        // Logger.recordOutput("Arm/velocity", this.pivotMotor.motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Arm/acceleration", this.pivotMotor.motor.getAcceleration().getValueAsDouble());
        // Logger.recordOutput("Arm/acceleration", this.pivotMotor.motor.getAcceleration().getValueAsDouble());
    }

    public static Arm getInstance(){
        if(instance == null) instance = new Arm();
        return instance;
    }
}
