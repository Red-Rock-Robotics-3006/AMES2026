// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import redrocklib.logging.SmartDashboardNumber;
// import redrocklib.wrappers.RedRockTalon;

// public class Intake extends SubsystemBase {
//     private static Intake instance = null;

//     public static final boolean kEnableIntakeTuning = true;

//     private RedRockTalon intakeMotorOne = new RedRockTalon(0, "intake-motor-one", "*"); //TODO: Change the ID to the correct one
//     private RedRockTalon intakeMotorTwo = new RedRockTalon(0, "intake-motor-two", "*"); //TODO: Change the ID to the correct one

//     private SmartDashboardNumber intakeTargetSpeed = new SmartDashboardNumber("intake/target/intake speed", 0, kEnableIntakeTuning && true); //TODO: Change the default value to the correct one
//     private SmartDashboardNumber intakeTargetSpeedRatio = new SmartDashboardNumber("intake/target/intake speed ratio", 0, kEnableIntakeTuning && true); //TODO: Change the default value to the correct one //ratio of the speed of motor 1 to motor 2
//     private SmartDashboardNumber intakeTargetReverseSpeed = new SmartDashboardNumber("intake/target/intake reverse speed", 0, kEnableIntakeTuning && true); //TODO: Change the default value to the correct one

//     public Intake() {
//         super();

//         this.intakeMotorOne.withTuningEnabled(kEnableIntakeTuning && true)
//         .withMotorOutputConfigs(
//             new MotorOutputConfigs()
//             .withInverted(InvertedValue.Clockwise_Positive)
//             .withPeakForwardDutyCycle(1d)
//             .withPeakReverseDutyCycle(-1d)
//             .withNeutralMode(NeutralModeValue.Brake)
//         ).withCurrentLimitConfigs(new CurrentLimitsConfigs()
//             .withSupplyCurrentLimit(40)
//             .withSupplyCurrentLimitEnable(true)
//             .withStatorCurrentLimit(80)
//             .withStatorCurrentLimitEnable(true)
//         );

//         this.intakeMotorTwo.withTuningEnabled(kEnableIntakeTuning && true)
//         .withMotorOutputConfigs(
//             new MotorOutputConfigs()
//             .withInverted(InvertedValue.Clockwise_Positive)
//             .withPeakForwardDutyCycle(1d)
//             .withPeakReverseDutyCycle(-1d)
//             .withNeutralMode(NeutralModeValue.Brake)
//         ).withCurrentLimitConfigs(new CurrentLimitsConfigs()
//             .withSupplyCurrentLimit(40)
//             .withSupplyCurrentLimitEnable(true)
//             .withStatorCurrentLimit(80)
//             .withStatorCurrentLimitEnable(true)
//         );
//     }

//     public void setintakeTargetSpeed(double speed) {
//         this.intakeMotorOne.motor.setControl(new DutyCycleOut(speed).withEnableFOC(false));
//         this.intakeMotorTwo.motor.setControl(new DutyCycleOut(speed * intakeTargetSpeedRatio.getNumber()).withEnableFOC(false));
//     }

//     public void stopIntake() {
//         this.setintakeTargetSpeed(0);
//     }

//     public void startIntake() {
//         this.setintakeTargetSpeed(intakeTargetSpeed.getNumber());
//     }

//     public void reverseIntake() {
//         this.setintakeTargetSpeed(intakeTargetReverseSpeed.getNumber());
//     }

//     public Command startIntakeCommand() {
//         return Commands.runOnce(() -> this.startIntake(), this);
//     }

//     public Command stopIntakeCommand() {
//         return Commands.runOnce(() -> this.stopIntake(), this);
//     }

//     public Command reverseIntakeCommand() {
//         return Commands.runOnce(() -> this.reverseIntake(), this);
//     }


//     @Override
//     public void periodic() {
//         this.intakeMotorOne.update();
//         this.intakeMotorTwo.update();

//         if (kEnableIntakeTuning) {
//             SmartDashboard.putNumber("intake/real monitoring/motor one speed",intakeMotorOne.motor.getVelocity().getValueAsDouble());
//             SmartDashboard.putNumber("intale/real monitoring/motor two speed", intakeMotorTwo.motor.getVelocity().getValueAsDouble());
//             SmartDashboard.putNumber("intake/real monitoring/speed ratio", intakeMotorTwo.motor.getVelocity().getValueAsDouble()/intakeMotorOne.motor.getVelocity().getValueAsDouble());
//         }
//     }

//     public static Intake getInstance() {
//         if (instance == null) instance = new Intake();
//         return instance;
//     }

// }
