// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.DockColumns;
import frc.robot.Superstructure.RobotState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.EndEffector;
// import frc.robot.subsystems.EndEffectorPrototype;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Odometry;
import redrocklib.logging.SmartDashboardNumber;

import com.ctre.phoenix6.signals.RobotEnableValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;


public class RobotContainer {
  private final static CommandXboxController testStick = new CommandXboxController(0);
  public final static CommandXboxController driveStick = new CommandXboxController(3);
  public final static CommandXboxController mechStick = new CommandXboxController(1);
  
  private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
  private SmartDashboardNumber testEEvel = new SmartDashboardNumber("RobotContainer/testEEvel", 0);
  // private final EndEffectorPrototype endEffectorPrototype = EndEffectorPrototype.getInstance();
  private final Superstructure superstructure = Superstructure.getInstance();
  private final Intake intake = Intake.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final Arm arm = Arm.getInstance();
  private final Odometry odometry = Odometry.getInstance();
  private final SwerveRequest.FieldCentric csdRequest = new SwerveRequest.FieldCentric().withDeadband(0.1);
  // private final Autos auto = new Auto();
  private SmartDashboardNumber maxTranslationProportion = new SmartDashboardNumber("CommandSwerveDrivetrain/maxTranslationProportion", 0.70);
  private SmartDashboardNumber maxRotationProportion = new SmartDashboardNumber("CommandSwerveDrivetrain/maxRotationProportion", 0.70);
  
  private final static Autos autos = Autos.getInstance();
  private SendableChooser<Command> chooser = new SendableChooser<>();

  private double distToTarget;

  public RobotContainer() {
    configureTestBindings();
    // configureBindings();
    configureAutoChooser();
  }

  private void configureAutoChooser(){
    chooser.setDefaultOption("noAuto", Commands.runOnce(() -> System.out.println("no auto")));

    chooser.addOption("testAuto", autos.testSlowAuto());
    chooser.addOption("Blue2ShellAuto", autos.Blue2ShellAuto());
    chooser.addOption("Red2ShellAuto", autos.Red2ShellAuto());

    SmartDashboard.putData("Auto Chooser", chooser);
  }
  
  private void configureBindings(){
    csd.setDefaultCommand(csd.applyRequest(() -> csdRequest.withVelocityX(-driveStick.getLeftY()*maxTranslationProportion.getNumber())
    													   .withVelocityY(-driveStick.getLeftX()*maxTranslationProportion.getNumber())
														   .withRotationalRate(-driveStick.getRightX()*maxRotationProportion.getNumber())));
    
    driveStick.rightStick().onTrue(Commands.runOnce(() ->csd.seedFieldCentric()));

    // Drivestick bindings
    driveStick.leftTrigger(0.25) //Shoots with lerp
      .onTrue(
        superstructure.endEffectCommand()
          .onlyIf(() -> odometry.get2dDistToTargetAllCams(superstructure.getIntDockCol(superstructure.getDockColumn())) <= 5))
      .onFalse(endEffector.stopEndEffectorCommand());
    
    driveStick.rightTrigger(0.25)
      .onTrue(Commands.sequence(
        endEffector.intakeCommand(), 
        intake.startIntakeCommand())
      )
      .onFalse(intake.stopIntakeCommand());
    driveStick.rightBumper()
      .onTrue(endEffector.outakeCommand())
      .onFalse(endEffector.stopEndEffectorCommand());

    //Set which column of dock targets to target when auto aligning
    mechStick.povCenter()
      	.onTrue(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.NONE)));
    mechStick.povUp()
      .onTrue(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT), superstructure))
      .onFalse(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.NONE), superstructure));

    mechStick.povLeft()
      .onTrue(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID), superstructure))
      .onFalse(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.NONE), superstructure));

    mechStick.povRight()
      .onTrue(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.RIGHT_MID), superstructure))
      .onFalse(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.NONE), superstructure));
    mechStick.povDown()
      .onTrue(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.RIGHT), superstructure))
      .onFalse(Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.NONE), superstructure));

	//x=L1, y=L2, b=L3, a=shelf
    mechStick.x()
      .onTrue(superstructure.goToArmPosCommand(RobotState.L1, superstructure.getDistToTarget()))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_SHELL));
    mechStick.y()
      .onTrue(superstructure.goToArmPosCommand(RobotState.L2, superstructure.getDistToTarget()))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_SHELL));
    mechStick.b()
	    .onTrue(superstructure.goToArmPosCommand(RobotState.L3, superstructure.getDistToTarget()))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_SHELL));
    mechStick.a()
	    .onTrue(superstructure.goToArmPosStaticCommand(RobotState.SHELF))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_SHELL));
    mechStick.rightBumper()
	    .onTrue(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_CELL))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTERMEDIATE));
    mechStick.leftBumper()
	    .onTrue(superstructure.goToArmPosStaticCommand(RobotState.INTAKING_SHELL))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.INTERMEDIATE));

    mechStick.leftTrigger(0.25) 
      .onTrue(
        Commands.sequence(
          superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
          endEffector.intakeCommand()
      ))
      .onFalse(
        Commands.sequence(
          superstructure.goToArmPosStaticCommand(RobotState.INTERMEDIATE),
          Commands.runOnce(() -> endEffector.stopEndEffectorCommand(), endEffector)
        )   
      );
    mechStick.rightTrigger(0.25) 
      .onTrue(
        Commands.sequence(
          superstructure.goToArmPosStaticCommand(RobotState.SOURCE_CELL),
          endEffector.intakeCommand()
      ))
      .onFalse(
        Commands.sequence(
          superstructure.goToArmPosStaticCommand(RobotState.INTERMEDIATE),
          Commands.runOnce(() -> endEffector.stopEndEffectorCommand(), endEffector)
        )   
      ); 
	}

  private void configureTestBindings(){
    testStick.rightTrigger()
	    .onTrue(superstructure.goToArmPosStaticCommand(RobotState.TEST))
      .onFalse(superstructure.goToArmPosStaticCommand(RobotState.STOW));
    // testStick.rightTrigger().onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.TEST)));
    testStick.rightBumper().onTrue(arm.zeroArmCommand());
    testStick.leftBumper().onTrue(arm.zeroArmCommandMove());
    testStick.leftTrigger().onTrue(arm.goToTestPosCommand());
    
    testStick.x().onTrue(endEffector.runIntakeTestCommand()).onFalse(endEffector.stopEndEffectorCommand());
    testStick.y().onTrue(endEffector.shootTestCommand()).onFalse(endEffector.stopIntakeTestCommandDutyCycle());
    // testStick.y().onTrue(endEffector.shootTestCommand()).onFalse(endEffector.stopIntakeTestCommandDutyCycle());
    // testStick.y()
    //   .onTrue(endEffectorPrototype.increaseEndEffectorSpeedTestCommand())
    //   .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    // testStick.a()
    //   .onTrue(endEffectorPrototype.decreaseEndEffectorSpeedTestCommand())
    //   .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());
    // testStick.x()
    //   .onTrue(endEffectorPrototype.startEndEffectorTestReverseCommand())
    //   .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    // testStick.b()
    //   .onTrue(endEffectorPrototype.startEndEffectorTestCommand())
    //   .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

      
    // testStick.rightTrigger()
    //   .onTrue(intake.startIntakeCommand())
    //   .onFalse(intake.stopIntakeCommand());
    // testStick.leftTrigger()
    //   .onTrue(intake.reverseIntakeCommand())
    //   .onFalse(intake.stopIntakeCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No auto configured");
  }
}
