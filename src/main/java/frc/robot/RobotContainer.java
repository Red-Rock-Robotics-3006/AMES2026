// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.DockColumns;
import frc.robot.Superstructure.RobotState;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.EndEffectorPrototype;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.signals.RobotEnableValue;
import com.ctre.phoenix6.swerve.SwerveRequest;


public class RobotContainer {
  private final static CommandXboxController testStick = new CommandXboxController(0);
  public final static CommandXboxController driveStick = new CommandXboxController(0);
  public final static CommandXboxController mechStick = new CommandXboxController(0);
  
  private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
  private final EndEffectorPrototype endEffectorPrototype = new EndEffectorPrototype();
  private final Superstructure superstructure = Superstructure.getInstance();
  private final Intake intake = new Intake();
  private final EndEffector endEffector = EndEffector.getInstance();

  private final SwerveRequest.FieldCentric csdRequest = new SwerveRequest.FieldCentric();
  public RobotContainer() {
    // configureTestBindings();
    configureBindings();
  }
  
  private void configureBindings(){
    // driveStick.rightStick().onTrue(csd.seedFieldCentric());

    //Drivestick bindings
    driveStick.leftTrigger(0.25) //Shoots with lerp
		.onTrue(superstructure.endEffectCommand())
		.onFalse(endEffector.stopEndEffectorCommand());
	driveStick.rightTrigger(0.25)
		.onTrue(Commands.sequence(
			endEffector.intakeCommand(), 
			intake.startIntakeCommand())
		)
		.onFalse(intake.stopIntakeCommand());

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
		.onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.L1), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.INTAKING_SHELL), superstructure));
    mechStick.y()
		.onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.L2), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.INTAKING_SHELL), superstructure));
    mechStick.b()
	    .onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.L3), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.INTAKING_SHELL), superstructure));
    mechStick.a()
	    .onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.SHELF), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.INTAKING_SHELL), superstructure));

	mechStick.leftTrigger(0.25) 
		.onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.SOURCE_SHELL), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.IDLE), superstructure)); //Stops all subsystems
	mechStick.rightTrigger(0.25) 
		.onTrue(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.SOURCE_CELL), superstructure))
		.onFalse(Commands.runOnce(() -> superstructure.setWantedRobotState(RobotState.IDLE), superstructure));
  }






  
  private void configureTestBindings(){
    testStick.y()
      .onTrue(endEffectorPrototype.increaseEndEffectorSpeedTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    testStick.a()
      .onTrue(endEffectorPrototype.decreaseEndEffectorSpeedTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());
    testStick.x()
      .onTrue(endEffectorPrototype.startEndEffectorTestReverseCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    testStick.b()
      .onTrue(endEffectorPrototype.startEndEffectorTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

      
    testStick.rightTrigger()
      .onTrue(intake.startIntakeCommand())
      .onFalse(intake.stopIntakeCommand());
    testStick.leftTrigger()
      .onTrue(intake.reverseIntakeCommand())
      .onFalse(intake.stopIntakeCommand());
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
