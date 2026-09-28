// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.EndEffectorPrototype;

public class RobotContainer {
  private final static CommandXboxController testStick = new CommandXboxController(0);

  private final EndEffectorPrototype endEffectorPrototype = new EndEffectorPrototype();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings(){
    testStick.y()
      .onTrue(endEffectorPrototype.increaseEndEffectorSpeedTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    testStick.a()
      .onTrue(endEffectorPrototype.decreaseEndEffectorSpeedTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());

    testStick.b()
      .onTrue(endEffectorPrototype.startEndEffectorTestCommand())
      .onFalse(endEffectorPrototype.stopEndEffectorTestCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
