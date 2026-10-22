// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
<<<<<<< HEAD
import frc.robot.subsystems.scrapped.EndEffectorPrototype;

public class RobotContainer {
  private final static CommandXboxController testStick = new CommandXboxController(0);
  public final static CommandXboxController driveStick = new CommandXboxController(0);
  
  // private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
=======
import frc.robot.subsystems.EndEffectorPrototype;

public class RobotContainer {
  private final static CommandXboxController testStick = new CommandXboxController(0);

>>>>>>> 1d6aa763e8acef9d0ddc10921c3b41d925c9f198
  private final EndEffectorPrototype endEffectorPrototype = new EndEffectorPrototype();
  public RobotContainer() {
    configureBindings();
  }

<<<<<<< HEAD
  private void configureTestBindings(){
=======
  private void configureBindings(){
>>>>>>> 1d6aa763e8acef9d0ddc10921c3b41d925c9f198
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
  }

<<<<<<< HEAD
  private void configureBindings(){
    // driveStick.rightStick().onTrue(csd.seedFieldCentric());
  }

=======
>>>>>>> 1d6aa763e8acef9d0ddc10921c3b41d925c9f198
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
