package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class Autos {
    private static Autos instance = null;

    private final Superstructure superstructure = Superstructure.getInstance();
    private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();

    private final AutoFactory factory;

    public Autos(AutoFactory factory){
        this.factory = factory;
    }

    public Command Blue2ShellAuto(){
        return Commands.sequence(
            
        );
    }
}
