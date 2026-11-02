package frc.robot.subsystems;


import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure;
import frc.robot.Superstructure.DockColumns;
import frc.robot.Superstructure.RobotState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardNumber;


public class Autos {


    private final Superstructure superstructure = Superstructure.getInstance();
    private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();


    private final AutoFactory factory;


    private SmartDashboardNumber waitShootSec = new SmartDashboardNumber("Autos/shoot-wait", 2);


    public Autos(AutoFactory factory){
        this.factory = factory;
    }

    //Moves 5 feet at 1 m/s
    public Command testSlowAuto(){
        return Commands.sequence(
            factory.trajectoryCmd("testSlow", 0)
        );
    }

    //Blue Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command Blue2ShellAuto(){
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L3), superstructure),
                factory.trajectoryCmd("B2Shell", 0)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            ),            
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.SOURCE_SHELL), superstructure),
                factory.trajectoryCmd("B2Shell", 1)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            superstructure.intakeEndEffectCommand(),
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L2), superstructure),
                factory.trajectoryCmd("B2Shell", 2)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            )
        );
    }


    //Red Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command Red2ShellAuto(){
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L3), superstructure),
                factory.trajectoryCmd("R2Shell", 0)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            ),            
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.SOURCE_SHELL), superstructure),
                factory.trajectoryCmd("R2Shell", 1)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            superstructure.intakeEndEffectCommand(),
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L2), superstructure),
                factory.trajectoryCmd("R2Shell", 2)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            )
        );
    }
}
