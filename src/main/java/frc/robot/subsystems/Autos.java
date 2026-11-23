package frc.robot.subsystems;


import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Superstructure;
import frc.robot.Superstructure.DockColumns;
import frc.robot.Superstructure.RobotState;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardNumber;


public class Autos {
    private static Autos instance = null;

    private final Superstructure superstructure = Superstructure.getInstance();
    private final CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
    private final EndEffector endEffector = EndEffector.getInstance();
    // private final AutoFactory factory;


    private SmartDashboardNumber waitShootSec = new SmartDashboardNumber("Autos/shoot-wait", 10);


    // public Autos(){
    //     this.factory = factory;
    // }

    //Moves 5 feet at 0.5 m/s
    public Command testSlowAuto(){
        return 
            Commands.sequence(
                Commands.parallel(
                    csd.followTrajectoryCmd("testSlow"),
                    superstructure.goToArmPosStaticCommand(RobotState.L3)
                ),
                Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                    Commands.waitSeconds(this.waitShootSec.getNumber()),
                    superstructure.endEffectStaticCommand()
                )
            );
    }

    //Blue Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command Blue2ShellAuto(){
        return Commands.sequence(
            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L3),
                csd.followTrajectoryCmd("B2Shell", 0)
            ),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectStaticCommand(),
                csd.stopCommand()
            ),            
            Commands.parallel(
                endEffector.stopEndEffectorCommand(),
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("B2Shell", 1)
            ),
            Commands.deadline(
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            Commands.parallel(
                endEffector.stopEndEffectorCommand(),
                superstructure.goToArmPosStaticCommand(RobotState.L2),
                csd.followTrajectoryCmd("B2Shell", 2)
            ),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectStaticCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand()
        );
    }


    //Red Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command Red2ShellAuto(){
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L3), superstructure),
                csd.followTrajectoryCmd("R2Shell", 0)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            ),            
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.SOURCE_SHELL), superstructure),
                csd.followTrajectoryCmd("R2Shell", 1)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            superstructure.intakeEndEffectCommand(),
            Commands.parallel(
                Commands.runOnce(() ->superstructure.setWantedRobotState(RobotState.L2), superstructure),
                csd.followTrajectoryCmd("R2Shell", 2)
            ),
            Commands.waitUntil(() -> superstructure.atWantedArmPosSuperstructure()),
            Commands.deadline(      //Shouldn't need autoalign auto should correctly align
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommand()
            )
        );
    }

    public static Autos getInstance(){
        if(instance == null) instance = new Autos();
        return instance;
    }
}
