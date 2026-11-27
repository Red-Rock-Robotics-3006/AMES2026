package frc.robot.subsystems;


import java.util.Set;

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


    private SmartDashboardNumber waitShootSec = new SmartDashboardNumber("Autos/shoot-wait", 1);
    private SmartDashboardNumber waitIntakeSec = new SmartDashboardNumber("Autos/shoot-wait", 3);


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
                    superstructure.endEffectCommandBoth()
                ),
                endEffector.stopEndEffectorCommand()
            );
    }

    //Blue Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command blue2ShellAuto(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),
            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L3),
                csd.followTrajectoryCmd("B2Shell", 0)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("B2Shell", 1)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L2),
                csd.followTrajectoryCmd("B2Shell", 2)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),
            
            Commands.deadline(     
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand()
        );
    }


    //Red Side starts bottom corner shoots preloaded shell L3, source intakes shell and shoots it again L2
    public Command red2ShellAuto(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),
            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L3),
                csd.followTrajectoryCmd("R2Shell", 0)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("R2Shell", 1)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L2),
                csd.followTrajectoryCmd("R2Shell", 2)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),
            
            Commands.deadline(     
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand()
        );
    }

    public Command blue3ShellAuto(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),
            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L3),
                csd.followTrajectoryCmd("B3Shell", 0)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("B3Shell", 1)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L1),
                csd.followTrajectoryCmd("B3Shell", 2)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(     
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

             Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("B3Shell", 3)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L2),
                csd.followTrajectoryCmd("B3Shell", 4)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand()
        );
    }

    public Command red3ShellAuto(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),
            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L3),
                csd.followTrajectoryCmd("R3Shell", 0)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("R3Shell", 1)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L1),
                csd.followTrajectoryCmd("R3Shell", 2)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(     
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

             Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SOURCE_SHELL),
                csd.followTrajectoryCmd("R3Shell", 3)
            ),

            Commands.deadline(
                Commands.waitSeconds(this.waitIntakeSec.getNumber()),
                endEffector.runIntakeTestCommand(),
                csd.stopCommand()
            ),
            endEffector.stopEndEffectorCommand(),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.L2),
                csd.followTrajectoryCmd("R3Shell", 4)
            ),
            csd.stopCommand(),

            Commands.parallel(
                Commands.runOnce(() -> superstructure.setDockTarget(DockColumns.LEFT_MID)),
                Commands.defer(() -> superstructure.fullAutoAlignCommand(), Set.of(csd))
            ).finallyDo(() -> superstructure.setDockTarget(DockColumns.NONE)),
            csd.stopCommand(),

            Commands.deadline(      
                Commands.waitSeconds(this.waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth()
            ),            
            endEffector.stopEndEffectorCommand()
        );
    }

    public Command redShelf1(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SHELF),
                csd.followTrajectoryCmd("RShelf1", 0)
            ),
            csd.stopCommand(),

            Commands.deadline(
                Commands.waitSeconds(waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth() 
            ),
            endEffector.stopEndEffectorCommand()
        );
    }

    public Command blueShelf1(){
        return Commands.sequence(
            Commands.runOnce(() ->csd.seedFieldCentric()),

            Commands.parallel(
                superstructure.goToArmPosStaticCommand(RobotState.SHELF),
                csd.followTrajectoryCmd("BShelf1", 0)
            ),
            csd.stopCommand(),

            Commands.deadline(
                Commands.waitSeconds(waitShootSec.getNumber()),
                superstructure.endEffectCommandBoth() 
            ),
            endEffector.stopEndEffectorCommand()
        );
    }

    public static Autos getInstance(){
        if(instance == null) instance = new Autos();
        return instance;
    }
}
