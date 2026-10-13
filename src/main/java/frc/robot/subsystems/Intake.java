package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    public Intake(){

    }

    @Override
    public void periodic(){

    }

    public static Intake getInstance(){
        if(instance == null) instance = new Intake();
        return instance;
    }
}
