package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.function.Supplier;

<<<<<<< Updated upstream
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3; 
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes
import edu.wpi.first.math.numbers.N1; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.DockColumns;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;


public class Odometry extends SubsystemBase{
    private static Odometry instance = null;

    private CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();

    private static ArrayList<Camera> cameras = new ArrayList<>();
   
    private static final Matrix<N3, N1> visionStdDevs  = VecBuilder.fill(0.05, 0.05, 99999999);

   
    public Odometry(){
        super("Odometry");

        Supplier<Rotation2d> gyrRotation = () -> csd.getState().Pose.getRotation();
<<<<<<< Updated upstream
        cameras.add(new Camera("Name1", gyrRotation, 0, 0, 0, 0, 0, 0)); //TODO name and add vals to cams
        cameras.add(new Camera("Name2", gyrRotation, 0, 0, 0, 0, 0, 0));
=======
        cameras.add(new Camera("LeftCam", gyrRotation, -.287, -.315, 0.19, 180, -Units.degreesToRadians(70), Units.degreesToRadians(-20))); //TODO name and add vals to cams
        cameras.add(new Camera("RightCam", gyrRotation, .287, -.315, 0.19, 180, -Units.degreesToRadians(70), Units.degreesToRadians(20)));
>>>>>>> Stashed changes
    
    }

    public Double get2dDistToTargetAllCams(int targetCol){
        for(Camera cam: cameras){
            if(cam.getValidTagMeasurements() != null)
                return cam.getDistToTarget(targetCol);
        }
        return null;
    }

    public Rotation2d get2dYawToTargetAllCams(int target){
        for(Camera cam: cameras){
            if(cam.getValidTagMeasurements() != null)
                return cam.getYawToTarget(target);
        }
        return null;
    }

    @Override
    public void periodic(){
        //For autos adds vision measurement when close to game element to hopefully reduce shifting pos
        for(Camera camera: cameras){
            Pose2d pos = camera.getRobotPose2d();
            if(pos != null)
               csd.addVisionMeasurement(pos, camera.getLatestTimeStamp(), visionStdDevs);
        }
<<<<<<< Updated upstream

    }

    public static Odometry getInstance(){
        if(instance == null) instance = new Odometry();
        return instance;
    }

=======
    }

    public static Odometry getInstance(){
        if(instance == null) instance = new Odometry();
        return instance;
    }

>>>>>>> Stashed changes
}