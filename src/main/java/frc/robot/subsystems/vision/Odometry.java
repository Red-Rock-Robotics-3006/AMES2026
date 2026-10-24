package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.DockColumns;


public class Odometry extends SubsystemBase{
    private static Odometry instance = null;

    private static ArrayList<Camera> cameras = new ArrayList<>();
    private final SwerveDrivePoseEstimator posEstimator;
    private final Supplier<SwerveDriveKinematics> kinematics;
    private final Supplier<Rotation2d> gyRotation2d;
    private final Supplier<SwerveModulePosition[]> modulePositions;

    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //Change to load custom field

    private PhotonPoseEstimator posEstimate1;
    private PhotonPoseEstimator posEstimate2;
    private PhotonPoseEstimator posEstimate3;
   
    public Odometry(Supplier<SwerveDriveKinematics> kinematics, Supplier<Rotation2d> gyroRotation2d, Supplier<SwerveModulePosition[]> modulePositions){
        super("Odometry");

        this.kinematics = kinematics;
        this.gyRotation2d = gyroRotation2d;
        this.modulePositions = modulePositions;
        posEstimator = new SwerveDrivePoseEstimator(
            this.kinematics.get(), 
            this.gyRotation2d.get(), 
            this.modulePositions.get(), 
            new Pose2d()
        );

        cameras.add(new Camera("Name", 0, 0, 0, 0, 0, 0)); //TODO name and add vals to cams
        cameras.add(new Camera("Name", 0, 0, 0, 0, 0, 0));
        cameras.add(new Camera("Name", 0, 0, 0, 0, 0, 0));
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
        posEstimator.update(gyRotation2d.get(), modulePositions.get());

        for(Camera camera: cameras){
            List<PhotonPipelineResult> unreadResults = camera.getPhotonCamera().getAllUnreadResults();
            if(unreadResults.get(unreadResults.size()-1).hasTargets()){
                posEstimator.addVisionMeasurement(null, 0);
            }
        }
    }

    public static Odometry getInstance(Supplier<SwerveDriveKinematics> kin, Supplier<Rotation2d> gyro2d, Supplier<SwerveModulePosition[]> modulePoses){
        if(instance == null) instance = new Odometry(kin, gyro2d, modulePoses);
        return instance;
    }
}