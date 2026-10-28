// package frc.robot.subsystems.vision;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.Vector;
// import java.util.function.Supplier;

// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.epilogue.Logged.Strategy;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.numbers.N3; 
// import edu.wpi.first.math.numbers.N1; 
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Superstructure.DockColumns;
// import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;


// public class Odometry extends SubsystemBase{
//     private static Odometry instance = null;

//     private CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();

//     private static ArrayList<Camera> cameras = new ArrayList<>();
//     private final PhotonPoseEstimator[] photonPoseEstimators;

//     private final SwerveDrivePoseEstimator posEstimator;
//     private final Supplier<SwerveDriveKinematics> kinematics;
//     private final Supplier<Rotation2d> gyRotation2d;
//     private final Supplier<SwerveModulePosition[]> modulePositions;
//     private static final Matrix<N3, N1> stateStdDevs  = VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(2));
//     private static final Matrix<N3, N1> visionStdDevs  = VecBuilder.fill(0.05, 0.05, 99999999);

//     private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //Change to load custom field

//     private PhotonPoseEstimator posEstimate1;
//     private PhotonPoseEstimator posEstimate2;
//     private PhotonPoseEstimator posEstimate3;
   
//     public Odometry(){
//         super("Odometry");

//         this.kinematics = csd.getKinematics();
//         this.gyRotation2d = csd.getGyroRotation2dSupplier();
//         this.modulePositions = csd.getModulePosesSupplier();

//         posEstimator = new SwerveDrivePoseEstimator(
//             this.kinematics.get(), 
//             this.gyRotation2d.get(), 
//             this.modulePositions.get(), 
//             new Pose2d()
//         );

//         cameras.add(new Camera("Name1", this.gyRotation2d, 0, 0, 0, 0, 0, 0)); //TODO name and add vals to cams
//         cameras.add(new Camera("Name2", this.gyRotation2d, 0, 0, 0, 0, 0, 0));
//         cameras.add(new Camera("Name3", this.gyRotation2d, 0, 0, 0, 0, 0, 0));
//         cameras.add(new Camera("Name4", this.gyRotation2d, 0, 0, 0, 0, 0, 0));
    
//         for(int i = 0; i < cameras.size(); i++){
//             photonPoseEstimators[i] = new PhotonPoseEstimator(field, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, cameras.get(i).getRobotToCamera());
//             //Depricated method, TODO find another way to do lowest ambiguity
//             // photonPoseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//         }
//     }

//     public Double get2dDistToTargetAllCams(int targetCol){
//         for(Camera cam: cameras){
//             if(cam.getValidTagMeasurements() != null)
//                 return cam.getDistToTarget(targetCol);
//         }
//         return null;
//     }

//     public Rotation2d get2dYawToTargetAllCams(int target){
//         for(Camera cam: cameras){
//             if(cam.getValidTagMeasurements() != null)
//                 return cam.getYawToTarget(target);
//         }
//         return null;
//     }

//     @Override
//     public void periodic(){
//         posEstimator.update(gyRotation2d.get(), modulePositions.get());

//         for(Camera camera: cameras){
//             List<PhotonPipelineResult> unreadResults = camera.getPhotonCamera().getAllUnreadResults();
//             if(unreadResults.get(unreadResults.size()-1).hasTargets()){
//                 posEstimator.addVisionMeasurement(null, 0);
//             }
//         }
//     }

//     public static Odometry getInstance(){
//         if(instance == null) instance = new Odometry();
//         return instance;
//     }

// }