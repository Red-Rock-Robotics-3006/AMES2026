package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.DockColumns;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;


public class Camera extends SubsystemBase{
    private PhotonCamera cam;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Pose2d visionPos;
    private Pose2d robotPos;
    private int[] validTags = {4, 8};
    private Transform3d robotToCamera;


    private Pose3d[] tagPose3ds = { //TODO use field cad
        new Pose3d(new Translation3d(9.3340855482, 5.0668855482, 0.6096), new Rotation3d(Math.PI/2, 0, (3*Math.PI)/4)), //red (roll: 90 deg, yaw: 135 deg)
        new Pose3d(new Translation3d(7.1251144518, 2.8579144518, 0.6096), new Rotation3d(Math.PI/2, 0, -Math.PI/4))  //blue (roll: 90 deg, yaw: 135 deg)  
    };


    private Pose2d[] blueDockTargets = { //TODO use field cad
        new Pose2d(6.558322799,  3.3726817484, Rotation2d.fromDegrees(-45)), //furthest to the left facing dock
        new Pose2d(6.923765807, 3.0103228592, Rotation2d.fromDegrees(-45)),
        new Pose2d(7.2767015756, 2.6513847404, Rotation2d.fromDegrees(-45)),
        new Pose2d(7.6421862904, 2.2953930978, Rotation2d.fromDegrees(-45))
    };


    private Pose2d[] redDockTargets = {
        new Pose2d(9.8613397134, 4.5168199732, Rotation2d.fromDegrees(135)), //furthest to the left looking at the doc
        new Pose2d(9.5021361392, 4.8759766844, Rotation2d.fromDegrees(135)),
        new Pose2d(9.1429333778, 5.2351532838, Rotation2d.fromDegrees(135)),
        new Pose2d(8.7837315308, 5.5943176404, Rotation2d.fromDegrees(135))
    };


    private Double distToTag = null;
    private double cameraHeight; //meters
    private double cameraPitch; //radians


    //timetamp stuff for target
    private double latestResultTimeStamp = 0;
    private SmartDashboardNumber oldThreshold = new SmartDashboardNumber("Camera/target-too-old-threshold", 0.25);

    private Supplier<Rotation2d> gyroAngle;

    private SmartDashboardNumber maxAmbiguity = new SmartDashboardNumber("Camera/max-ambiguity", 0.2);
    // private SmartDashboardNumber maxDistFromTag = new SmartDashboardNumber("Camera/max-distance-from-tag", 1.524); //5 feet in meters
   
    private SmartDashboardBoolean isRedAlliance = new SmartDashboardBoolean("Camera/is-red-alliance", true);


    public Camera(String camName, Supplier<Rotation2d> gyroAngle, double x, double y, double z, double roll, double pitch, double yaw){
        super("Camera: " + camName);

        this.gyroAngle = gyroAngle;

        cam = new PhotonCamera(camName);
        List<PhotonPipelineResult> unreadResults = cam.getAllUnreadResults();
        if(!unreadResults.isEmpty()){
            result = unreadResults.get(unreadResults.size() -1);
            target = result.getBestTarget();
        }
        else{
            target = null;
            result = null;
        }


        this.robotToCamera = new Transform3d(
            new Translation3d(x, y, z),
            new Rotation3d(roll, pitch, yaw)
        );
    }


    public boolean getIsRedAlliance(){
        return isRedAlliance.getValue();
    }
   
    public Transform3d getValidTagMeasurements(){
        if(target == null || distToTag == null) return null;
        if((target.getFiducialId() == validTags[0] || target.getFiducialId() == validTags[1])){
            if(target.getPoseAmbiguity() < this.maxAmbiguity.getNumber()){
                return target.getBestCameraToTarget();
            }
        }
        return null;
    }


    //0-3, 0 left, 1 left-middle, 2 right-middle, 3 right
    public Double getDistToTarget(int targetCol){
        if(robotPos == null) return null;
        return isRedAlliance.getValue()
            ? PhotonUtils.getDistanceToPose(robotPos, redDockTargets[targetCol])
            : PhotonUtils.getDistanceToPose(robotPos, blueDockTargets[targetCol]);
    }
   
    public Rotation2d getYawToTarget(int target){
        if(robotPos == null) return null;
        return isRedAlliance.getValue()
            ? PhotonUtils.getYawToPose(robotPos, redDockTargets[target])
            : PhotonUtils.getYawToPose(robotPos, blueDockTargets[target]);
    }


    public Pose2d getRobotPose2d(){
        return this.robotPos;
    }
   
    public PhotonCamera getPhotonCamera(){
        return cam;
    }

    public Transform3d getRobotToCamera(){
        return this.robotToCamera;
    }

    @Override
    public void periodic(){
        List<PhotonPipelineResult> unreadResults = cam.getAllUnreadResults();
        if(!unreadResults.isEmpty()){
            result = unreadResults.get(unreadResults.size() -1);
            latestResultTimeStamp = result.getTimestampSeconds();
            target = result.hasTargets()? result.getBestTarget(): null;
            if(target != null){
                SmartDashboard.putNumber("Camera/targetTag/-pitch", target.getPitch());
                SmartDashboard.putNumber("Camera/targetTag/-area", target.getArea());
                SmartDashboard.putNumber("Camera/targetTag/-ambiguity", target.getPoseAmbiguity());
                SmartDashboard.putNumber("Camera/targetTag/-yaw", target.getYaw());
                SmartDashboard.putNumber("Camera/targetTag/-skew", target.getSkew());
                SmartDashboard.putNumber("Camera/targetTag/-fiducial-id", target.getFiducialId());
            }
        }
       
        //checks if target tag measurement is too old
        boolean tooOld = Timer.getFPGATimestamp() - latestResultTimeStamp > oldThreshold.getNumber();
        if(tooOld) target = null;
       
        SmartDashboard.putBoolean("Camera/camera-latest-result", target != null);

        distToTag  = target == null? null: target.getBestCameraToTarget().getTranslation().getNorm();
        
        Transform3d validTagMeasures = this.getValidTagMeasurements();
        if(validTagMeasures != null){
            int idx = target.getFiducialId() == validTags[0]? 0: 1;
            visionPos = PhotonUtils.estimateFieldToRobotAprilTag(
                validTagMeasures, 
                tagPose3ds[idx], 
                robotToCamera.inverse()
            ).toPose2d();
        }
        else{
            visionPos = null;
        }
        
        robotPos = visionPos != null? new Pose2d(visionPos.getTranslation(), gyroAngle.get()): null;
    }
}
