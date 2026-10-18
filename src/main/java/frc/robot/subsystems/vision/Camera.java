package frc.robot.subsystems.vision;

import java.util.List;

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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;

public class Camera extends SubsystemBase{
    private PhotonCamera cam;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Pose2d robotPos;
    private int[] validTags = {4, 8};

    private Pose3d[] tagPose3ds = { //TODO use field cad
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)),
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0))
    };

    private Transform3d cameraToRobot = new Transform3d( //TODO where is camera on robot
        new Translation3d(0, 0, 0), 
        new Rotation3d(0, 0, 0)
    );

    private Pose2d[] redDockTargets = { //TODO use field cad
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    };
    private Pose2d[] blueDockTargets = {
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    };

    private Double distToTag = null;
    private double cameraHeight; //meters
    private double cameraPitch; //radians

    private SmartDashboardNumber maxAmbiguity = new SmartDashboardNumber("Camera/max-ambiguity", 0);
    private SmartDashboardNumber maxDistFromTag = new SmartDashboardNumber("Camera/max-distance-from-tag", 1.524); //5 feet in meters
    
    private SmartDashboardBoolean isRedAlliance = new SmartDashboardBoolean("Camera/is-red-alliance", true);

    public Camera(String camName, double x, double y, double z, double roll, double pitch, double yaw){
        super("Camera");

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

        this.cameraToRobot = new Transform3d(
            new Translation3d(x, y, z), 
            new Rotation3d(roll, pitch, yaw)
        );
    }

    public boolean getIsRedAlliance(){
        return isRedAlliance.getValue();
    }
    
    private Transform3d getValidTagMeasurements(){
        if(result.hasTargets() && (target.getFiducialId() == validTags[0] || target.getFiducialId() == validTags[1])){
            if(target.getPoseAmbiguity() < this.maxAmbiguity.getNumber() && distToTag <= maxDistFromTag.getNumber()){
                return target.getBestCameraToTarget();
            }
        }
        return null;
    }

    //0-3, 0 left, 1 left-middle, 2 right-middle, 3 right
    public Double getDistToTarget(int target){
        if(robotPos == null) return null;
        return isRedAlliance.getValue()? PhotonUtils.getDistanceToPose(robotPos, redDockTargets[target]): PhotonUtils.getDistanceToPose(robotPos, blueDockTargets[target]);
    }
    
    public Rotation2d getYawToTarget(int target){
        if(robotPos == null) return null;
        return isRedAlliance.getValue()? PhotonUtils.getYawToPose(robotPos, redDockTargets[target]): PhotonUtils.getYawToPose(robotPos, blueDockTargets[target]);
    }
    
    @Override
    public void periodic(){
        List<PhotonPipelineResult> unreadResults = cam.getAllUnreadResults();
        if(!unreadResults.isEmpty()){
            result = unreadResults.get(unreadResults.size() -1);
            target = result.getBestTarget();
            SmartDashboard.putNumber("Camera/targetTag/-pitch", target.getPitch());
            SmartDashboard.putNumber("Camera/targetTag/-area", target.getArea());
            SmartDashboard.putNumber("Camera/targetTag/-ambiguity", target.getPoseAmbiguity());
            SmartDashboard.putNumber("Camera/targetTag/-yaw", target.getYaw());
            SmartDashboard.putNumber("Camera/targetTag/-skew", target.getSkew());
            SmartDashboard.putNumber("Camera/targetTag/-fiducial-id", target.getFiducialId());
        }
        else{
            target = null;
            result = null;
        }
        SmartDashboard.putBoolean("Camera/camera-latest-result", result.hasTargets());

        if(this.getValidTagMeasurements() != null){
            if(target.getFiducialId() == validTags[0])
                distToTag = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraToRobot.getTranslation().getZ(), 
                    tagPose3ds[0].getTranslation().getZ(), 
                    cameraToRobot.getRotation().getY(), 
                    tagPose3ds[0].getRotation().getY()
                );
            else if(target.getFiducialId() == validTags[1]){
                distToTag = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraToRobot.getTranslation().getZ(), 
                    tagPose3ds[1].getTranslation().getZ(), 
                    cameraToRobot.getRotation().getY(), 
                    tagPose3ds[1].getRotation().getY()
                );   
            }
        }
        else{
            distToTag = null;
        }

        if(this.getValidTagMeasurements() != null){
            if(target.getFiducialId() == validTags[0]){
                robotPos = PhotonUtils.estimateFieldToRobotAprilTag(
                    this.getValidTagMeasurements(), 
                    tagPose3ds[0], 
                    cameraToRobot
                ).toPose2d();
            }
            else if(target.getFiducialId() == validTags[1]){
                robotPos = PhotonUtils.estimateFieldToRobotAprilTag(
                    this.getValidTagMeasurements(), 
                    tagPose3ds[1], 
                    cameraToRobot
                ).toPose2d();   
            }
        }
        else{
            robotPos = null;
        }
    }
}
