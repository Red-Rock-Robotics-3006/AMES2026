// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;


// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import frc.robot.subsystems.vision.Odometry;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.vision.Camera;
// import redrocklib.logging.SmartDashboardNumber;

// public class AutoAlign extends SubsystemBase{
//     private static AutoAlign instance = null;
//     CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
    
//     //PID and movement
//     private SmartDashboardNumber maxTranslationSpeed = new SmartDashboardNumber("AutoAlign/maxTranslationSpeed", 0);
//     private SmartDashboardNumber maxRotationSpeed = new SmartDashboardNumber("AutoAlign/maxRotationSpeed", 0);
//     private PIDController pidY = new PIDController(0, 0, 0);
//     private PIDController pidX = new PIDController(0, 0, 0);
//     private PIDController pidTheta = new PIDController(0, 0, 0);

//     //Alignment Thresholds
//     private SmartDashboardNumber translationThreshold = new SmartDashboardNumber("AutoAlign/translationThreshold", 0);
//     private SmartDashboardNumber rotationThreshold = new SmartDashboardNumber("AutoAlign/rotationThreshold", 0);
    
//     //Interfacing with CSD, this is a robot centric pose because this uses 2d tracking
//     private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric();

//     //Odometry variables
//     private final Odometry odometry = Odometry.getInstance();
//     private Pose2d relativePose2d = new Pose2d();


//     //The target exists to be used in the methods here; it gets set in the autoAlignToTarget method and gets interfaced by the odometry logic in periodic.
//     private int target = -1;

//     private AutoAlign(){
//         super("AutoAlign");
//         //Theta can loop around like a circle
//         pidTheta.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     //Request builders, use PID and clamp to max translation and rotation speeds.
//     private double yRequest(double distY){
//         return MathUtil.clamp(pidY.calculate(0,distY), -this.maxTranslationSpeed.getNumber(),this.maxTranslationSpeed.getNumber());
//     }
    
//     private double xRequest(double distX){
//         return MathUtil.clamp(pidX.calculate(0,distX), -this.maxTranslationSpeed.getNumber(),this.maxTranslationSpeed.getNumber());
//     }
    
//     private double angleRequest(double distTheta){
//         return MathUtil.clamp(pidTheta.calculate(0,distTheta), -this.maxRotationSpeed.getNumber(),this.maxRotationSpeed.getNumber());
//     }


//     //each tick this method runs while auto aligning. It takes in the dimensions boolean[], which has whether a certain component is aligning(true means aligning), and applies a request to the swerve if the cams have a valid robot pose.
//     private void align(){
//         if(this.relativePose2d == null) return;
        
//         csd.setControl(swerveRequest.
//                         withVelocityX(xRequest(this.relativePose2d.getX())).
//                         withVelocityY(yRequest(this.relativePose2d.getY())).
//                         withRotationalRate(angleRequest(this.relativePose2d.getRotation().getRadians())));

//     }

//     //sets the target for odometry, resets the PID to clear stored values when aligning to something new, activates PID until aligned or interrupted(if interrupted sets velocities to 0) 
//     public Command autoAlignToTarget(int target, boolean angleOnly){
//         return angleOnly ?
//             Commands.sequence(Commands.runOnce(() -> setTarget(target), this),
//                 Commands.runOnce(() -> this.resetPID()),
//                 Commands.run(() -> this.alignAngleOnly(), csd)
//                 .until(() -> this.alignedToTargetAngleOnly())).
//                 finallyDo(() -> csd.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)))
//             :
//             Commands.sequence(Commands.runOnce(() -> setTarget(target), this),
//                 Commands.runOnce(() -> this.resetPID()),
//                 Commands.run(() -> this.align(), csd)
//                 .until(() -> this.alignedToTarget())).
//                 finallyDo(() -> csd.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
//     }

//     public Command stopAutoAlignCommand(){
//         return Commands.runOnce(() -> csd.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)), csd);
//     }

//     public void alignAngleOnly(){
//         if(this.relativePose2d == null) return;

//         csd.setControl(swerveRequest.
//                         withVelocityX(0).
//                         withVelocityY(0).
//                         withRotationalRate(angleRequest(this.relativePose2d.getRotation().getRadians())));
//     }

//     public boolean alignedToTarget() {
//         return (this.relativePose2d.getX() < this.translationThreshold.getNumber()) &&
//                (this.relativePose2d.getY() < this.translationThreshold.getNumber()) &&
//                (this.relativePose2d.getRotation().getRadians() < this.rotationThreshold.getNumber()); 
//     }

//     public boolean alignedToTargetAngleOnly() {
//         return (this.relativePose2d.getRotation().getRadians() < this.rotationThreshold.getNumber()); 
//     }

//     private void setTarget(int target){
//         this.target = target;
//     }
    
//     private void resetPID(){
//         this.pidTheta.reset();
//         this.pidY.reset();
//         this.pidX.reset();
//     }

//     @Override
//     public void periodic(){
//         //Update odometry values
//         Double distFromTarget = odometry.get2dDistToTargetAllCams(this.target);
//         Double angleFromTarget = odometry.get2dYawToTargetAllCams(this.target).getRadians();

//         if(distFromTarget == null || angleFromTarget == null){
//             this.relativePose2d = null;
//         }
//         else{
//             this.relativePose2d = new Pose2d(distFromTarget * Math.cos(angleFromTarget), distFromTarget * Math.sin(angleFromTarget), odometry.get2dYawToTargetAllCams(this.target));
//         }


//         SmartDashboard.putNumber("AutoAlign/Relative X from Target", this.relativePose2d.getX());
//         SmartDashboard.putNumber("AutoAlign/Relative Y from Target", this.relativePose2d.getY());
//         SmartDashboard.putNumber("AutoAlign/Relative \u03B8 from Target", this.relativePose2d.getRotation().getDegrees());

//     }

//     public static AutoAlign getInstance(){
//         if(instance == null) instance = new AutoAlign();
//         return instance;
//     }
// }
