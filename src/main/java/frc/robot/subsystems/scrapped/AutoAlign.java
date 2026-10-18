// package frc.robot.subsystems.scrapped;

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
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.vision.Camera;
// import redrocklib.logging.SmartDashboardNumber;

// public class AutoAlign extends SubsystemBase{
//     CommandSwerveDrivetrain csd = CommandSwerveDrivetrain.getInstance();
    
    
    
//     //PID and movement
//     private SmartDashboardNumber maxTranslationSpeed = new SmartDashboardNumber("AutoAlign/maxTranslationSpeed", 0);
//     private SmartDashboardNumber maxRotationSpeed = new SmartDashboardNumber("AutoAlign/maxRotationSpeed", 0);
//     PIDController pidY = new PIDController(0, 0, 0);
//     PIDController pidX = new PIDController(0, 0, 0);
//     PIDController pidTheta = new PIDController(0, 0, 0);

//     //Alignment Thresholds
//     private SmartDashboardNumber translationThreshold = new SmartDashboardNumber("AutoAlign/translationThreshold", 0);
//     private SmartDashboardNumber rotationThreshold = new SmartDashboardNumber("AutoAlign/rotationThreshold", 0);
    
//     //Interfacing with CSD, this is a robot centric pose because this uses 2d tracking
//     private final SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric();

//     //Odometry variables
//     Odometry odometry = Odometry.getInstance();
//     Pose2d relativePose2d = new Pose2d();

//     //The target exists to be used in the methods here; it gets set in the autoAlignToTarget method and gets interfaced by the odometry logic in periodic.
//     int target = -1;

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
//     private void align(boolean[] dimensions){
//         if(this.relativePose2d == null) return;
        
//         csd.setControl(swerveRequest.
//                         withVelocityX(dimensions[0]?xRequest(this.relativePose2d.getX()):-RobotContainer.driveStick.getLeftY()).
//                         withVelocityY(dimensions[1]?yRequest(this.relativePose2d.getY()):-RobotContainer.driveStick.getLeftX()).
//                         withRotationalRate(dimensions[2]?angleRequest(this.relativePose2d.getRotation().getRadians()):-RobotContainer.driveStick.getRightX()));

//     }

//     //sets the target for odometry, resets the PID to clear stored values when aligning to something new, activates PID until aligned or interrupted(if interrupted sets velocities to 0) 
//     public Command autoAlignToTarget(int target, boolean[] dimensions){
//         return Commands.sequence(Commands.runOnce(() -> setTarget(target), this),
//                                  Commands.runOnce(() -> this.resetPID()),
//                                  Commands.run(() -> this.align(dimensions), csd)
//                                     .until(() -> this.alignedToTarget(dimensions))).
//                                 finallyDo(() -> csd.setControl(swerveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)))
//                                 ;
//     }

//     //Checks if all dimensions to be aligned in are aligned w/ threshold
//     private boolean alignedToTarget(boolean[] dimensions) {
//         return (!dimensions[0] || this.relativePose2d.getX() < this.translationThreshold.getNumber()) &&
//                (!dimensions[1] || this.relativePose2d.getY() < this.translationThreshold.getNumber()) &&
//                (!dimensions[2] || this.relativePose2d.getRotation().getRadians() < this.rotationThreshold.getNumber()); 
//     }

//     private void setTarget(int target){
//         this.target = target;
//     }
    
//     private void resetPID(){
//         this.pidTheta.reset();
//         this.pidY.reset();
//         this.pidX.reset();
//     }

//     //Converts field centric driver input to robot centric input.
//     private Translation2d driverToRobotFrame(double driverVx, double driverVy) {
//         //gets gyro-based rotation
//         Rotation2d robotHeading = csd.getState().Pose.getRotation();
//         //gets forward direction based on alliance(+x direction relative to driver)
//         Rotation2d operatorForward = csd.getOperatorForwardDirection();

//         //finds the driver's inputted speed vector and converts it into a field-relative vector, then converts it into a robot-relative vector.
//         Translation2d driverVector = new Translation2d(driverVx, driverVy);
//         Translation2d fieldVector = driverVector.rotateBy(operatorForward);

//         Translation2d robotVector = fieldVector.rotateBy(robotHeading.unaryMinus());

//         return robotVector;
//     }

//     @Override
//     public void periodic(){
//         //Update odometry values
//         Double distFromTarget = odometry.getDistToTarget(this.target);
//         Double angleFromTarget = odometry.getYawToTarget(this.target).getRadians();

//         if(distFromTarget == null || angleFromTarget == null){
//             this.relativePose2d = null;
//         }
//         else{
//             this.relativePose2d = new Pose2d(distFromTarget * Math.cos(angleFromTarget), distFromTarget * Math.sin(angleFromTarget), odometry.getYawToTarget(this.target));
//         }
//     }
// }
