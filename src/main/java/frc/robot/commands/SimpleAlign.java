package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class SimpleAlign extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivetrain;
  private double tagID = -1;
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 1;  
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0;
  public static final double X_SETPOINT_REEF_ALIGNMENT = 0.02; 
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.01;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = -1.07; 
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 10;
  private final SwerveRequest.RobotCentric drive_request = new SwerveRequest.RobotCentric();
  


  public SimpleAlign(boolean isRightScore, CommandSwerveDrivetrain drivetrain) {
    xController = new PIDController(0.25, 0, 0);  // Vertical movement
    yController = new PIDController(0.25, 0, 0);  // Horizontal movement
    rotController = new PIDController(0.025, 0.01, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

@Override
public void initialize() {
  this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-fright");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-fright") && LimelightHelpers.getFiducialID("limelight-fright") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-fright");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);
      
      
      drivetrain.setControl(drive_request.withVelocityX(xSpeed).withVelocityY(-ySpeed).withRotationalRate(rotValue));

      System.out.println("x velocity output: " + xSpeed);
      System.out.println("y velocity output: " + ySpeed);
      System.out.println("rot velocity output: " + rotValue);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
        drivetrain.setControl(drive_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive_request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
}

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(POSE_VALIDATION_TIME);
  }
}




