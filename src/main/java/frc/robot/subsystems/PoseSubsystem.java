package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Pose extends SubsystemBase{
    private Pigeon2 gyro; 
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
 
private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
          gyro.getRotation2d(),
        drivetrain.getModuleLocations(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));



}

