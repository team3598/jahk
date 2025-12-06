package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.stuff.AprilTagRegion;

public class ReefAlignment {

// create variables for reef tags
    public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
    public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();

static{
        var field = RobotContainer.getFieldLayout(); }

    private final CommandSwerveDrivetrain drivetrain;

public ReefAlignment(CommandSwerveDrivetrain drivetrain, AprilTagFieldLayout field) {
    // drivetrain call
    this.drivetrain = drivetrain;



// add reef tags - pose / based on alliances
        Arrays.stream(AprilTagRegion.kReef.blue()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                blueReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.red()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                redReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });

        Arrays.stream(AprilTagRegion.kReef.both()).forEach((i) -> {
            field.getTagPose(i).ifPresent((p) -> {
                allReefTagPoses.add(new Pose2d(
                    p.getMeasureX(),
                    p.getMeasureY(),
                    p.getRotation().toRotation2d()
                ));
            });
        });
    }



}

