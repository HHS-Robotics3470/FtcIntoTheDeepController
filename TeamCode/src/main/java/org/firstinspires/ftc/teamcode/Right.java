package org.firstinspires.ftc.teamcode;
//Fix Blue Left Configuration

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Right-Specimen", group = "Autonomous")
public class Right extends LinearOpMode {
    private SampleMecanumDrive drive;

    // Define all points beforehand
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d midPoint = new Pose2d(20, 20, Math.toRadians(180));
    private Pose2d backPoint = new Pose2d(20, -25, Math.toRadians(180));
    private Pose2d dropPoint = new Pose2d(20, -39.5, Math.toRadians(180));
    private Pose2d collectPoint = new Pose2d(20, -61.5, Math.toRadians(180));
    private Pose2d strafePoint = new Pose2d(20, -61.5, Math.toRadians(270));

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();

        if (opModeIsActive()) {
            // Set the initial pose of the robot
            drive.setPoseEstimate(startPose);

            // Follow trajectories using splineToSplineHeading
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(startPose)
                            .splineToSplineHeading(midPoint, Math.toRadians(180)) // Move to midPoint
                            .addDisplacementMarker(() -> robot.claw.specimenAuto()) // Claw action
                            .addDisplacementMarker(() -> robot.lifts.GoToPositionVertical(1750)) // Lifts to position
                            .splineToSplineHeading(backPoint, Math.toRadians(180)) // Move back
                            .addDisplacementMarker(() -> robot.claw.clawOpen()) // Open claw
                            .splineToSplineHeading(dropPoint, Math.toRadians(180)) // Move to dropPoint
                            .splineToSplineHeading(collectPoint, Math.toRadians(180)) // Move to collectPoint
                            .addDisplacementMarker(() -> {
                                robot.claw.wristDown();
                                robot.claw.armRest();
                                robot.lifts.GoToPositionVertical(0);
                            })
                            .splineToSplineHeading(strafePoint, Math.toRadians(270)) // Strafe left
                            .build()
            );

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
