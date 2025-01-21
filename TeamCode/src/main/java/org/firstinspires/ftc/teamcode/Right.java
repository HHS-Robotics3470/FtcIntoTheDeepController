package org.firstinspires.ftc.teamcode;
// Fix Blue Left Configuration

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Right-Specimen", group = "Autonomous")
public class Right extends LinearOpMode {
    private SampleMecanumDrive drive;

    // Define all points beforehand with no angle changes
    private Pose2d startPose = new Pose2d(1, 1, 0);
    private Pose2d midPoint = new Pose2d(5, 5, 0); // Move to with no angle turn
    private Pose2d forwardPoint = new Pose2d(4, 0, 0); // Move straight forward to x = 4
    private Pose2d backPoint = new Pose2d(0, 5, 0); // Move back to x = 0
    private Pose2d strafePoint = new Pose2d(0, -10, 0); // Strafe right (to y = 10)

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

            // Build and follow the trajectory sequence
            drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(midPoint)

                            .addDisplacementMarker(() -> {
                                robot.claw.specimenAuto(); // Perform wrist action
                                robot.lifts.GoToPositionVertical(2700); // Move lift to position
                            })

                            // Move straight forward to x = 4
                            .splineToSplineHeading(forwardPoint, Math.toRadians(0))
                            .waitSeconds(1)
                            // Open claw
                            .addDisplacementMarker(() -> robot.claw.clawOpen())

                            // Move back to x = 0
                            .splineToSplineHeading(backPoint, Math.toRadians(0))

                            // Strafe right to y = 10 while lowering wrist and arm
                            .splineToSplineHeading(strafePoint, Math.toRadians(0))
                            .addDisplacementMarker(() -> {
                                robot.claw.wristDown();
                                robot.claw.armRest();
                                robot.lifts.GoToPositionVertical(0);
                            })

                            .build()
            );

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
