package org.firstinspires.ftc.teamcode;
// Fix Blue Left Configuration

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Right-Specimen", group = "Autonomous")
public class Right extends LinearOpMode {
    private SampleMecanumDrive drive;

    // Define all points beforehand with no angle changes
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d midPoint = new Pose2d(20, 20, Math.toRadians(180)); // Move to with no angle turn
    private Pose2d forwardPoint = new Pose2d(27, 20, Math.toRadians(180)); // Move straight forward to x = 4
    private Pose2d backPoint = new Pose2d(0, 5, Math.toRadians(180)); // Move back to x = 0
    private Pose2d strafePoint = new Pose2d(0, -10, Math.toRadians(180)); // Strafe right (to y = 10)

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
            TrajectorySequence traj =  drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> {
                        robot.claw.specimenAuto(); // Perform wrist action
                        robot.lifts.AutoSpec(); // Move lift to position
                        robot.lifts.AutoWait();
                    })
                    .waitSeconds(1)
                    // Move to point before attaching
                    .back(38)
                    // Wait for lifts
                    .addDisplacementMarker(() -> {
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.1)
                    .forward(30)
                    .addDisplacementMarker(() -> {
                        robot.claw.armRest();
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();
                    })
                    .strafeLeft(80)
                    .forward(40)


//
//                    // Move back to x = 0
//                    .splineToSplineHeading(backPoint, Math.toRadians(0))
//
//                    // Strafe right to y = 10 while lowering wrist and arm
//                    .splineToSplineHeading(strafePoint, Math.toRadians(0))
//                    .addDisplacementMarker(() -> {
//                        robot.claw.wristDown();
//                        robot.claw.armRest();
//                        robot.lifts.GoToPositionVertical(0);
//                    })
                    .build();

            drive.followTrajectorySequence(traj);

            while (opModeIsActive() && !isStopRequested())
            {
                drive.update();
                robot.lifts.stateUpdate();
                telemetry.addData("Lift State", robot.lifts.getCurrentState());
                telemetry.addData("Lift position", robot.lLift.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
