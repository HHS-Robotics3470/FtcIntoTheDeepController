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

            robot.intake.pitchDown();

            // Build and follow the trajectory sequence
            TrajectorySequence traj =  drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> {
                        robot.claw.specimenAuto(); // Perform wrist action
                        robot.lifts.AutoSpec(); // Move lift to position
                        robot.lifts.AutoWait();
                    })
                    // Move to point before attaching
                    .back(38)
                    // Wait for lifts
                    .addDisplacementMarker(() -> {
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.1)
                    .forward(20)
                    .turn(0.4)

                    //Moving to get next specimen
                    .addDisplacementMarker(() -> {
                        robot.claw.armRest();
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();
                    })
                    .strafeLeft(25)
                    .addDisplacementMarker(() -> {
                        robot.wrist.setPosition(0.345);
                        robot.claw.specimen();
                    })
                    .forward(10)
                    .waitSeconds(0.8)
                    .forward(25)
                    .waitSeconds(0.5)
                    .addTemporalMarker(6.5, () -> {
                        robot.claw.clawClose();
                    })
                    .waitSeconds(0.3)

                    //Raise claw for next speciment
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoSpec();
                        robot.lifts.AutoWait();
                        robot.claw.specimenAuto();
                    })
                    .back(20)
                    .turn(-0.6)
                    .strafeRight(45)
                    .turn(0.001)
                    .back(9)
                    .addTemporalMarker( 12.4, () -> {
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.2)
                    .forward(15)



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

            while (opModeIsActive() && !isStopRequested() && drive.isBusy())
            {

                drive.update();
                robot.lifts.stateUpdate();
                telemetry.addData("Lift State", robot.lifts.getCurrentState());
                telemetry.addData("Lift position", robot.lLift.getCurrentPosition());
                telemetry.update();
            }

            if (!drive.isBusy())
            {
                robot.claw.specimen();
                robot.lifts.GoToPositionVertical(0);
            }

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
