package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Lifts;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Left Faster-Buckets", group = "Autonomous")
public class LeftFaster extends LinearOpMode {
    private RobotHardware robotHardware;
    private SampleMecanumDrive drive;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));
    private Pose2d firstTurnPose = new Pose2d(-16, -38.8, Math.toRadians(207));
    private Pose2d dropOffPose = new Pose2d(-18, -40, Math.toRadians(63));
    private Pose2d intakePose = new Pose2d(-16, -13.2, Math.toRadians(54));
    private Pose2d grabPose = new Pose2d(-16, 0, Math.toRadians(54));
    private Pose2d finalPose = new Pose2d(-11, -3.3, Math.toRadians(270));

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        RobotHardware robot = new RobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        if (opModeIsActive()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> robot.lifts.AutoHigh()) // Lifts to height 3800
                    .addDisplacementMarker(() -> {
                        robot.claw.armUp();
                        robot.claw.wristUP();
                    })
                    .splineToSplineHeading(firstTurnPose, Math.toRadians(207))
                    .splineToSplineHeading(dropOffPose, Math.toRadians(63))
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.14)
                    .addDisplacementMarker(() -> {
                        robot.claw.wristDown();
                        robot.claw.armRest();
                        robot.lifts.AutoLow();
                    })
                    .splineToSplineHeading(intakePose, Math.toRadians(54))
                    .addDisplacementMarker(() -> {
                        robot.intake.pitchDown();
                        robot.intake.startIntake();
                    })
                    .splineToSplineHeading(grabPose, Math.toRadians(54))
                    .waitSeconds(0.78)
                    .addDisplacementMarker(() -> robot.intake.pitchUp())
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.intake.stopIntake();
                    })
                    .waitSeconds(0.03)
                    .addDisplacementMarker(() -> robot.claw.grab())
                    .splineToSplineHeading(finalPose, Math.toRadians(-54))
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoHigh();
                        robot.claw.armUp();
                        robot.claw.wristUP();
                    })
                    .splineToSplineHeading(new Pose2d(-5, 0, Math.toRadians(-54)), Math.toRadians(-54))
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.15)
                    .addDisplacementMarker(() -> {
                        robot.claw.wristDown();
                        robot.claw.armRest();
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();
                    })
                    .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(270)), Math.toRadians(270))
                    .build();

            // Follow the trajectory sequence
            drive.followTrajectorySequenceAsync(trajSeq);

            while (opModeIsActive() && !isStopRequested()) {
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
