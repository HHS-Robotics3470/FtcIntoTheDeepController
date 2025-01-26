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
    private Pose2d bucket = new Pose2d(2, 36, Math.toRadians(60));

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
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .splineToSplineHeading(bucket, Math.toRadians(60))
                    .build();

            // Follow the trajectory sequence
            drive.followTrajectoryAsync(traj);

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
