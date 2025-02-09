package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.HoldLastLift;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


//NOTE: NEEDS TO BE ALL FIXED AND CHANGED SINCE THIS IS FOR LEFT!!! REWRITTEN FROM RIGHT CODE!

@Autonomous(name = "Left Faster-Buckets", group = "Autonomous")
public class LeftFaster extends LinearOpMode {
    private RobotHardware robot;
    private SampleMecanumDrive drive;

    public enum DRIVE_STATE {
        start,
        dropping1,
        align1,
        grabbing1,
        dropping2,
        delay1,
        align2,
        grabbing2,
        delay2,
        dropping3,
        improv1,
        improv2,
        improve3,
        improvDelay,
        end
    }

    private DRIVE_STATE current_state;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d specimen1 = new Pose2d(-22, 0, Math.toRadians(180));

    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose, Math.toRadians(180))
                .splineToSplineHeading(specimen1, Math.toRadians(180))
                .build();

        current_state = DRIVE_STATE.start;
        HoldLastLift.setHeight(0);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            robot.lifts.stateUpdate();

            switch (current_state) {
                case start:
                    robot.lifts.AutoSpec();
                    robot.claw.specimenAuto();
                    robot.lifts.AutoWait();
                    drive.followTrajectoryAsync(traj1);
                    current_state = DRIVE_STATE.dropping3;
                    break;
                case end:
                    telemetry.addData("Status", "Autonomous Complete");
                    telemetry.update();
                    break;
            }
        }
    }