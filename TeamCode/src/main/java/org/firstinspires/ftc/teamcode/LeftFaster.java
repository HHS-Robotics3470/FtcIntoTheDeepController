package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.HoldLastLift;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Left Faster-Buckets", group = "Autonomous")
public class LeftFaster extends LinearOpMode {
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    private ElapsedTime waitTimer = new ElapsedTime();

    public enum DRIVE_STATE {
        start,
        specimen,
        grab1,
        end
    }

    private DRIVE_STATE current_state;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d specimenPose = new Pose2d(25, -5, Math.toRadians(180));
    private Pose2d backspecPose = new Pose2d(13, 0, Math.toRadians(180));
    private Pose2d grab1pose = new Pose2d(13, 28, Math.toRadians(180));
    private Pose2d grab2pose = new Pose2d(5, 28, Math.toRadians(0));
    private Pose2d grab3pose = new Pose2d(-20, 35, Math.toRadians(0));


    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        Trajectory dropspecimenTraj = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(specimenPose.vec(), Math.toRadians(180))
                .build();
        Trajectory movetosample1Traj = drive.trajectoryBuilder(specimenPose)
                .splineToConstantHeading(backspecPose.vec(), Math.toRadians(180))
                .splineToConstantHeading(grab1pose.vec(), Math.toRadians(180))
                .splineToSplineHeading(grab2pose, Math.toRadians(180))
                .splineToConstantHeading(grab3pose.vec(), Math.toRadians(0))
                .build();

        current_state = DRIVE_STATE.start;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            robot.lifts.stateUpdate();

            switch (current_state) {
                case start:
                    robot.lifts.AutoSpec();
                    robot.claw.specimenAuto();
                    robot.lifts.AutoWait();
                    drive.followTrajectoryAsync(dropspecimenTraj);
                    current_state = DRIVE_STATE.specimen;
                    break;
                case specimen:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();
                        drive.followTrajectory(movetosample1Traj);
                        current_state = DRIVE_STATE.end;
                    }
                    break;
                case end:
                    telemetry.addData("Status", "Autonomous Complete");
                    telemetry.update();
                    break;
            }
        }
    }
}
