package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.HoldLastLift;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//x = forward.backward, y = strafe

@Autonomous(name = "Right-hopefully 2 specimen", group = "Autonomous")
public class RedRight extends LinearOpMode {
    private RobotHardware robot;
    private SampleMecanumDrive drive;
    private int debugger  = 0;

    public enum DRIVE_STATE{
        inactive,
        start,
        drop,
        push,
        push2,
        push3,
        cycle,
        grab2,
        end
    }

    private DRIVE_STATE current_state;
    private boolean isWaiting = false;

    //Trajectories
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d predropPose = new Pose2d(20,0, Math.toRadians(180));
    private Pose2d dropPose = new Pose2d(29.5, 0, Math.toRadians(180));
//    private Pose2d push0 = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d push1 = new Pose2d(20, -49.5, Math.toRadians(180));
    private Pose2d push2 = new Pose2d(55, -49.5, Math.toRadians(180));
    private Pose2d push3 = new Pose2d(55, -55, Math.toRadians(180));
    private Pose2d push4 = new Pose2d(55, -66, Math.toRadians(180));
    private Pose2d zone1 = new Pose2d(-1, -60, Math.toRadians(180));
    private Pose2d zone2 = new Pose2d(0.4, -60, Math.toRadians(180));
    private Pose2d cycle1 = new Pose2d(10, -60, Math.toRadians(180));
    private Pose2d cycle2 = new Pose2d(32.9, 15, Math.toRadians(180));
    private Pose2d park = new Pose2d(5, -50, Math.toRadians(180));
    private ElapsedTime time = new ElapsedTime();
//
    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robot = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match

        isWaiting = false;

        robot.init();
        robot.intake.pitchTransfer();
        robot.intake.fourBarTransfer();
        robot.intake.pitchSpecimenRest();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory startTraj = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(predropPose.vec(), Math.toRadians(180))
                .build();
        Trajectory dropTraj = drive.trajectoryBuilder(predropPose)
                .splineToConstantHeading(dropPose.vec(), Math.toRadians(180))
                .build();
        Trajectory pushTraj = drive.trajectoryBuilder(dropPose)
                .splineToConstantHeading(predropPose.vec(), Math.toRadians(180))
//                .splineToConstantHeading(push0.vec(), Math.toRadians(180))
                .splineToConstantHeading(push1.vec(), Math.toRadians(180))
                .splineToConstantHeading(push2.vec(), Math.toRadians(180))
                .build();
        Trajectory push2Traj = drive.trajectoryBuilder(push2)
                .splineToConstantHeading(push3.vec(), Math.toRadians(180))
                .splineToSplineHeading(zone1, Math.toRadians(180))
                .splineToConstantHeading(push3.vec(), Math.toRadians(180))
                .build();
        Trajectory push3Traj = drive.trajectoryBuilder(push3)
                .splineToConstantHeading(push4.vec(), Math.toRadians(180))
                .splineToSplineHeading(zone2, Math.toRadians(180))
                .build();
        Trajectory cycleTraj = drive.trajectoryBuilder(zone2)
                .splineToConstantHeading(cycle1.vec(), Math.toRadians(180))
                .splineToConstantHeading(cycle2.vec(), Math.toRadians(180))
                .build();
        Trajectory parkTraj = drive.trajectoryBuilder(cycle2)
                .splineToConstantHeading(park.vec(), Math.toRadians(180))
                .build();

        telemetry.addData("Status", "Trajectories built");
        telemetry.update();

        waitForStart();
        current_state = DRIVE_STATE.start;
        HoldLastLift.setHeight(0);

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            robot.lifts.stateUpdate();
            robot.lifts.backLift();

            switch (current_state)
            {
                case inactive:
                    break;
                case start:
                    robot.claw.specimenAuto();
                    robot.claw.armSpecimenAuto();
                    drive.followTrajectoryAsync(startTraj);
                    current_state = DRIVE_STATE.drop;
                    break;
                case drop:
                    if (!drive.isBusy())
                    {
                        drive.followTrajectoryAsync(dropTraj);
                        current_state = DRIVE_STATE.push;
                    }
                    break;
                case push:
                    if (!drive.isBusy())
                    {
                        if (!isWaiting)
                        {
                            isWaiting = true;
                            robot.lifts.ActulaAutoSpec();
                            robot.lifts.AutoWait();
                            robot.claw.clawOpen();

                            time.reset();
                            isWaiting = true;

                            robot.claw.armDown();
                            robot.claw.specimen();
                            robot.lifts.AutoLow();
                            robot.lifts.AutoWait();
                            drive.followTrajectoryAsync(pushTraj);
                            current_state = DRIVE_STATE.push2;
                            isWaiting = false;
                        }
                    }
                    break;
                case push2:
                    if (!drive.isBusy())
                    {

                        drive.followTrajectoryAsync(push2Traj);
                        current_state = DRIVE_STATE.push3;
                    }
                    break;
                case push3:
                    if (!drive.isBusy())
                    {
                        drive.followTrajectoryAsync(push3Traj);
                        current_state = DRIVE_STATE.cycle;
                    }
                    break;
                case cycle:
                    if (!drive.isBusy())
                    {
                        if (!isWaiting)
                        {
                            time.reset();
                            isWaiting = true;
                        }
                        else if (isWaiting && time.seconds() > 1.2)
                        {
                            robot.lifts.GoToPositionVertical(500);
                            robot.claw.specimenAuto();
                            robot.claw.armSpecimenAuto();
                            isWaiting = false;
                            drive.followTrajectoryAsync(cycleTraj);
                            current_state = DRIVE_STATE.grab2;

                        }
                        else if (isWaiting && time.seconds() > 0.8)
                        {
                            robot.claw.clawClose();
                        }
                        break;
                    }
                case grab2:
                    if (!drive.isBusy())
                    {
                        robot.lifts.ActulaAutoSpec();
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();

                        robot.claw.armDown();
                        robot.claw.specimen();
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();

                        drive.followTrajectoryAsync(parkTraj);
                        current_state = DRIVE_STATE.end;
                        break;
                    }
                case end:
                    if (!drive.isBusy())
                    {
                        stop();
                    }

            }

//            telemetry.addData("Lift State", robot.lifts.getCurrentState());
            telemetry.addData("Drive State", current_state);
            telemetry.addData("debug", debugger);
            telemetry.addData("problematic bool", isWaiting);
            telemetry.addData("time", time.seconds());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));


            telemetry.update();
        }
//
        telemetry.addData("Status", "Autonomous Routine Complete");
        telemetry.update();

    }
}