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

@Autonomous(name = "Right Faster-hopefully 3 specimen", group = "Autonomous")
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
        end
    }

    private DRIVE_STATE current_state;
    private boolean isWaiting = false;

    //Trajectories

    
    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d predropPose = new Pose2d(20,0, Math.toRadians(180));
    private Pose2d dropPose = new Pose2d(29.4, 0, Math.toRadians(180));
    private Pose2d push1 = new Pose2d(10, -50, Math.toRadians(180));
    private Pose2d push2 = new Pose2d(75, -50, Math.toRadians(180));
    private Pose2d push3 = new Pose2d(75, -55, Math.toRadians(180));
    private Pose2d push4 = new Pose2d(75, -60, Math.toRadians(180));
    private Pose2d zone1 = new Pose2d(0, -55, Math.toRadians(180));
    private Pose2d zone2 = new Pose2d(10, -60, Math.toRadians(180));
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robot = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();

        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory startTraj = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(predropPose, Math.toRadians(180))
                .build();
        Trajectory dropTraj = drive.trajectoryBuilder(predropPose)
                .splineToSplineHeading(dropPose, Math.toRadians(180))
                .build();
        Trajectory pushTraj = drive.trajectoryBuilder(dropPose)
                .splineToConstantHeading(predropPose.vec(), Math.toRadians(180))
                .splineToConstantHeading(push1.vec(), Math.toRadians(180))
                .splineToConstantHeading(push2.vec(), Math.toRadians(180))
                .build();
        Trajectory push2Traj = drive.trajectoryBuilder(push2)
                .splineToConstantHeading(push3.vec(), Math.toRadians(180))
                .splineToConstantHeading(zone1.vec(), Math.toRadians(180))
                .splineToConstantHeading(push3.vec(), Math.toRadians(180))
                .splineToConstantHeading(push4.vec(), Math.toRadians(180))
                .splineToConstantHeading(zone2.vec(), Math.toRadians(180))
                .build();


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
                    robot.claw.armUp();
                    drive.followTrajectoryAsync(startTraj);
                    current_state = DRIVE_STATE.drop;
                    break;
                case drop:
                    if (!drive.isBusy())
                    {
                        drive.followTrajectory(dropTraj);
                        current_state = DRIVE_STATE.push;
                    }
                    break;
                case push:
                    if (!drive.isBusy())
                    {
                        if (!isWaiting)
                        {
                            robot.lifts.TeleOpSpec();
                            robot.lifts.AutoWait();

                            time.reset();
                            isWaiting = true;
                        }
                        else if (isWaiting && time.seconds() >= 1)
                        {
                            isWaiting = false;
                            robot.claw.clawOpen();
                            robot.lifts.AutoLow();
                            robot.lifts.AutoWait();
                            drive.followTrajectory(pushTraj);
                            current_state = DRIVE_STATE.push2;
                        }
                    }
                case push2:
                    if (!drive.isBusy())
                    {
                        drive.followTrajectory(push2Traj);
                        current_state = DRIVE_STATE.inactive;
                    }

            }

//            telemetry.addData("Lift State", robot.lifts.getCurrentState());
            telemetry.addData("Drive State", current_state);
            telemetry.addData("debug", debugger);
            telemetry.update();
        }

        telemetry.addData("Status", "Autonomous Routine Complete");
        telemetry.update();

    }
}