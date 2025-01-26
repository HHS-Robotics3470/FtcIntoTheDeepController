package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//x = forward.backward, y = strafe

@Autonomous(name = "Right Faster-hopefully 3 specimen", group = "Autonomous")
public class RedRight extends LinearOpMode {
    private RobotHardware robot;
    private SampleMecanumDrive drive;

    public enum DRIVE_STATE{
        start,
        dropping1,
        align1,
        grabbing1,
        dropping2,
        delay1,
        align2,
        grabbing2,
        dropping3,
        end
    }

    private DRIVE_STATE current_state;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
    private Pose2d specimen1 = new Pose2d(22, 0, Math.toRadians(180));
    private Pose2d align11 = new Pose2d(15, -30, Math.toRadians(180));
    private Pose2d align12 = new Pose2d(35, -55, Math.toRadians(180));
    private Pose2d align13 = new Pose2d(75, -55, Math.toRadians(180));
    private Pose2d align14 = new Pose2d(75, -50, Math.toRadians(180));
    private Pose2d align15 = new Pose2d(3, -50, Math.toRadians(180));
    private Pose2d specimen20 = new Pose2d(10, -50, Math.toRadians(180));
    private Pose2d specimen21 = new Pose2d(4, 10, Math.toRadians(180));
    private Pose2d specimen22 = new Pose2d(30, 16, Math.toRadians(180));
    private Pose2d align21 = new Pose2d(25, -50, Math.toRadians(180));
    private Pose2d align22 = new Pose2d(4, -50, Math.toRadians(180));

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
        robot.intake.sweeperAuto();

        //start to first specimen drop positon
        Trajectory traj1 = drive.trajectoryBuilder(startPose, Math.toRadians(180))
                .splineToSplineHeading(specimen1, Math.toRadians(180))
                .build();
        //position before grabbing second specimen
        Trajectory traj2 = drive.trajectoryBuilder(specimen1)
                .splineToConstantHeading(align11.vec(), Math.toRadians(180))
                .splineToConstantHeading(align12.vec(), Math.toRadians(180))
                .splineToConstantHeading(align13.vec(), Math.toRadians(180))
                .build();
        //grap and move two specimen
        Trajectory traj3 = drive.trajectoryBuilder(align13)
                .splineToConstantHeading(align14.vec(), Math.toRadians(180))
                .splineToConstantHeading(align15.vec(), Math.toRadians(180))
                .build();
        //drop second specimen
        Trajectory traj4 = drive.trajectoryBuilder(align15)
                .addTemporalMarker(1, () -> {
                    robot.claw.specimenAuto();
                })
                .splineToConstantHeading(specimen20.vec(), Math.toRadians(180))
                .splineToConstantHeading(specimen21.vec(), Math.toRadians(180))
                .splineToConstantHeading(specimen22.vec(), Math.toRadians(180))
                .build();
        //align for third specimen
        Trajectory traj5 = drive.trajectoryBuilder(specimen22)
                .splineToConstantHeading(align21.vec(), Math.toRadians(180))
                .build();
        //grab for third specimen
        Trajectory traj6 = drive.trajectoryBuilder(align21)
                .splineToConstantHeading(align22.vec(), Math.toRadians(180))
                .build();

        current_state = DRIVE_STATE.start;

        while (opModeIsActive() && !isStopRequested())
        {
            drive.update();
            robot.lifts.stateUpdate();

            switch (current_state)
            {
                case start:
                    robot.lifts.AutoSpec();
                    robot.claw.specimenAuto();
                    robot.lifts.AutoWait();
                    drive.followTrajectoryAsync(traj1);
                    current_state = DRIVE_STATE.dropping1;
                    break;
                case dropping1:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.claw.clawOpen();
                        robot.claw.specimen();
                        robot.lifts.AutoLow();
                        drive.followTrajectoryAsync(traj2);
                        current_state = DRIVE_STATE.align1;
                    }
                    break;
                case align1:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        drive.followTrajectoryAsync(traj3);
                        current_state = DRIVE_STATE.grabbing1;
                    }
                    break;
                case grabbing1:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        time.reset();
                        current_state = DRIVE_STATE.delay1;
                    }
                    break;
                case delay1:
                    if (time.milliseconds() >= 1000)
                    {
                        robot.claw.clawClose();
                        robot.intake.sweeperFinal();

                    }
                    if (time.milliseconds() >= 1800)
                    {

                        robot.intake.sweeperInitial();
                        robot.lifts.AutoSpec();
                        drive.followTrajectoryAsync(traj4);
                        current_state = DRIVE_STATE.dropping2;
                    }
                    break;
                case dropping2:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.claw.clawOpen();
                        drive.followTrajectoryAsync(traj5);
                        current_state = DRIVE_STATE.align2;
                    }
                    break;
                case align2:
                    break;
            }

            telemetry.addData("Lift State", robot.lifts.getCurrentState());
            telemetry.addData("Drive State", current_state);
            telemetry.addData("Time", time.milliseconds());
            telemetry.update();
        }

        telemetry.addData("Status", "Autonomous Routine Complete");
        telemetry.update();

    }
}