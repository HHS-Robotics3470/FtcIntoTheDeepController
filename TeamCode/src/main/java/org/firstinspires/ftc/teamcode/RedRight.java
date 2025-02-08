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

    public enum DRIVE_STATE{
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
    private Pose2d specimen1 = new Pose2d(22, 0, Math.toRadians(180));

    private Pose2d align11 = new Pose2d(15, -30, Math.toRadians(180));
    private Pose2d align12 = new Pose2d(35, -55, Math.toRadians(180));
    private Pose2d align13 = new Pose2d(75, -55, Math.toRadians(180));
    private Pose2d align14 = new Pose2d(75, -44, Math.toRadians(180));
    private Pose2d alignx = new Pose2d(74, -48, Math.toRadians(180));
    private Pose2d align15 = new Pose2d(5, -48, Math.toRadians(180));
    private Pose2d specimen20 = new Pose2d(8, -48, Math.toRadians(180));
    private Pose2d specimen21 = new Pose2d(5, 10, Math.toRadians(180));
    private Pose2d specimen22 = new Pose2d(35, 27, Math.toRadians(180));
    private Pose2d align21 = new Pose2d(11, -50, Math.toRadians(180));
    private Pose2d align22 = new Pose2d(4.9, -50, Math.toRadians(180));
    private Pose2d specimen31 = new Pose2d(5, 25, Math.toRadians(180));
    private Pose2d specimen32 = new Pose2d(35, 30, Math.toRadians(180));
    private Pose2d endPose = new Pose2d (17, 25, Math.toRadians(180));


    private Pose2d  aligny1 = new Pose2d(11, -55, Math.toRadians(180));
    private Pose2d  aligny2 = new Pose2d(10, -55, Math.toRadians(0));
    private Pose2d  grapy1 = new Pose2d(10, 10, Math.toRadians(180));
    private Pose2d  graby2 = new Pose2d(26, 10, Math.toRadians(180));
    private Pose2d  dropy = new Pose2d(10, 10, Math.toRadians(180));

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

        //start to first specimen drop positon
        Trajectory traj1 = drive.trajectoryBuilder(startPose, Math.toRadians(180))
                .splineToSplineHeading(specimen1, Math.toRadians(180))
                .build();
        //improvise
        Trajectory trajy2 = drive.trajectoryBuilder(specimen1)
                .splineToSplineHeading(aligny1, Math.toRadians(180))
                .splineToSplineHeading(aligny2, Math.toRadians(0))
                .build();
        Trajectory trajy3 = drive.trajectoryBuilder(aligny2)
                .splineToSplineHeading(grapy1, Math.toRadians(180))
                .splineToSplineHeading(graby2, Math.toRadians(180))
                .build();
        Trajectory trajy4 = drive.trajectoryBuilder(graby2)
                .splineToSplineHeading(dropy, Math.toRadians(180))
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
                .splineToConstantHeading(alignx.vec(), Math.toRadians(180))
                .splineToConstantHeading(align15.vec(), Math.toRadians(180))
                .build();
        //drop second specimen
        Trajectory traj4 = drive.trajectoryBuilder(align15)
                .splineToConstantHeading(specimen20.vec(), Math.toRadians(180))
                .splineToConstantHeading(specimen21.vec(), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.claw.specimenAuto();
                })
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
        //drop 3rd specimen
        Trajectory traj7 = drive.trajectoryBuilder(align22)
                .splineToConstantHeading(specimen31.vec(), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.claw.specimenAuto();
                })
                .splineToConstantHeading(specimen32.vec(), Math.toRadians(180))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(specimen32)
                .splineToConstantHeading(endPose.vec(), Math.toRadians(180))
                .build();

        current_state = DRIVE_STATE.start;
        HoldLastLift.setHeight(0);

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
                    current_state = DRIVE_STATE.dropping3;
                    break;
                case improv1:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.claw.clawOpen();
                        robot.claw.specimen();
                        robot.lifts.AutoLow();
                        drive.followTrajectoryAsync(trajy2);
                        current_state = DRIVE_STATE.improvDelay;
                    }
                    break;
                case improvDelay:
                    if (time.milliseconds() >= 1000)
                    {
                        robot.claw.clawClose();
                    }
                    if (time.milliseconds() >= 1800)
                    {

                        robot.lifts.AutoSpec();
                        drive.followTrajectoryAsync(trajy3);
                        current_state = DRIVE_STATE.improv2;
                    }
                    break;
                case improv2:
                      if (!drive.isBusy() && robot.lifts.IsInactive())
                      {
                          robot.claw.clawOpen();
                          drive.followTrajectoryAsync(trajy4);
                          current_state = DRIVE_STATE.end;
                      }
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
                    }
                    if (time.milliseconds() >= 1800)
                    {

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
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.claw.specimen();
                        robot.lifts.AutoLow();
                        drive.followTrajectoryAsync(traj6);
                        current_state = DRIVE_STATE.grabbing2;
                    }
                    break;
                case grabbing2:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        time.reset();
                        current_state = DRIVE_STATE.delay2;
                    }
                    break;
                case delay2:
                    if (time.milliseconds() >= 1000)
                    {
                        robot.claw.clawClose();
                    }
                    if (time.milliseconds() >= 1800)
                    {

                        robot.lifts.AutoSpec();
                        drive.followTrajectoryAsync(traj7);
                        current_state = DRIVE_STATE.dropping3;
                    }
                    break;
                case dropping3:
                    if (!drive.isBusy() && robot.lifts.IsInactive())
                    {
                        robot.claw.clawOpen();
                        drive.followTrajectoryAsync(traj8);
                        current_state = DRIVE_STATE.end;
                    }
                    break;
                case end:
                    if (!drive.isBusy()) {
                        robot.claw.specimen();
                        robot.lifts.GoToPositionVertical(0);
                    }
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