package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Blue Left", group = "Autonomous")
public class BlueRight extends LinearOpMode {
    private RobotHardware robot;
    private SampleMecanumDrive drive;
//
    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robot = new RobotHardware(this);
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();
        robot.init();
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

        if (opModeIsActive()) {
            // Initial lift and wrist/arm raise
            robot.claw.armUp();
            robot.claw.wristUP();
            sleep(100); // Allow time for lift and arm to move

            // Strafe left
            Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(15).build();
            drive.followTrajectory(traj1);

            // Cycle 1: Backward movement, lift, and drop
            executeCycle1();

            // Strafe right for positioning
            Trajectory traj11 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(11).build();
            drive.followTrajectory(traj11);

            // Cycle 2: Pickup and drop sequence
            executeCycle2();

            // Final strafe right
            Trajectory traj7 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(5).build();
            drive.followTrajectory(traj7);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    private void executeCycle1() {
        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(37).build();
        drive.followTrajectory(traj2);

        robot.lifts.GoToPositionVertical(3800);
        robot.claw.armUp();
        robot.claw.wristUP();
        drive.turn(Math.toRadians(66));

        Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(2).build();
        drive.followTrajectory(traj3);

        robot.claw.clawOpen();
        sleep(140);
        resetClawAndLift();
        drive.turn(Math.toRadians(60));
    }

    private void executeCycle2() {
        Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate()).forward(26).build();
        robot.intake.pitchDown();
        robot.intake.startIntake();
        sleep(200);
        drive.followTrajectory(traj4);

        robot.intake.pitchUp();
        sleep(200);
        robot.intake.stopIntake();
        sleep(30);
        robot.claw.grab();

        Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(24).build();
        drive.followTrajectory(traj5);

        drive.turn(Math.toRadians(-63));
        robot.lifts.GoToPositionVertical(3800);
        robot.claw.armUp();
        robot.claw.wristUP();

        Trajectory traj6 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(9).build();
        drive.followTrajectory(traj6);

        sleep(30);
        robot.claw.clawOpen();
        sleep(150);
        resetClawAndLift();
    }

    private void resetClawAndLift() {
        robot.claw.wristDown();
        robot.claw.armRest();
        robot.lifts.GoToPositionVertical(0);
    }
}
