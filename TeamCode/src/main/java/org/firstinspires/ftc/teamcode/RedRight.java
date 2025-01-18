package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "test-dont use", group = "Autonomous")
public class RedRight extends LinearOpMode {
    private RobotHardware robotHardware;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        // Initialize hardware and components
//        RobotHardware robot = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();
//        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(0, 0, 0);

        if (opModeIsActive()) {
            Trajectory traj1 = drive.trajectoryBuilder(start)
                    .splineToSplineHeading(new Pose2d(20, 20, Math.toRadians(90)), 0)
                    .build();

            drive.followTrajectory(traj1);


            telemetry.addData("Status", "Autonomous Routine Complete");
            telemetry.update();
        }
    }

}