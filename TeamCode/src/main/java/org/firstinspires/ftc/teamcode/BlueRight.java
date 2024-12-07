package org.firstinspires.ftc.teamcode;
//Fix Blue Left Configuration

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Lifts;
import org.firstinspires.ftc.teamcode.Components.Claw;

@Autonomous(name = "Blue Right", group = "Autonomous")
public class BlueRight extends LinearOpMode {
    private RobotHardware robotHardware;
    private RobotHardware Mecnum;
    private Lifts lifts;
    private Claw claw;
    private SampleMecanumDrive drive;


    // Speed control constants
//    public final double DRIVE_SPEED_MAX_AUTO = 0.5;
//    public final double DRIVE_SPEED_SLOW_AUTO = 0.05;
//    public double driveSpeedControl = DRIVE_SPEED_MAX_AUTO;
//
//    // Individual motor speed scaling
//    public double AUTOspeedFLeft = 1.0;
//    public double AUTOspeedFRight = 1.0;
//    public double AUTOspeedBLeft = 2.5; // Back should have more power
//    public double AUTOspeedBRight = 2.5; // Back should have more power


    @Override
    public void runOpMode() {
        // Initialize hardware and components
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap); // Initialize with 'this' LinearOpMode

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the start of the match
        waitForStart();




        if (opModeIsActive()) {
//            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



            drive.setPoseEstimate(new Pose2d(0,0, Math.toRadians(180)));
            Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(20, 20), Math.toRadians(180))
                    .build();
            Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(20, 20, Math.toRadians(180)))
                    .back(17.397)
                    .build();
            drive.followTrajectory(traj);
            robot.claw.specimenAuto();
            robot.lifts.GoToPositionVertical(1600);
            drive.followTrajectory(traj2);

            // Define the starting pose
            drive.setPoseEstimate(new Pose2d(20, 20, Math.toRadians(180)));

// Define a single "backward" trajectory


// Define the lift height parameters
            int targetHeight = 900; // Target height
            int decrementStep = 190; // Amount to decrease in each step
            int currentHeight = 1600; // Starting height

// Loop to simultaneously lower the lift and move backward
            while (currentHeight > targetHeight) {
                // Command the lift to move to the next height step
                int nextHeight = Math.max(currentHeight - decrementStep, targetHeight); // Ensure it doesn't go below the target
                robot.lifts.GoToPositionVertical(nextHeight);
                boolean x = true;

                // Wait until the lift reaches the next position
                while (x == true) {
                    // Follow the backward trajectory
                    Trajectory traj69 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .back(9) // Move 9 unit backward
                            .build();
                    drive.followTrajectory(traj69);


                    x = false;
                }



                // Update the current height
                currentHeight = nextHeight;
            }

// Finalize at the target height
            robot.lifts.GoToPositionVertical(targetHeight);
            robot.claw.clawOpen();
            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(17.35)

                    .build();
            drive.followTrajectory(traj3);
            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(87)
                    .build();
            drive.followTrajectory(traj4);

            robot.lifts.GoToPositionVertical(0);


            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    /**
     * Set power for each motor with independent speed adjustments.
     */
//    public void setDrivePower(double autoPower) {
//        robotHardware.fLeft.setPower(autoPower * driveSpeedControl * AUTOspeedFLeft);
//        robotHardware.fRight.setPower(autoPower * driveSpeedControl * AUTOspeedFRight);
//        robotHardware.bLeft.setPower(autoPower * driveSpeedControl * AUTOspeedBLeft);
//        robotHardware.bRight.setPower(autoPower * driveSpeedControl * AUTOspeedBRight);
//    }

}
