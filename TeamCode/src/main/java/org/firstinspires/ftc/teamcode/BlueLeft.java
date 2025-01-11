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
public class BlueLeft extends LinearOpMode {
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
                    .back(45)
                    .build();

            drive.followTrajectory(traj);
            robot.claw.specimenAuto();
            robot.lifts.GoToPositionVertical(1710);



            Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(20, 20, Math.toRadians(180)))
                    .back(16)
                    .build();
            drive.followTrajectory(traj3);
            robot.claw.clawOpen();









            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(22)

                    .build();
            drive.followTrajectory(traj4);

//

            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(140)

                    .build();
            drive.followTrajectory(traj5);

            robot.claw.wristDown();
            robot.claw.armRest();
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
