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

@Autonomous(name = "Blue Left", group = "Autonomous")  //Name Confusion NEEDS FIXES
public class BlueRight extends LinearOpMode {
    private RobotHardware robotHardware;
    private RobotHardware Mecnum;
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

        // Initialize with 'this' LinearOpMode

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the start of the match
        waitForStart();
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);


        if (opModeIsActive()) {
//            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

            //strafe Left

            Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(16).build(); //Should be strafeLeft

            drive.followTrajectory(traj1);

            //Cycle 1

            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(38.8).build();


            drive.followTrajectory(traj2);



            robot.lifts.GoToPositionVertical(3800);



            robot.claw.armUp();
            robot.claw.wristUP();
            drive.turn(Math.toRadians(63));


            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeLeft(2).build();
            drive.followTrajectory(traj3);

            robot.claw.clawOpen();

            sleep(140);

            robot.claw.wristDown();
            robot.claw.armRest();
            robot.lifts.GoToPositionVertical(0);
            drive.turn(Math.toRadians(54.7));
            robot.intake.pitchDown();
            robot.intake.startIntake();



            //2nd Cycle

            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(26)
                    .build();



            robot.intake.startIntake();
            sleep(200);
            drive.followTrajectory(traj4);

            robot.intake.pitchUp();
            sleep(200);
            robot.intake.stopIntake();
            sleep(30);

            robot.claw.grab();




            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(26)
                    .build();
            drive.followTrajectory(traj5);

            drive.turn(Math.toRadians(-54));

            robot.lifts.GoToPositionVertical(3800);

            robot.claw.armUp();
            robot.claw.wristUP();

            Trajectory traj6 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(6.3)
                    .build();
            drive.followTrajectory(traj6);

            sleep(30);
            robot.claw.clawOpen();
//

            sleep(150);

            robot.claw.wristDown();
            robot.claw.armRest();
            robot.lifts.GoToPositionVertical(0);

            //strafe right
            Trajectory traj7 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(5).build();
            drive.followTrajectory(traj7);


















//            Trajectory traj7 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(35).build();
//            drive.followTrajectory(traj7);
//
//            robot.claw.lvl1hang();
//            robot.claw.ThreadSleep(5000);




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