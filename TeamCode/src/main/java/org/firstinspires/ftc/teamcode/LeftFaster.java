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

@Autonomous(name = "Left -Buckets", group = "Autonomous")
public class LeftFaster extends LinearOpMode {
    private RobotHardware robotHardware;
    private RobotHardware Mecnum;
    private SampleMecanumDrive drive;

//
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

            Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate()).back(40.7).build();


            drive.followTrajectory(traj2);



            robot.lifts.GoToPositionVertical(3800);



            robot.claw.armUp();
            robot.claw.wristUP();
            drive.turn(Math.toRadians(63));


            Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate()).strafeRight(1.3).build();
            drive.followTrajectory(traj3);

            robot.claw.clawOpen();

            sleep(340);

            robot.claw.wristDown();
            robot.claw.armRest();
            robot.lifts.GoToPositionVertical(0);
            drive.turn(Math.toRadians(54.3));
            robot.intake.pitchDown();
//            robot.intake.startIntake();



            //2nd Cycle

            Trajectory traj4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(12)
                    .build();


            drive.turn(Math.toRadians(-10));
//            robot.intake.startIntake();
            sleep(780);
            drive.followTrajectory(traj4);


            Trajectory traj90 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft(3.5)
                    .build();

//            robot.intake.pitchIntaking();
//            robot.intake.fourBarIntaking();
//            sleep(120);
//            robot.intake.clawIntakeClose();
//            sleep(30);
//            robot.intake.fourBarTransfer();
//            robot.intake.pitchTransfer();
//            sleep(30);

            drive.followTrajectory(traj90);
            robot.claw.armRest();

            robot.claw.clawOpen();
            robot.intake.clawIntakeOpen();
            robot.intake.pitchIntaking();
            robot.intake.fourBarIntaking();
            sleep(200);
            robot.intake.clawIntakeClose();

            sleep(400);

        //new code
                robot.intake.pitchTransfer();
                robot.intake.fourBarTransfer();
                robot.intake.clawIntakeClose();
                robot.claw.clawOpen();
            drive.turn(Math.toRadians(10));
                robot.claw.wristDown();
                sleep(450);



                robot.intake.pitchTransfer();
                robot.intake.clawIntakeClose();
                robot.claw.armDown();
                sleep(300);


                robot.intake.clawIntakeOpen();
                sleep(100);
                robot.intake.pitchTransfer();
                robot.claw.clawClose();
                sleep(400);

                robot.claw.armRest();
                sleep(300);


                robot.claw.wristUP();
                robot.claw.armUp();

            Trajectory traj91 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(5.5)
                    .build();
            drive.followTrajectory(traj91);




            Trajectory traj5 = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(10)
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






















//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Components.HoldLastLift;
//import org.firstinspires.ftc.teamcode.Components.RobotHardware;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//
//@Autonomous(name = "Left Faster-Buckets", group = "Autonomous")
//public class LeftFaster extends LinearOpMode {
//    private RobotHardware robot;
//    private SampleMecanumDrive drive;
//    private ElapsedTime waitTimer = new ElapsedTime();
//
//    public enum DRIVE_STATE {
//        start,
//        specimen,
//        grab1,
//        end
//    }
//
//    private DRIVE_STATE current_state;
//
//    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
//    private Pose2d specimenPose = new Pose2d(25, -5, Math.toRadians(180));
//    private Pose2d backspecPose = new Pose2d(13, 0, Math.toRadians(180));
//    private Pose2d grab1pose = new Pose2d(13, 28, Math.toRadians(180));
//    private Pose2d grab2pose = new Pose2d(5, 28, Math.toRadians(0));
//    private Pose2d grab3pose = new Pose2d(-20, 35, Math.toRadians(0));
//
////
//    @Override
//    public void runOpMode() {
//        robot = new RobotHardware(this);
//        robot.init();
//        drive = new SampleMecanumDrive(hardwareMap);
//        drive.setPoseEstimate(startPose);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//
//        Trajectory dropspecimenTraj = drive.trajectoryBuilder(startPose)
//                .splineToConstantHeading(specimenPose.vec(), Math.toRadians(180))
//                .build();
//        Trajectory movetosample1Traj = drive.trajectoryBuilder(specimenPose)
//                .splineToConstantHeading(backspecPose.vec(), Math.toRadians(180))
//                .splineToConstantHeading(grab1pose.vec(), Math.toRadians(180))
//                .splineToSplineHeading(grab2pose, Math.toRadians(180))
//                .splineToConstantHeading(grab3pose.vec(), Math.toRadians(0))
//                .build();
//
//        current_state = DRIVE_STATE.start;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//            robot.lifts.stateUpdate();
//
//            switch (current_state) {
//                case start:
//                    robot.lifts.AutoSpec();
//                    robot.claw.specimenAuto();
//                    robot.lifts.AutoWait();
//                    drive.followTrajectoryAsync(dropspecimenTraj);
//                    current_state = DRIVE_STATE.specimen;
//                    break;
//                case specimen:
//                    if (!drive.isBusy() && robot.lifts.IsInactive())
//                    {
//                        robot.lifts.AutoLow();
//                        robot.lifts.AutoWait();
//                        robot.claw.clawOpen();
//                        drive.followTrajectory(movetosample1Traj);
//                        current_state = DRIVE_STATE.end;
//                    }
//                    break;
//                case end:
//                    telemetry.addData("Status", "Autonomous Complete");
//                    telemetry.update();
//                    break;
//            }
//        }
//    }
//}
