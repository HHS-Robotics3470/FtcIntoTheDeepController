package org.firstinspires.ftc.teamcode;
//Fix Blue Left Configuration

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Lifts;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "the broken one", group = "Autonomous")  //Name Confusion NEEDS FIXES
public class LeftFaster extends LinearOpMode {
    private RobotHardware robotHardware;
    private RobotHardware Mecnum;
    private SampleMecanumDrive drive;

    private Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));
    private Pose2d bucketPos = new Pose2d(1, 36, Math.toRadians(60));


    @Override
    public void runOpMode() {
        // Initialize hardware and components
        RobotHardware robot = new RobotHardware(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);

        if (opModeIsActive()) {
            // Set the initial pose of the robot
            drive.setPoseEstimate(startPose);

            // Create a trajectory sequence
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .addDisplacementMarker(() -> robot.lifts.AutoHigh()) //Lifts to height 3800 high
                    .addDisplacementMarker(() -> {
                        robot.claw.armUp(); //Arm goes up
                        robot.claw.wristUP(); //Wrist goes up
                    })

                    .turn(Math.toRadians(63))  // Turn the robot
                    .strafeLeft(2)  // Strafe left again
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();
                    })
                    .waitSeconds(0.14)  // Wait for a short period
                    .addDisplacementMarker(() -> {
                        robot.claw.wristDown(); //Wrist goes down
                        robot.claw.armRest(); //Arm goes down
                        robot.lifts.AutoLow();
                    })
                    .turn(Math.toRadians(54.3))  // Turn the robot
                    .addDisplacementMarker(() -> {
                        robot.intake.pitchDown();   //Pitch goes down for intake process
                        robot.intake.startIntake(); //Begin Intake Motors
                    })
                    .forward(26.8)  // Move forward with wait codes
                    .waitSeconds(0.78)
                    .addDisplacementMarker(() -> robot.intake.pitchUp())  //WAIT TO Pitch Up
                    .waitSeconds(0.5)
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.intake.stopIntake();
                    }) //WAIT TO Stop Intake Motors
                    .waitSeconds(0.03)
                    .addDisplacementMarker(() -> robot.claw.grab())  //WAIT TO Grab Block
                    .back(26.8)  // Move back
                    .turn(Math.toRadians(-54))  // Turn the robot

                    //Raise up to 3800 height
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoHigh();
                        robot.claw.armUp(); //Raise ArmUp
                        robot.claw.wristUP(); //Raise wristUP
                    })
                    .back(6.3)  // Move back again
                    .waitSeconds(0.03)
                    .addDisplacementMarker(() -> {
                        robot.lifts.AutoWait();
                        robot.claw.clawOpen();
                    })// WAIT TO Claw OPEN
                    .waitSeconds(0.15)
                    .addDisplacementMarker(() -> {
                        robot.claw.wristDown();         //Wrist down
                        robot.claw.armRest();           //Arm rest
                        robot.lifts.AutoLow();
                        robot.lifts.AutoWait();//Lifts go all the way down
                    })
                    .strafeRight(5)  // Strafe right to finish
                    .build();  // Build the trajectory sequence

            // Follow the trajectory sequence

            drive.followTrajectorySequenceAsync(trajSeq);

            while (opModeIsActive() && !isStopRequested())
            {
                drive.update();
                robot.lifts.stateUpdate();
                telemetry.addData("Lift State", robot.lifts.getCurrentState());
                telemetry.addData("Lift position", robot.lLift.getCurrentPosition());
                telemetry.update();
            }


            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

}