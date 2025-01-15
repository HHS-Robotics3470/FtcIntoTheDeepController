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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Left-sample cycle", group = "Autonomous")  //Name Confusion NEEDS FIXES
public class BlueRight extends LinearOpMode {
    private RobotHardware robotHardware;
    private SampleMecanumDrive drive;

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
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270));
            drive.setPoseEstimate(startPose);

            // Create a trajectory sequence
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(16)  // Strafe left
                    .back(38.8)  // Move backward

                    .addDisplacementMarker(38.8, () -> robot.lifts.GoToPositionVertical(3800)) //Lifts to height 3800 high
                    .addDisplacementMarker(40.3, () -> {
                        robot.claw.armUp(); //Arm goes up
                        robot.claw.wristUP(); //Wrist goes up
                    })
                    .turn(Math.toRadians(63))  // Turn the robot
                    .strafeLeft(2)  // Strafe left again
                    .addDisplacementMarker(42.3, () -> robot.claw.clawOpen())
                    .waitSeconds(0.14)  // Wait for a short period
                    .addDisplacementMarker(44.3, () -> {
                        robot.claw.wristDown(); //Wrist goes down
                        robot.claw.armRest(); //Arm goes down
                        robot.lifts.GoToPositionVertical(0);
                    })
                    .turn(Math.toRadians(54.3))  // Turn the robot
                    .addDisplacementMarker(46.8, () -> {
                        robot.intake.pitchDown();   //Pitch goes down for intake process
                        robot.intake.startIntake(); //Begin Intake Motors
                    })
                    .forward(26.8)  // Move forward with wait codes
                    .waitSeconds(0.78)
                    .addDisplacementMarker(73.6, () -> robot.intake.pitchUp())  //WAIT TO Pitch Up
                    .waitSeconds(0.5)
                    .addDisplacementMarker(76.8, () -> robot.intake.stopIntake()) //WAIT TO Stop Intake Motors
                    .waitSeconds(0.03)
                    .addDisplacementMarker(80, () -> robot.claw.grab())  //WAIT TO Grab Block
                    .back(26.8)  // Move back
                    .turn(Math.toRadians(-54))  // Turn the robot
                    .addDisplacementMarker(107, () -> robot.lifts.GoToPositionVertical(3800)) //Raise up to 3800 height
                    .addDisplacementMarker(109.2, () -> {
                        robot.claw.armUp(); //Raise ArmUp
                        robot.claw.wristUP(); //Raise wristUP
                    })
                    .back(6.3)  // Move back again
                    .waitSeconds(0.03)
                    .addDisplacementMarker(115.5, () -> robot.claw.clawOpen())// WAIT TO Claw OPEN
                    .waitSeconds(0.15)
                    .addDisplacementMarker(115.8, () -> {
                        robot.claw.wristDown();         //Wrist down
                        robot.claw.armRest();           //Arm rest
                        robot.lifts.GoToPositionVertical(0); //Lifts go all the way down
                    })
                    .strafeRight(5)  // Strafe right to finish
                    .build();  // Build the trajectory sequence

            // Follow the trajectory sequence
            drive.followTrajectorySequence(trajSeq);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}
