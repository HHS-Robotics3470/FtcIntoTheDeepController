package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Components.Lifts;
import org.firstinspires.ftc.teamcode.Components.Claw;

@Autonomous(name = "Blue Left", group = "Autonomous")
public class BlueLeft extends LinearOpMode {
    private RobotHardware robotHardware;
    private RobotHardware Mecnum;
    private Lifts lifts;
    private Claw claw;
    private SampleMecanumDrive drive;


    // Speed control constants
    public final double DRIVE_SPEED_MAX_AUTO = 0.5;
    public final double DRIVE_SPEED_SLOW_AUTO = 0.05;
    public double driveSpeedControl = DRIVE_SPEED_MAX_AUTO;

    // Individual motor speed scaling
    public double AUTOspeedFLeft = 1.0;
    public double AUTOspeedFRight = 1.0;
    public double AUTOspeedBLeft = 2.5; // Back should have more power
    public double AUTOspeedBRight = 2.5; // Back should have more power

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robotHardware = new RobotHardware(this);
        drive = new SampleMecanumDrive(this.hardwareMap); // Initialize with 'this' LinearOpMode
        robotHardware.init();  // Call init() to set up the hardware

        Trajectory traj1 =  drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(5, 5), Math.toRadians(270))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();


        if (opModeIsActive()) {
//            // Move straight left
//            robotHardware.mecnum.strafe(1, 1000); // Adjust time or power as needed
//
//            // Rotate around (180 degrees)
//            robotHardware.mecnum.rotate(0.5, 1500); // Adjust timing for a full 180-degree turn
//            sleep(500);

            drive.followTrajectory(traj1);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    /**
     * Set power for each motor with independent speed adjustments.
     */
    public void setDrivePower(double autoPower) {
        robotHardware.fLeft.setPower(autoPower * driveSpeedControl * AUTOspeedFLeft);
        robotHardware.fRight.setPower(autoPower * driveSpeedControl * AUTOspeedFRight);
        robotHardware.bLeft.setPower(autoPower * driveSpeedControl * AUTOspeedBLeft);
        robotHardware.bRight.setPower(autoPower * driveSpeedControl * AUTOspeedBRight);
    }

}
