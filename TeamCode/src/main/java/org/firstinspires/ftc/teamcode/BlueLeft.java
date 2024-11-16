package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.Components.Lifts;
import org.firstinspires.ftc.teamcode.Components.Claw;

@Autonomous(name = "Blue Left", group = "Autonomous")
public class BlueLeft extends LinearOpMode {
    private RobotHardware robotHardware;
    private Lifts lifts;
    private Claw claw;

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robotHardware = new RobotHardware(this);  // Initialize with 'this' LinearOpMode
        robotHardware.init();  // Call init() to set up the hardware

        lifts = new Lifts();  // Initialize the lift system
        lifts.init(robotHardware);

        claw = new Claw();  // Initialize the claw system
        claw.init(robotHardware);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();

        if (opModeIsActive()) {
            // Move straight left
            strafe(1, 1000); // Adjust time or power as needed

            // Raise the lift to the height limit
            //lifts.raiseLift();
            //sleep(2000); // Wait to ensure lift reaches the height

            // Open the claw to release the object
            //claw.clawOpen();
            //sleep(1000);

            // Open the claw to release the object
            // claw.clawClose();
            //sleep(1000);

            // Lower the lift to normal position
            //lifts.lowerLift();
            //sleep(2000);

            // Turn around (180 degrees)
            rotate(0.5, 1500); // Adjust timing for a full 180-degree turn
            sleep(500);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    /**
     * Move the robot forward for a specific duration.
     *
     * @param power the power level to set for the motors
     * @param time the duration to move in milliseconds
     */
    private void moveForward(double power, int time) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(power);
        robotHardware.bLeft.setPower(power);
        robotHardware.bRight.setPower(power);
        sleep(time);
        stopMoving();
    }

    /**
     * Stop all drive motors.
     */
    private void stopMoving() {
        robotHardware.fLeft.setPower(0);
        robotHardware.fRight.setPower(0);
        robotHardware.bLeft.setPower(0);
        robotHardware.bRight.setPower(0);
    }

    /**
     * Strafe the robot either left or right.
     *
     * @param power the power level to set for the motors
     * @param time the duration to move in milliseconds
     */
    private void strafe(double power, int time) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(-power);
        robotHardware.bLeft.setPower(-power);
        robotHardware.bRight.setPower(power);
        sleep(time);
        stopMoving();
    }

    /**
     * Rotate the robot either left or right.
     *
     * @param power the power level to set for the motors
     * @param time the duration to rotate in milliseconds
     */
    private void rotate(double power, int time) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(-power);
        robotHardware.bLeft.setPower(power);
        robotHardware.bRight.setPower(-power);
        sleep(time);
        stopMoving();
    }
}
