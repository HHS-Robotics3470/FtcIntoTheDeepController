package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutonomousOp", group = "Autonomous")
public class AutonomousOp extends LinearOpMode {
    private RobotHardware robotHardware;

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robotHardware = new RobotHardware(this);  // Initialize with 'this' LinearOpMode
        robotHardware.init();  // Call init() to set up the hardware

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();

        if (opModeIsActive()) {
            //Auto code starts



//            // Step 1: Move forward
//            moveForward(0.5, 1000);  // Move forward with power 0.5 for 1000 ms
//            sleep(500);
//
//            // Step 2: Use claw to grab an object
//            robotHardware.claw.clawOpen();
//            sleep(500);
//            robotHardware.claw.armDown();
//            sleep(500);
//            robotHardware.claw.clawClose();  // Use robotHardware's claw to close
//            sleep(500);
//            robotHardware.claw.armRest();
//
//            // Step 3: Raise the lift
//            robotHardware.lifts.raiseLift();
//            sleep(1500);  // Allow time for the lift to reach its position
//
//            // Step 4: Move to drop zone
//            moveForward(0.5, 1500);  // Adjust power and time as needed
//            sleep(500);
//
//            // Step 5: Drop the object
//            robotHardware.claw.armDown();
//            sleep(500);
//            robotHardware.claw.clawOpen();
//            sleep(500);
//            robotHardware.claw.armRest();
//
//            // Step 6: Lower the lift
//            robotHardware.lifts.lowerLift();
//            sleep(1500);
//
//            // Step 7: Strafe to adjust position
//            strafe(0.5, 1000, true);  // Strafe right for 1000 ms
//            sleep(500);
//
//            // Step 8: Rotate robot (for example, rotate 90 degrees to the right)
//            rotate(0.5, 1000, true);  // Rotate right for 1000 ms
//            sleep(500);
//
//            // Step 9: Move to parking position
//            park();
//            sleep(500);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();

            //Auto code ends
        }
    }

    /**
     * Move the robot forward for a specific duration.
     *
     * @param power the power level to set for the motors
     */
    private void moveForward(double power) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(power);
        robotHardware.bLeft.setPower(power);
        robotHardware.bRight.setPower(power);
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
     * @param durationMs the duration to strafe in milliseconds
     * @param right whether to strafe right (true) or left (false)
     */
    private void strafe(double power, int durationMs, boolean right) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(-power);
        robotHardware.bLeft.setPower(-power);
        robotHardware.bRight.setPower(power);
    }

    /**
     * Rotate the robot either left or right.
     *
     * @param power the power level to set for the motors
     * @param durationMs the duration to rotate in milliseconds
     * @param right whether to rotate right (true) or left (false)
     */
    private void rotate(double power, int durationMs, boolean right) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(-power);
        robotHardware.bLeft.setPower(-power);
        robotHardware.bRight.setPower(power);
    }

    /**
     * Park the robot at the designated position.
     */
    private void park() {
        moveForward(0.3);  // Start moving forward with power 0.3
        sleep(2000);       // Continue moving for 2000 ms
        stopMoving();      // Stop the robot after the specified time
    }
}
