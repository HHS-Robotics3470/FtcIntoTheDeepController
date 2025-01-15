package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;

@Autonomous(name = "also don't use", group = "Autonomous")
public class RedRight extends LinearOpMode {
    private RobotHardware robotHardware;

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robotHardware = new RobotHardware(this);
        robotHardware.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start of the match
        waitForStart();

        if (opModeIsActive()) {
            // Strafe Northwest
            strafe(-1, 1500); // Adjust the duration for strafing NW
            stopMoving();

            // Raise Lift
            //robotHardware.lifts.raiseLift();
            //sleep(1000); // Allow time for lift to reach position

            // Lower Lift Slightly
            //robotHardware.lifts.lowerLift();
            //sleep(500); // Adjust for the partial lowering of lift


            // Open Claw
            //robotHardware.claw.clawOpen();
            //sleep(500);

            // Back Up
            moveForward(-0.5, 1000); // Move backward with moderate speed
            stopMoving();

            // Fully Lower Lift
            //robotHardware.lifts.lowerLift();
            //sleep(1000); // Allow enough time for the lift to fully lower

            // Close Claw
            //robotHardware.claw.clawClose();
            //sleep(500);

            // Move Southeast to Park
            strafe(1, 1500); // Adjust the duration for strafing SE
            stopMoving();

            // Turn around (180 degrees)
            rotate(0.5, 1500); // Adjust timing for a full 180-degree turn
            sleep(500);

            telemetry.addData("Status", "Autonomous Routine Complete");
            telemetry.update();
        }
    }

    /**
     * Move the robot forward/backward for a specific duration.
     *
     * @param power the power level to set for the motors
     * @param time  the duration to move
     */
    private void moveForward(double power, int time) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(power);
        robotHardware.bLeft.setPower(power);
        robotHardware.bRight.setPower(power);
        sleep(time);
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
     * Strafe the robot in a direction.
     *
     * @param power the power level to set for the motors
     * @param time  the duration to strafe
     */
    private void strafe(double power, int time) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(-power);
        robotHardware.bLeft.setPower(-power);
        robotHardware.bRight.setPower(power);
        sleep(time);
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
