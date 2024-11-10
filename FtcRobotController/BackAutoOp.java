package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "BackAutoOp", group = "Autonomous")
public class BackAutoOp extends LinearOpMode {
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
            // Example parking sequence for the back side
            moveForward(0.5);    // Move forward
            sleep(1000);         // Adjust timing as needed
            stopMoving();
            sleep(500);

            // Rotate if necessary
            rotate(0.5, 500, true);
            sleep(500);

            // Forward final positioning
            moveForward(0.3);
            sleep(800);
            stopMoving();

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    private void moveForward(double power) {
        robotHardware.fLeft.setPower(power);
        robotHardware.fRight.setPower(power);
        robotHardware.bLeft.setPower(power);
        robotHardware.bRight.setPower(power);
    }

    private void stopMoving() {
        robotHardware.fLeft.setPower(0);
        robotHardware.fRight.setPower(0);
        robotHardware.bLeft.setPower(0);
        robotHardware.bRight.setPower(0);
    }

    private void strafe(double power, int durationMs, boolean right) {
        if (right) {
            robotHardware.fLeft.setPower(power);
            robotHardware.fRight.setPower(-power);
            robotHardware.bLeft.setPower(-power);
            robotHardware.bRight.setPower(power);
        } else {
            robotHardware.fLeft.setPower(-power);
            robotHardware.fRight.setPower(power);
            robotHardware.bLeft.setPower(power);
            robotHardware.bRight.setPower(-power);
        }
        sleep(durationMs);
        stopMoving();
    }

    private void rotate(double power, int durationMs, boolean right) {
        if (right) {
            robotHardware.fLeft.setPower(power);
            robotHardware.fRight.setPower(-power);
            robotHardware.bLeft.setPower(power);
            robotHardware.bRight.setPower(-power);
        } else {
            robotHardware.fLeft.setPower(-power);
            robotHardware.fRight.setPower(power);
            robotHardware.bLeft.setPower(-power);
            robotHardware.bRight.setPower(power);
        }
        sleep(durationMs);
        stopMoving();
    }
}
