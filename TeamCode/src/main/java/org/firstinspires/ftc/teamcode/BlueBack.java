package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;

@Disabled
@Autonomous(name = "AutonomousOp", group = "Autonomous")
public class BlueBack extends LinearOpMode {
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
            //Auto code starts (One side)



//           // Step 1: Move forward for parking distance
//            moveForward(0.5, 1000); // Move forward with 50% power for 1 second
//            sleep(500); // Pause for stability
//
//            // Optional: Step 2: Rotate robot to align for parking
//            rotate(0.5, 500, true); // Rotate right at 50% power for 0.5 seconds
//            sleep(500);
//
//            // Step 3: Move forward a bit more to park
//            moveForward(0.3, 800); // Move forward with 30% power for 0.8 seconds
//            stopMoving(); // Stop the robot



            //-----------------------------------------
            //OPPOSITE SIDE

            //// Step 1: Strafe left to align with parking lane
            //            strafe(0.5, 800, false);  // Strafe left at 50% power for 0.8 seconds
            //            sleep(500);
            //
            //            // Step 2: Move forward to parking position
            //            moveForward(0.5, 1200);  // Move forward at 50% power for 1.2 seconds
            //            sleep(500);
            //
            //            // Optional: Step 3: Rotate to align if necessary
            //            rotate(0.5, 500, true);  // Rotate right at 50% power for 0.5 seconds
            //            sleep(500);
            //
            //            // Step 4: Final forward movement for parking
            //            moveForward(0.3, 800);  // Move forward at 30% power for 0.8 seconds
            //            stopMoving();  // Stop the robot

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
    private void moveForward(double power, int times) {

        for (int i = 0; i < times; i++)
        {
            robotHardware.fLeft.setPower(power);
            robotHardware.fRight.setPower(power);
            robotHardware.bLeft.setPower(power);
            robotHardware.bRight.setPower(power);
        }

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
     */
    private void strafe(double power, int times) {
        for (int i = 0; i < times; i++)
        {
            robotHardware.fLeft.setPower(power);
            robotHardware.fRight.setPower(-power);
            robotHardware.bLeft.setPower(-power);
            robotHardware.bRight.setPower(power);
        }
    }

    /**
     * Rotate the robot either left or right.
     *
     * @param power the power level to set for the motors
     */
    private void rotate(double power, int times) {
        for (int i = 0; i < times; i++)
        {
            robotHardware.fLeft.setPower(power);
            robotHardware.fRight.setPower(-power);
            robotHardware.bLeft.setPower(-power);
            robotHardware.bRight.setPower(power);
        }
    }


}
