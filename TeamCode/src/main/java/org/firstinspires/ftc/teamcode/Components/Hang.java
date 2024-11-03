package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Hang {
    private LinearOpMode myOpMode;
    private DcMotor hangMotor;
    private Servo hangServo;

    // Constants for safety
    private static final double MAX_HANG_POWER = 1.0; // Maximum power for lifting
    private static final double MIN_HANG_POWER = -1.0; // Maximum power for lowering
    private static final int HANG_TIMEOUT_MS = 3000; // Time to stop hanging if it exceeds the limit

    // State variables
    private boolean isHanging = false;
    private long hangStartTime;

    public Hang(LinearOpMode opMode, DcMotor hangMotor, Servo hangServo) {
        myOpMode = opMode;
        this.hangMotor = hangMotor;
        this.hangServo = hangServo;

        // Set the initial position of the servo
        hangServo.setPosition(0.0); // Assuming this is the unlocked position
    }

    // Method to lift the robot
    public void lift() {
        hangMotor.setPower(MAX_HANG_POWER);
        isHanging = true;
        hangStartTime = System.currentTimeMillis();
    }

    // Method to lower the robot
    public void lower() {
        hangMotor.setPower(MIN_HANG_POWER);
        isHanging = true;
        hangStartTime = System.currentTimeMillis();
    }

    // Method to stop the lift
    public void stop() {
        hangMotor.setPower(0);
        isHanging = false;
    }

    // Method to lock the hang mechanism
    public void lock() {
        hangServo.setPosition(1.0); // Adjust based on servo mechanics
    }

    // Method to unlock the hang mechanism
    public void unlock() {
        hangServo.setPosition(0.0); // Adjust based on servo mechanics
    }

    // Safety checks to stop lifting if it takes too long
    public void checkHangTimeout() {
        if (isHanging && (System.currentTimeMillis() - hangStartTime) > HANG_TIMEOUT_MS) {
            stop();
            myOpMode.telemetry.addData("Hang", "Timeout! Stopped lifting.");
            myOpMode.telemetry.update();
        }
    }
}
