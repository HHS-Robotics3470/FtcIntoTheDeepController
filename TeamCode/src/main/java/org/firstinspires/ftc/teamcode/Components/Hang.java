package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Hang implements Component{
    private DcMotorEx hangMotor;
    private Servo hangLock1;
    private Servo hangLock2;
    private OpMode myOpMode;

    private final double LOCK_POS= 1;
    private final double UNLOCK_POS= 0;

    // Constants for safety
    private static final double MAX_HANG_POWER = 1.0; // Maximum power for lifting
    private static final double MIN_HANG_POWER = -1.0; // Maximum power for lowering
    private static final int HANG_TIMEOUT_MS = 3000; // Time to stop hanging if it exceeds the limit

    // State variables
    private boolean isHanging = false;
    private long hangStartTime;

    @Override
    public void init(RobotHardware robotHardware) {
        hangMotor = robotHardware.hangMotor;
        hangLock1 = robotHardware.lock1;
        hangLock2 = robotHardware.lock2;
        myOpMode = robotHardware.myOpMode;
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
        hangLock1.setPosition(LOCK_POS);
        hangLock2.setPosition(LOCK_POS);// Adjust based on servo mechanics
    }

    // Method to unlock the hang mechanism
    public void unlock() {
        hangLock1.setPosition(UNLOCK_POS);
        hangLock2.setPosition(UNLOCK_POS); // Adjust based on servo mechanics
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
