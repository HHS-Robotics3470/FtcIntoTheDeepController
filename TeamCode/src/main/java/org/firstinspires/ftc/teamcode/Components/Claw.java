package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Claw {
    private LinearOpMode myOpMode;
    private Servo clawServo;     // Servo to open/close the claw
    private Servo clawPitch;     // Servo to adjust the pitch of the claw

    // Constants for servo positions
    private static final double CLAW_OPEN_POSITION = 1.0;   // Adjust as needed for your claw design
    private static final double CLAW_CLOSE_POSITION = 0.0;  // Adjust as needed for your claw design
    private static final double PITCH_UP_POSITION = 1.0;    // Adjust as needed for your pitch servo
    private static final double PITCH_DOWN_POSITION = 0.0;  // Adjust as needed for your pitch servo

    public Claw(LinearOpMode opMode, Servo clawServo, Servo clawPitch) {
        myOpMode = opMode;
        this.clawServo = clawServo;
        this.clawPitch = clawPitch;

        // Initialize the claw and pitch positions
        clawServo.setPosition(CLAW_OPEN_POSITION);  // Start with the claw open
        clawPitch.setPosition(PITCH_DOWN_POSITION);  // Start with the pitch down
    }

    // Method to open the claw
    public void open() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    // Method to close the claw
    public void close() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
    }

    // Method to set the pitch of the claw up
    public void pitchUp() {
        clawPitch.setPosition(PITCH_UP_POSITION);
    }

    // Method to set the pitch of the claw down
    public void pitchDown() {
        clawPitch.setPosition(PITCH_DOWN_POSITION);
    }

    // Method to toggle the claw's open/close state
    public void toggle() {
        if (clawServo.getPosition() == CLAW_OPEN_POSITION) {
            close();
        } else {
            open();
        }
    }

    // Method to stop the claw servo (optional, for safety)
    public void stop() {
        // Currently, the servo position is set directly, no need for a stop method.
        // You may want to add functionality here if needed in the future.
    }
}
