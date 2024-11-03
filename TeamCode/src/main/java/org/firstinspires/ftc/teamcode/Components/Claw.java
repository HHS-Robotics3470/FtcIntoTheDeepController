package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Claw implements Component{
    private LinearOpMode myOpMode;
    private Servo clawServo;     // Servo to open/close the claw
    private Servo clawPitch;     // Servo to adjust the pitch of the claw

    // Constants for servo positions
    private final double CLAW_OPEN_POSITION = 1.0;   // Adjust as needed for your claw design
    private final double CLAW_CLOSE_POSITION = 0.0;  // Adjust as needed for your claw design
    private final double PITCH_UP_POSITION = 1.0;    // Adjust as needed for your pitch servo
    private final double PITCH_DOWN_POSITION = 0.0;  // Adjust as needed for your pitch servo

    @Override
    public void init(RobotHardware robotHardware) {
        clawServo = robotHardware.claw;
        clawPitch = robotHardware.clawPitch;
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
        if (Math.abs(clawServo.getPosition()-CLAW_OPEN_POSITION) < 0.0001) {
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
