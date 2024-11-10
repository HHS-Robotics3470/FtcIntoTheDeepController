package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Claw implements Component{
    private LinearOpMode myOpMode;
    private Servo clawServo;     // Servo to open/close the claw
    private Servo armRight;
    private Servo armLeft;
    private Servo wrist;// Servo to adjust the pitch of the claw

    private boolean ifSwinged = false;

    // Constants for servo positions
    private final double CLAW_OPEN_POSITION = 0.05;   // Adjust as needed for your claw design
    private final double CLAW_CLOSE_POSITION = 0;  // Adjust as needed for your claw design
    private final double ARM_UP_POSITION = 1.0;    // Adjust as needed for your pitch servo
    private final double ARM_DOWN_POSITION = 0.0;
    private final double ARM_REST_POSITION = 0.5;
    private final double WRIST_UP_POSITION = 0.45;
    private final double WRIST_DOWN_POSITION = 0.1;// Adjust as needed for your pitch servo

    @Override
    public void init(RobotHardware robotHardware) {
        clawServo = robotHardware.clawServo;
        armRight = robotHardware.armRight;
        armLeft = robotHardware.armLeft;
        wrist = robotHardware.wrist;

        armRight.setDirection(Servo.Direction.FORWARD);
        armLeft.setDirection(Servo.Direction.REVERSE);

        clawServo.setPosition(CLAW_OPEN_POSITION);
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
        wrist.setPosition(WRIST_DOWN_POSITION);

        ifSwinged = false;
    }

    // Method to open the claw
    public void clawOpen() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    // Method to close the claw
    public void clawClose() {
        clawServo.setPosition(CLAW_CLOSE_POSITION);
    }

    // Method to set the pitch of the claw up
    public void armUp() {
        armRight.setPosition(ARM_UP_POSITION);
        armLeft.setPosition(ARM_UP_POSITION);
    }

    // Method to set the pitch of the claw down
    public void armDown() {
        armRight.setPosition(ARM_DOWN_POSITION);
        armLeft.setPosition(ARM_DOWN_POSITION);
    }

    public void armRest(){
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
    }

    public void wristUP(){
        wrist.setPosition(WRIST_UP_POSITION);
    }

    public void wristDown() {
        wrist.setPosition(WRIST_DOWN_POSITION);
    }

    public void grab()
    {
        clawOpen();
        wristDown();
        armDown();
        clawClose();
        armRest();
        wristUP();
    }

    // Method to toggle the claw's open/close state
    public void toggleClaw() {
        if (Math.abs(clawServo.getPosition() - CLAW_OPEN_POSITION) < 0.0001) {
            clawClose();
        } else {
            clawOpen();
        }
    }


    public void swing()
    {
        if (!ifSwinged)
        {
            armUp();
            wristUP();
            ifSwinged = true;
        }
        else
        {
            armRest();
            wristDown();
            ifSwinged = false;
        }


    }

//
//    // Method to stop the claw servo (optional, for safety)
//    public void stop() {
//        // Currently, the servo position is set directly, no need for a stop method.
//        // You may want to add functionality here if needed in the future.
//    }


}
