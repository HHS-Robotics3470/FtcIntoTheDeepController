package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Claw implements Component{
    private LinearOpMode myOpMode;
    private Servo clawServo;     // Servo to open/close the claw
    private Servo armRight;
    private Servo armLeft;
    private Servo wrist;// Servo to adjust the pitch of the claw
    private Servo hang;

    private boolean ifSwinged = false;
    //
    //

    // Constants for servo positions
    private final double CLAW_OPEN_POSITION = 0.056;   // Adjust as needed for your claw design
    private final double CLAW_CLOSE_POSITION = 0;  // Adjust as needed for your claw design
    private final double ARM_UP_POSITION = 0.325;    // Adjust as needed for your pitch servo
    private final double ARM_DOWN_POSITION = 0.228;
    private final double ARM_REST_POSITION = 0.252;
    private final double WRIST_UP_POSITION = 0.35;
    private final double WRIST_AUTO_POSITION = 0.5;
    private final double WRIST_DOWN_POSITION = 0.237
            ;
    private final double WRIST_SPECIMEN = 0.345;
    private final double ARM_SPECIMEN = 0.35;
    private final double ARM_AUTO = 0.315;
    private final double HANG_INITIAL = 0;
    private final double HANG_ACTIVATED = 0.115;


    ;// Adjust as needed for your pitch servo

    @Override
    public void init(RobotHardware robotHardware) {
        myOpMode = robotHardware.myOpMode;
        wrist = robotHardware.wrist;
        clawServo = robotHardware.clawServo;
        armRight = robotHardware.armRight;
        armLeft = robotHardware.armLeft;
        hang = robotHardware.lock1;

        hang.setDirection(Servo.Direction.REVERSE);

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.FORWARD);

        wrist.setDirection(Servo.Direction.REVERSE);

        clawServo.setPosition(CLAW_CLOSE_POSITION);
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
        wrist.setPosition(WRIST_DOWN_POSITION);
        wrist.setPosition(WRIST_UP_POSITION);
        hang.setPosition(HANG_INITIAL);

//
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

    public void ThreadSleep(int milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void grab()
    {
        clawOpen();
        wristDown();
        ThreadSleep(150);
        armDown();
        ThreadSleep(250);
        clawClose();
        ThreadSleep(400);
        armRest();
        ThreadSleep(300);
        wristUP();
    }
    //areeb the set position right now is good to transfer but when you use the wristDown() command it rotatates weirdly idk how to explain it wrist up is alaso scuffed idk <3

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
            wristUP();
            ifSwinged = false;
        }
    }

    public void specimen()
    {
        wrist.setPosition(WRIST_SPECIMEN);
        armRight.setPosition(ARM_SPECIMEN);
        armLeft.setPosition(ARM_SPECIMEN);
    }

    public void specimenAuto() {
        wrist.setPosition(WRIST_AUTO_POSITION);
        armRight.setPosition(ARM_AUTO);
        armLeft.setPosition(ARM_AUTO);
    }

    public void lvl1hang()
    {
        if (Math.abs(hang.getPosition() - HANG_ACTIVATED) < 0.0001) {
            hang.setPosition(HANG_INITIAL);
        } else {
            hang.setPosition(HANG_ACTIVATED);
        }
    }
//
//    // Method to stop the claw servo (optional, for safety)
//    public void stop() {
//        // Currently, the servo position is set directly, no need for a stop method.
//        // You may want to add functionality here if needed in the future.
//    }


}