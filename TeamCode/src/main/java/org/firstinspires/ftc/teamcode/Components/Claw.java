package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Claw implements Component{
    private LinearOpMode myOpMode;
    private Servo clawServo;     // Servo to open/close the claw
    private Servo armRight;
    private Servo armLeft;
    private Servo wrist;// Servo to adjust the pitch of the claw
    private Servo hang;
    private Lifts lifts;
    private boolean ifSwinged = false;
    private boolean ifOpen = false;
    private boolean ifSpecimen = false;

    private enum GRAB_STATE{
        INACTIVE,
        ACTION1,
        WAIT1,
        ACTION2,
        WAIT2,
        ACTION3,
        WAIT3,
        ACTION4,
        WAIT4,
        ACTION5
    }
    private ElapsedTime time = new ElapsedTime();
    private GRAB_STATE grabCurrentState = GRAB_STATE.INACTIVE;
    //
    //

    // Constants for servo positions
    private final double CLAW_OPEN_POSITION = 0.056;   // Adjust as needed for your claw design
    private final double CLAW_CLOSE_POSITION = 0.0;  // Adjust as needed for your claw design
    private final double ARM_UP_POSITION = 0.328;    // Adjust as needed for your pitch servo
    private final double ARM_DOWN_POSITION = 0.27375;
    private final double ARM_REST_POSITION = 0.28;
    private final double ARM_SPEC_INTAKE_POSITION = 0.259; //intake spec arm
    private final double WRIST_UP_POSITION = 0.455;
    private final double WRIST_AUTO_POSITION = 0.6; //.5 //outtake spec wrist
    private final double WRIST_DOWN_POSITION = 0.23;

    private final double WRIST_SPECIMEN = 0.42; //intake spec wrist
    private final double ARM_SPECIMEN = 0.243;
    private final double ARM_AUTO = 0.315; //outtake spec arm
    private final double HANG_INITIAL = -0.115;
    private final double HANG_ACTIVATED = 0;


    ;// Adjust as needed for your pitch servo

    @Override
    public void init(RobotHardware robotHardware) {
        myOpMode = robotHardware.myOpMode;
        wrist = robotHardware.wrist;
        clawServo = robotHardware.clawServo;
        armRight = robotHardware.armRight;
        armLeft = robotHardware.armLeft;
        hang = robotHardware.lock1;
        lifts = robotHardware.lifts;

        hang.setDirection(Servo.Direction.REVERSE);

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.FORWARD);

        clawServo.setPosition(CLAW_CLOSE_POSITION);
        ifOpen = false;
        armRight.setPosition(ARM_REST_POSITION);
        armLeft.setPosition(ARM_REST_POSITION);
        wrist.setPosition(WRIST_DOWN_POSITION);
        wrist.setPosition(WRIST_UP_POSITION);
        hang.setPosition(HANG_INITIAL);

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

    public void grabParallel()
    {
        if (grabCurrentState == GRAB_STATE.INACTIVE) {
            grabCurrentState = GRAB_STATE.ACTION1;
        }
    }

    public void grabUpdate() {
        switch (grabCurrentState)
        {
            case ACTION1:
                clawOpen();
                wristDown();
                time.reset();
                grabCurrentState = GRAB_STATE.WAIT1;
                break;
            case WAIT1:
                if (time.milliseconds() == 150) {
                    grabCurrentState = GRAB_STATE.ACTION2;
                }
                break;
            case ACTION2:
                armDown();
                time.reset();
                grabCurrentState = GRAB_STATE.WAIT2;
            case WAIT2:
                if (time.milliseconds() == 250)
                {
                    grabCurrentState = GRAB_STATE.ACTION3;
                }
                break;
            case ACTION3:
                clawClose();
                time.reset();
                grabCurrentState = GRAB_STATE.WAIT3;
                break;
            case WAIT3:
                if (time.milliseconds() == 400)
                {
                    grabCurrentState = GRAB_STATE.ACTION4;
                }
                break;
            case ACTION4:
                armRest();
                time.reset();
                grabCurrentState = GRAB_STATE.WAIT4;
                break;
            case WAIT4:
                if (time.milliseconds() == 300)
                {
                    grabCurrentState = GRAB_STATE.ACTION5;
                }
                break;
            case ACTION5:
                wristUP();
                grabCurrentState = GRAB_STATE.INACTIVE;
                break;
        }
    }

    // Method to toggle the claw's open/close state
    public void toggleClaw() {
        if (ifOpen) {
            clawClose();
            ifOpen = false;
        } else {
            clawOpen();
            ifOpen = true;
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

    public void specimen() //intake spec
    {
        wrist.setPosition(WRIST_SPECIMEN);
        armRight.setPosition(ARM_SPEC_INTAKE_POSITION);
        armLeft.setPosition(ARM_SPEC_INTAKE_POSITION);
        clawOpen();
    }

    public void specimenAuto() //outtake spec
    {
        wrist.setPosition(WRIST_AUTO_POSITION);
        armRight.setPosition(ARM_AUTO);
        armLeft.setPosition(ARM_AUTO);
        clawClose();
    }

    public void lvl1hang()
    {
        if (Math.abs(hang.getPosition() - HANG_ACTIVATED) < 0.0001) {
            hang.setPosition(HANG_INITIAL);
        } else {
            hang.setPosition(HANG_ACTIVATED);
        }
    }

    public boolean toggleSpecimen()
    {
        if (ifSpecimen)
        {
            specimenAuto();
            ifSpecimen = false;
        }
        else
        {
            specimen();
            ifSpecimen = true;
        }
        return ifSpecimen;
    }
//
//    // Method to stop the claw servo (optional, for safety)
//    public void stop() {
//        // Currently, the servo position is set directly, no need for a stop method.
//        // You may want to add functionality here if needed in the future.
//    }


}