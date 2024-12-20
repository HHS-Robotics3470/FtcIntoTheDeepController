package org.firstinspires.ftc.teamcode.Components;

import android.os.SystemClock;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import android.os.SystemClock;

public class Intake implements Component {

    public final double PITCH_TRANSFER = 0.205;
    public final double PITCH_INTAKE_READY = 0.06;
    public final double PITCH_INTAKING = 0;

    //public final double INTAKE_POWER = 1;
    private final double INTAKE_CLAW_OPEN_POSITION = 0.2;   // Adjust as needed for your claw design
    private final double INTAKE_CLAW_CLOSE_POSITION = 0.015;  // Adjust as needed for your claw design
    private final double INTAKE_CLAW_WRIST1_POSITION = 0;
    private final double INTAKE_CLAW_WRIST2_POSITION = 0.1;

    // Declare a CRServo object for the continuous intake motor
    //public CRServo intakeMotor;

    // Declare a Servo object for the intake pitch control
    public Servo intakePitch;
    public Servo clawIntake;
    public Servo wristIntake;
    private boolean ifWristing = false;



    // Initialize the intake motor and pitch servo using RobotHardware
    @Override
    public void init(RobotHardware robotHardware) {
        // Retrieve hardware components from RobotHardware
        //intakeMotor = robotHardware.roller; // Assuming roller is defined as a CRServo in RobotHardware
        intakePitch = robotHardware.intakePitch; // Assuming intakePitch is defined as a Servo in RobotHardware
        clawIntake = robotHardware.clawIntake;
        wristIntake = robotHardware.wristIntake;

        // Set motor direction for intake motor (CRServo)
        //intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // or Direction.REVERSE as needed
        // Set initial position for the intake pitch servo
        intakePitch.setPosition(PITCH_TRANSFER); // Neutral position; adjust range [0.0, 1.0] as needed
        clawIntake.setPosition(INTAKE_CLAW_OPEN_POSITION);
        wristIntake.setPosition(INTAKE_CLAW_WRIST1_POSITION);

        ifWristing = false;
    }

    public void ThreadSleep(int milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void intakeOut() {
        clawIntake.setPosition(INTAKE_CLAW_OPEN_POSITION);
        intakePitch.setPosition(PITCH_INTAKE_READY);
    }
    public void intakeRelease() {
        clawIntake.setPosition(INTAKE_CLAW_OPEN_POSITION);
        ThreadSleep(250);
    }
    public void wristing()
    {
        if (!ifWristing)
        {
            wristIntake.setPosition(INTAKE_CLAW_WRIST2_POSITION);
            ifWristing = true;
        }
        else
        {
            wristIntake.setPosition(INTAKE_CLAW_WRIST1_POSITION);
            ifWristing = false;
        }
    }



    public void intaking() {
            intakePitch.setPosition(PITCH_INTAKE_READY);
            clawIntake.setPosition(INTAKE_CLAW_OPEN_POSITION);
            intakePitch.setPosition(PITCH_INTAKING);
            ThreadSleep(500);
            clawIntake.setPosition(INTAKE_CLAW_CLOSE_POSITION);
            ThreadSleep(500);
            wristIntake.setPosition(INTAKE_CLAW_WRIST1_POSITION);
            intakePitch.setPosition(PITCH_TRANSFER);
        }
    }

    // Start intake motor
    /*
    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }
    */
//    // Set position for the intake pitch servo
//    public void setIntakePitch(double position) {
//        // Ensure the position is within the servo's range
//        position = Math.max(0.0, Math.min(position, 1.0)); // Clamp between 0.0 and 1.0
//        intakePitch.setPosition(position);
//    }

    // Optional: Method to stop the intake motor
    /*
    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void reverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void pitchUp()
    {
        intakePitch.setPosition(PITCH_UP);
    }

    public void pitchDown()
    {
        intakePitch.setPosition(PITCH_DOWN);
    }

    public void toggle() {
        if (Math.abs(intakePitch.getPosition()-PITCH_UP) < 0.0001) {
            pitchDown();
        } else {
            pitchUp();
        }
    }

     */


