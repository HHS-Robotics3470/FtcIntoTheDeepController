package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Component {

    public final double PITCH_DOWN = 0.01;
    public final double PITCH_UP = 0.08;
    public final double PITCH_REST = 0.0508;
    public final double INTAKE_POWER = 1;
    public final double PASSIVE_INTAKE_POWER = 0.4;

    public final double SWEEPER_INITIAL_POSITION = 0;
    public final double SWEEPER_FINAL_POSITION = 0.71;

    private boolean ifPressed;

    // Declare a CRServo object for the continuous intake motor
    public CRServo intakeMotor;

    // Declare a Servo object for the intake pitch control
    public Servo intakePitch;
    public Servo sweeper;

    // Initialize the intake motor and pitch servo using RobotHardware
    @Override
    public void init(RobotHardware robotHardware) {
        // Retrieve hardware components from RobotHardware
        intakeMotor = robotHardware.roller; // Assuming roller is defined as a CRServo in RobotHardware
        intakePitch = robotHardware.intakePitch; // Assuming intakePitch is defined as a Servo in RobotHardware
        // Set motor direction for intake motor (CRServo)
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // or Direction.REVERSE as needed
        // Set initial position for the intake pitch servo
        intakePitch.setPosition(PITCH_UP); // Neutral position; adjust range [0.0, 1.0] as needed

        sweeper = robotHardware.sweeper;
        sweeper.setPosition(SWEEPER_INITIAL_POSITION);

        sweeper.setDirection(Servo.Direction.FORWARD);

        ifPressed = false;


    }


    // Start intake motor
    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void passiveIntake() {
        intakeMotor.setPower(PASSIVE_INTAKE_POWER);
    }
//    // Set position for the intake pitch servo
//    public void setIntakePitch(double position) {
//        // Ensure the position is within the servo's range
//        position = Math.max(0.0, Math.min(position, 1.0)); // Clamp between 0.0 and 1.0
//        intakePitch.setPosition(position);
//    }

    // Optional: Method to stop the intake motor
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

    public void pitchRest(){
        intakePitch.setPosition(PITCH_REST);
    }


    public void sweeperInitial(){
        sweeper.setPosition(SWEEPER_INITIAL_POSITION);
    }

    public void sweeperFinal() {sweeper.setPosition(SWEEPER_FINAL_POSITION);}

    public void sweeperPress(){
        if (!ifPressed)
        {
           sweeperFinal();
            ifPressed = true;
        }
        else
        {
           sweeperInitial();
            ifPressed = false;
        }
    };





    public void toggle() {
        if (Math.abs(intakePitch.getPosition()-PITCH_UP) < 0.0001) {
            pitchDown();
        } else {
            pitchUp();
        }
    }




}