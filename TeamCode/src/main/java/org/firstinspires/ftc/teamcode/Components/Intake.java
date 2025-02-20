package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake implements Component {

    public final double PITCH_DOWN = 0.08;
    public final double PITCH_UP = 0.05;
    public final double PITCH_TRANSFER = 0.12;
    public final double PITCH_SPECIMEN_REST = 0.2;
    public final double PITCH_TRANSFER_SAMPLE = 0.15;
    public final double PITCH_INTAKING = 0;
    public final double PITCH_INTAKE_READY = 0.08;
    public final double PITCH_REST = 0.0588;
    private final double INTAKE_CLAW_OPEN_POSITION = 0.2;
    private final double INTAKE_CLAW_CLOSE_POSITION = 0.015;

    public final double SWEEPER_INITIAL_POSITION = 0.008;
    public final double SWEEPER_FINAL_POSITION = 0.053;
    public final double SWEEPER_AUTO_POSITION = 0.34;

    public final double FOUR_PITCH_TRANSFER = 0.02 ;
    public final double FOUR_PITCH_INTAKING = 1;

    private boolean ifPressed;

    // Declare a CRServo object for the continuous intake motor
    //public CRServo intakeMotor;
    public Servo clawIntake;

    // Declare a Servo object for the intake pitch control
    public Servo intakePitch;
    public Servo intakeWrist;
    public Servo fourBarPitch;

    // Initialize the intake motor and pitch servo using RobotHardware
    @Override
    public void init(RobotHardware robotHardware) {
        clawIntake = robotHardware.clawIntake;
        intakePitch = robotHardware.intakePitch; // Assuming intakePitch is defined as a Servo in RobotHardware
        fourBarPitch = robotHardware.fourBarPitch;
        intakeWrist = robotHardware.intakeWrist;

        fourBarPitch.setDirection(Servo.Direction.FORWARD);
        intakeWrist.setDirection(Servo.Direction.FORWARD);

        intakePitch.setPosition(PITCH_TRANSFER);
        clawIntake.setPosition(INTAKE_CLAW_CLOSE_POSITION);
        intakeWrist.setPosition(SWEEPER_INITIAL_POSITION);
        fourBarPitch.setPosition(FOUR_PITCH_INTAKING);

        ifPressed = false;


    }


    // Start intake motor
   // public void startIntake() {
  //      intakeMotor.setPower(INTAKE_POWER);
  //  }

  //  public void passiveIntake() {
       // intakeMotor.setPower(PASSIVE_INTAKE_POWER);
  //  }
//    // Set position for the intake pitch servo
//    public void setIntakePitch(double position) {
//        // Ensure the position is within the servo's range
//        position = Math.max(0.0, Math.min(position, 1.0)); // Clamp between 0.0 and 1.0
//        intakePitch.setPosition(position);
//    }

    // Optional: Method to stop the intake motor
  //  public void stopIntake() {
   //     intakeMotor.setPower(0);
  //  }

  //  public void reverseIntake() {
  //      intakeMotor.setPower(-INTAKE_POWER);
  //  }

    public void pitchUp()
    {
        intakePitch.setPosition(PITCH_UP);
    }

    public void pitchSpecimenRest()
    {
        intakePitch.setPosition(PITCH_SPECIMEN_REST);
    }

    public void pitchDown()
    {
        intakePitch.setPosition(PITCH_DOWN);
    }

    public void pitchRest(){
        intakePitch.setPosition(PITCH_REST);
    }

    //for claw intake
    public void pitchTransfer()
    {
        intakePitch.setPosition(PITCH_TRANSFER_SAMPLE);
    }
    public void pitchIntakeReady()
    {
        intakePitch.setPosition(PITCH_INTAKE_READY);
    }
    public void pitchIntaking()
    {
        intakePitch.setPosition(PITCH_INTAKING);
    }

    public void clawIntakeClose()
    {
        clawIntake.setPosition(INTAKE_CLAW_CLOSE_POSITION);
    }
    public void clawIntakeOpen()
    {
        clawIntake.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }


    public void fourBarIntaking()
    {
        fourBarPitch.setPosition(FOUR_PITCH_INTAKING);
    }
    public void fourBarTransfer()
    {
        fourBarPitch.setPosition(FOUR_PITCH_TRANSFER);
    }

    public void sweeperInitial(){
        intakeWrist.setPosition(SWEEPER_INITIAL_POSITION);
    }

    public void sweeperFinal() {intakeWrist.setPosition(SWEEPER_FINAL_POSITION);}

    public void sweeperAuto() {
        intakeWrist.setPosition(SWEEPER_AUTO_POSITION);
    }

    public void wristing(){
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
    }





    public void toggle() {
        if (Math.abs(intakePitch.getPosition()-PITCH_UP) < 0.0001) {
            pitchDown();
        } else {
            pitchUp();
        }
    }




}