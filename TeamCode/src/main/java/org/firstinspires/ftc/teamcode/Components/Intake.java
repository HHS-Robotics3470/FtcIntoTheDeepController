package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Intake implements Component {

    // Declare a CRServo object for the continuous intake motor
    public CRServo intakeMotor;

    // Declare a Servo object for the intake pitch control
    public Servo intakePitch;

    // Initialize the intake motor and pitch servo using RobotHardware
    @Override
    public void init(RobotHardware robotHardware) {
        // Retrieve hardware components from RobotHardware
        intakeMotor = robotHardware.roller; // Assuming roller is defined as a CRServo in RobotHardware
        intakePitch = robotHardware.intakePitch; // Assuming intakePitch is defined as a Servo in RobotHardware

        // Set motor direction for intake motor (CRServo)
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // or Direction.REVERSE as needed

        // Set initial position for the intake pitch servo
        intakePitch.setPosition(0.5); // Neutral position; adjust range [0.0, 1.0] as needed
    }


    // Set power to the intake motor
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    // Set position for the intake pitch servo
    public void setIntakePitch(double position) {
        // Ensure the position is within the servo's range
        position = Math.max(0.0, Math.min(position, 1.0)); // Clamp between 0.0 and 1.0
        intakePitch.setPosition(position);
    }

    // Optional: Method to stop the intake motor
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
}
