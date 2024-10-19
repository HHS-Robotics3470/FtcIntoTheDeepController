package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Intake implements Component {

    // Declare a LinearOpMode object
    private LinearOpMode myOpMode = null;

    // Declare a DcMotor object for the intake motor
    public DcMotor intake;

    // Constructor
    public Intake(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    // Initialize the intake motor using RobotHardware
    @Override
    public void init(RobotHardware robotHardware) {
        intake = robotHardware.intake;
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Alternative initialization without RobotHardware (if needed)
    public void init() {
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intakey");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set power to the intake motor
    public void setPower(double power) {
        intake.setPower(power);
    }
}
