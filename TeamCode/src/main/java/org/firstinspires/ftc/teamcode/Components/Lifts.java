package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Lifts implements Component {
    private LinearOpMode myOpMode = null;

    // Constructor
    public Lifts(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    // Lift motors
    public DcMotor lLift;
    public DcMotor rLift;

    // Init function
    public void init(RobotHardware robotHardware) {
        // Initialize lift motors from RobotHardware
        lLift = robotHardware.lLift;
        rLift = robotHardware.rLift;
    }

    // Raise Lift function
    public void raiseLift() {
        if (lLift.getCurrentPosition() < 0) {
            rLift.setTargetPosition(0);
            lLift.setTargetPosition(0);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(1);
            lLift.setPower(1);
        }
    }

    // Lower Lift function
    public void lowerLift() {
        if (rLift.getCurrentPosition() > -4250) {
            rLift.setTargetPosition(-4275);
            lLift.setTargetPosition(-4275);
            rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rLift.setPower(-1);
            lLift.setPower(-1);
        }
    }

    // Stop Lift function
    public void stopLift() {
        rLift.setTargetPosition(rLift.getCurrentPosition());
        lLift.setTargetPosition(lLift.getCurrentPosition());
    }
}
