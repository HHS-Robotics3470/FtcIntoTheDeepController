//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.Components.RobotHardware;
//
//@Autonomous(name = "BlueRight", group = "Autonomous")
//public class BlueRight extends LinearOpMode {
//    private RobotHardware robotHardware;
//
//    @Override
//    public void runOpMode() {
//        // Initialize hardware and components
//        robotHardware = new RobotHardware(this);
//        robotHardware.init();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Wait for the start of the match
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Strafe Northwest
//            strafe(-1, 1500);
//            stopMoving();
//
//            // Back Up
//            moveForward(-0.5, 1000);
//            stopMoving();
//
//            // Move Southeast to Park
//            strafe(1, 1500);
//            stopMoving();
//
//            // Turn around (180 degrees)
//            rotate(0.5, 1500);
//            sleep(500);
//
//            telemetry.addData("Status", "Autonomous Routine Complete");
//            telemetry.update();
//        }
//    }
//
//    private void moveForward(double power, int time) {
//        robotHardware.mecnum.setDrivePowerAll(power, power, power, power);
//        sleep(time);
//    }
//
//    private void stopMoving() {
//        robotHardware.mecnum.setDrivePowerAll(0, 0, 0, 0);
//    }
//
//    private void strafe(double power, int time) {
//        robotHardware.mecnum.setDrivePowerAll(power, -power, -power, power);
//        sleep(time);
//    }
//
//    private void rotate(double power, int time) {
//        robotHardware.mecnum.setDrivePowerAll(power, -power, power, -power);
//        sleep(time);
//        stopMoving();
//    }
//}
