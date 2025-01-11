
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="The Only TeleOP", group="Linear OpMode")
public class TeleOP extends LinearOpMode {
    private boolean b1state = false;
    private boolean b2state = false;
    private boolean y1state = false;
    private boolean a2state = false;
    private boolean a3state = false;
    private boolean bdpadUpState = false;


    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        ElapsedTime mStateTime = new ElapsedTime();
        ElapsedTime xStateTime = new ElapsedTime();

        int v_state = 0;

        waitForStart();
        robot.init();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Claw", robot.clawServo.getPosition());
            telemetry.addData("arm right", robot.armRight.getPosition());
            telemetry.addData("arm left", robot.armLeft.getPosition());
            telemetry.addData("up lift position", robot.rLift.getCurrentPosition());
            telemetry.addData("foward lift position", robot.extendo.getCurrentPosition());


            robot.localizer.update();
            telemetry.addData("Front Encoder", robot.frontEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", robot.leftEncoder.getCurrentPosition());
            telemetry.update();

            robot.mecnum.brake(1 - gamepad1.right_trigger);
            robot.mecnum.driveRobot(gamepad1);

            if (gamepad1.a) {
                robot.intake.pitchDown();
                robot.intake.startIntake();
            } else if (gamepad1.x) {
                robot.intake.pitchDown();
                robot.intake.reverseIntake();
            } else if (gamepad1.dpad_up) {
                robot.intake.pitchUp();
                robot.intake.startIntake();
            } else if (gamepad1.dpad_down) {
                robot.intake.pitchUp();
                robot.intake.reverseIntake();
            } else {
                robot.intake.pitchRest();
                robot.intake.stopIntake();
            }


            if (gamepad1.right_bumper) {
                robot.lifts.forwardLift();
            } else if (gamepad1.left_bumper) {
                robot.lifts.backLift();
            } else {
                robot.lifts.stopLiftHorizontal();
            }

            if (gamepad2.right_bumper) {
                robot.lifts.raiseLift();
            } else if (gamepad2.left_bumper) {
                robot.lifts.lowerLift();
            } else {
                robot.lifts.stopLiftVertical();
            }


//            if (gamepad1.b && !b1state) {
//                b1state = true;
//                v_state = 0;
//                mStateTime.reset();
//            }
//            if (b1state) {
//                switch (v_state) {
//                    case 0:
//                        robot.intake.pitchUp();
//                        robot.claw.clawOpen();
//                        robot.claw.wristDown();
//                        if (mStateTime.seconds() >= 0.15) {
//                            mStateTime.reset();
//                            v_state++;
//                        }
//                        break;
//
//                    case 1:
//                        robot.intake.pitchUp();
//                        robot.claw.armDown();
//                        if (mStateTime.seconds() >= 0.25) {
//                            mStateTime.reset();
//                            v_state++;
//                        }
//                        break;
//
//                    case 2:
//                        robot.intake.pitchUp();
//                        robot.claw.clawClose();
//                        if (mStateTime.seconds() >= 0.4) {
//                            mStateTime.reset();
//                            v_state++;
//                        }
//                        break;
//
//                    case 3:
//                        robot.intake.pitchDown();
//                        robot.claw.armRest();
//                        if (mStateTime.seconds() >= 0.3) {
//                            mStateTime.reset();
//                            v_state++;
//                        }
//                        break;
//
//                    case 4:
//                        robot.intake.pitchDown();
//                        robot.claw.wristUP();
//                        robot.claw.armUp();
//                        b1state = false;
//                        break;
//                }
//            }
//
            if (gamepad1.b && !b1state)
            {
                robot.claw.grabParallel();
                b1state = true;
            }
            else if (!gamepad1.b && b1state) {
                b1state = false;
            }
            robot.claw.grabUpdate();



            if (gamepad2.y && !y1state) {
                robot.claw.swing();
                y1state = true;
            } else if (!gamepad2.y && y1state) {
                y1state = false;
            }

            if (gamepad2.b && !b2state) {
                robot.claw.toggleClaw();
                b2state = true;
            } else if (!gamepad2.b && b2state) {
                b2state = false;
            }

            if (gamepad2.x) {
                robot.claw.specimen();
            }

            if (gamepad2.a && !a3state) {
                robot.claw.lvl1hang();
                a3state = true;
            } else if (!gamepad2.a && a3state) {
                a3state = false;
            }

            if (gamepad2.dpad_up && !bdpadUpState) {
                robot.lifts.GoToPositionVertical(3555);
                bdpadUpState = true;
            } else if (!gamepad2.y && bdpadUpState) {
                bdpadUpState = false;
            }


            telemetry.update();
        }
    }
}