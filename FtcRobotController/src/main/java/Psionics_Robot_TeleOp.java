/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Psionics_Robot_TeleOp", group="Psionics")  // @Autonomous(...) is the other common choice
//@Disabled
public class Psionics_Robot_TeleOp extends LinearOpMode {
    /* Declare OpMode members. */
    Psionics_Robot_Hardware robot = new Psionics_Robot_Hardware();

    private ElapsedTime runtime = new ElapsedTime();

    private double buttonPusherServoPosition = 0.0;
    private double buttonPusherServoRate = Psionics_Robot_Hardware.BUTTONPUSHER_RATE;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if(gamepad2.y || gamepad2.dpad_up) {
                buttonPusherServoPosition = robot.buttonPusher.getPosition();
                buttonPusherServoPosition -= buttonPusherServoRate;
                if(buttonPusherServoPosition <= Psionics_Robot_Hardware.BUTTONPUSHER_MAX_RANGE)
                    buttonPusherServoPosition = Psionics_Robot_Hardware.BUTTONPUSHER_MAX_RANGE;
                robot.buttonPusher.setPosition(buttonPusherServoPosition);
            }
            else if(gamepad2.a || gamepad2.dpad_down) {
                buttonPusherServoPosition = robot.buttonPusher.getPosition();
                buttonPusherServoPosition += buttonPusherServoRate;
                if(buttonPusherServoPosition >= Psionics_Robot_Hardware.BUTTONPUSHER_MIN_RANGE)
                    buttonPusherServoPosition = Psionics_Robot_Hardware.BUTTONPUSHER_MIN_RANGE;
                robot.buttonPusher.setPosition(buttonPusherServoPosition);
            }

                robot.leftFrontMotor.setPower(-gamepad1.left_stick_y);
                robot.leftBackMotor.setPower(-gamepad1.left_stick_y);
                robot.rightFrontMotor.setPower(-gamepad1.right_stick_y);
                robot.rightBackMotor.setPower(-gamepad1.right_stick_y);


            if(gamepad2.left_trigger != 0.0) {
                if (gamepad2.left_bumper) {
                    robot.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                    robot.intakeMotor.setPower(gamepad2.left_trigger);
                } else {
                    robot.intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                    robot.intakeMotor.setPower(gamepad2.left_trigger);
                }
            }
            else {
                    robot.intakeMotor.setPower(0);
            }

            if(gamepad2.right_trigger != 0.0) {
                if(gamepad2.right_bumper) {
                    robot.flickerMotor.setDirection(DcMotor.Direction.REVERSE);
                    robot.flickerMotor.setPower(gamepad2.right_trigger);
                }
                else {
                    robot.flickerMotor.setDirection(DcMotor.Direction.FORWARD);
                    robot.flickerMotor.setPower(1);
                }
            }
            else {
                robot.flickerMotor.setPower(0);
            }

            telemetry.addData("Left Front Encoder: ", String.format("%7d", robot.leftFrontMotor.getCurrentPosition()));
            telemetry.addData("Right Front Encoder: ", String.format("%7d", robot.rightFrontMotor.getCurrentPosition()));
            telemetry.addData("Intake Encoder: ", String.format("%7d", robot.intakeMotor.getCurrentPosition()));
            telemetry.addData("Flicker Encoder: ", String.format("%7d", robot.flickerMotor.getCurrentPosition()));

            telemetry.addData("Left Front Motor Power: ", String.format("%.2f", robot.leftFrontMotor.getPower()));
            telemetry.addData("Right Front Motor Power: ", String.format("%.2f", robot.rightFrontMotor.getPower()));
            telemetry.addData("Intake Motor Power: ", String.format("%.2f", robot.intakeMotor.getPower()));
            telemetry.addData("Flicker Motor Power: ", String.format("%.2f", robot.flickerMotor.getPower()));
            //telemetry.addData("Y Button: ", gamepad1.y);
            //telemetry.addData("A Button: ", gamepad1.a);
            telemetry.addData("Servo position: ", String.format("%.2f", robot.buttonPusher.getPosition()));
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}

