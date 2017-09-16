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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Psionics hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PsionicsAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="LineFollowerBR", group="Psionics")
//@Disabled
public class LineFollowerBR  extends LinearOpMode {

    /* Declare OpMode members. */
    Psionics_Robot_Hardware2 robot = new Psionics_Robot_Hardware2();

    double SVL1;
    double SVL2;
    double SVR1;
    double SVR2;

    OpticalDistanceSensor odsBL1;
    OpticalDistanceSensor odsBL2;
    OpticalDistanceSensor odsBR1;
    OpticalDistanceSensor odsBR2;

    // Use a PsionicBot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double DISTANCE_EROR_COMPENSATION = 60 / 55.5;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (DISTANCE_EROR_COMPENSATION * WHEEL_DIAMETER_INCHES * 3.1415);
    static final double RIGHT_SPEED_COMPENSATION = 2.1;
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    public double getColorLF(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= 1.5) {
            color = 1.0;
        }
        return color;
    }

    public double getColorRF(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= 1.2) {
            color = 1.0;
        }
        return color;
    }

    public double getColorLB(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= .2) {
            color = 1.0;
        }
        return color;
    }

    public double getColorRB(double sensorValue) {
        double color = 0.0;
        if (sensorValue >= .1) {
            color = 1.0;
        }
        return color;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        odsBL1 = hardwareMap.opticalDistanceSensor.get("odsBL1");
        odsBL2 = hardwareMap.opticalDistanceSensor.get("odsBL2");
        odsBR1 = hardwareMap.opticalDistanceSensor.get("odsBR1");
        odsBR2 = hardwareMap.opticalDistanceSensor.get("odsBR2");
        SVL1 = odsBL1.getRawLightDetected();
        SVL2 = odsBL2.getRawLightDetected();
        SVR1 = odsBR1.getRawLightDetected();
        SVR2 = odsBR2.getRawLightDetected();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Bot : Debug - Resetting Encoders"));

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        waitForStart();

    SVL1 = odsBL1.getRawLightDetected();
    SVL2 = odsBL2.getRawLightDetected();
    SVR1 = odsBR1.getRawLightDetected();
    SVR2 = odsBR2.getRawLightDetected();

    telemetry.addData("L1", SVL1);
    telemetry.addData("L2", SVL2);
    telemetry.addData("R1", SVR1);
    telemetry.addData("R2", SVR2);
    telemetry.update();
     //    lineFollowF(SVL2, SVR2);
     //   sleep(5000);
        lineFollowB(SVL1, SVR1);
    // S1: Forward 60 Inches with 60 Sec timeout
    //encoderDrive(TURN_SPEED,   12, -12, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout
    //encoderDrive(DRIVE_SPEED, 6, 6, 5.0);       // Forward 6 Inches with 5 sec timeout
    //encoderDrive(TURN_SPEED, -12, 12, 5.0); // Turn Left 12 Inches with 5 sec timeout
    //encoderDrive(DRIVE_SPEED, 6, 6, 5.0); // Forward 6 Inches with 5 sec timeout
    //encoderDrive(TURN_SPEED, 12, -12, 5.0); // Turn Right 12 Inches with 5 sec timeout
    //encoderDrive(DRIVE_SPEED, 6, 6, 5.0); // Forward 6 Inches with 5 sec timeout
    //encoderDrive(TURN_SPEED, -12, 12, 5.0); // Turn Left 12 Inches with 5 sec timeout
    //encoderDrive(DRIVE_SPEED, 6 , 6, 5.0); // Forward 6 Inches with 5 sec timeout
    // encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

    //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
    //robot.rightClaw.setPosition(0.0);
    //sleep(1000);     // pause for servos to move


    //Send telemetry message to indicate path complete
  // telemetry.addData("Psionics_Bot_Path", "Complete");
        ///telemetry.update();
        //DbgLog.msg(String.format("Psionics_Bot : Debug - Complete"));
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches,
                             double timeoutS) throws InterruptedException {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            int currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            int currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            int lastCurrentLeftFrontPosition = currentLeftFrontPosition;
            int lastCurrentRightFrontPosition = currentRightFrontPosition;
            int lastCurrentLeftBackPosition = currentLeftBackPosition;
            int lastCurrentRightBackPosition = currentRightBackPosition;
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = currentLeftFrontPosition + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = currentRightFrontPosition + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = currentLeftBackPosition + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = currentRightBackPosition + (int) (rightBackInches * COUNTS_PER_INCH);


            //Send telemetry message to indicate starting position
            telemetry.addData("Psionics_Bot_Path", "Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Bot : Debug - Starting at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed) * RIGHT_SPEED_COMPENSATION);

            // Send telemetry message to indicate new target location
            telemetry.addData("Psionics_Bot_Path", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Bot : Debug - Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
                currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
                currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
                currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();


                if (runtime.milliseconds() - TimeLastDbgMsglogged >= 100.0) {
                    DbgMsglogged = false;
                    TimeLastDbgMsglogged += 100.0;
                }
                if ((lastCurrentLeftFrontPosition != currentLeftFrontPosition) || (lastCurrentRightFrontPosition != currentRightFrontPosition) || (lastCurrentLeftBackPosition != currentLeftBackPosition) || (lastCurrentRightBackPosition != currentRightBackPosition)) {
                    DbgMsglogged = false;
                    lastCurrentLeftFrontPosition = currentLeftFrontPosition;
                    lastCurrentRightFrontPosition = currentRightFrontPosition;
                    lastCurrentLeftBackPosition = currentLeftBackPosition;
                    lastCurrentRightBackPosition = currentRightBackPosition;
                }

                if (DbgMsglogged == false) {
                    // Display it for the driver.
                    telemetry.addData("Psionics_Bot_Path", "Running at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition);
                    telemetry.update();
                    DbgLog.msg(String.format("Psionics_Bot : Debug - Currently at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));
                    DbgMsglogged = true;
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            currentLeftFrontPosition = robot.leftFrontMotor.getCurrentPosition();
            currentRightFrontPosition = robot.rightFrontMotor.getCurrentPosition();
            currentLeftBackPosition = robot.leftBackMotor.getCurrentPosition();
            currentRightBackPosition = robot.rightBackMotor.getCurrentPosition();

            DbgLog.msg(String.format("Psionics_Bot : Debug - Finally at %7d :%7d :%7d :%7d", currentLeftFrontPosition, currentRightFrontPosition, currentLeftBackPosition, currentRightBackPosition));

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void lineFollowB(double sensorValueL, double sensorValueR) throws InterruptedException {
        while (1 == 1) {

            sensorValueL = odsBL1.getRawLightDetected();
            sensorValueR = odsBR1.getRawLightDetected();

            double colorL = getColorLB(sensorValueL); //right way to call this?
            double colorR = getColorRB(sensorValueR);
            telemetry.addData("Color L:", colorL);
            telemetry.addData("Color R:", colorR);
            telemetry.update();

            if (colorL == 0.0 && colorR == 1.0)
            {
                encoderDrive(DRIVE_SPEED, 1, 1, 1, 1, 3.0);
                sleep(1000);
            }
            else if (colorL == 1.0 && colorR == 0.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (colorL == 1.0 && colorR == 1.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (colorL == 0.0 && colorR == 0.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
        }
    }


    public void lineFollowF(double sensorValueL, double sensorValueR) throws InterruptedException {
        while (1 == 1) {

            sensorValueL = odsBL2.getRawLightDetected();
            sensorValueR = odsBR2.getRawLightDetected();

            double colorL = getColorLF(sensorValueL); //right way to call this?
            double colorR = getColorRF(sensorValueR);
            telemetry.addData("Color L:", colorL);
            telemetry.addData("Color R:", colorR);
            telemetry.update();

            if (colorL == 0.0 && colorR == 1.0)
            {
                encoderDrive(DRIVE_SPEED, 1, 1, 1, 1, 3.0);
                sleep(1000);
            }
            else if (colorL == 1.0 && colorR == 0.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (colorL == 1.0 && colorR == 1.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
            else if (colorL == 0.0 && colorR == 0.0)
            {
                encoderDrive(DRIVE_SPEED, -3, 3, -3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3, 3.0);
                encoderDrive(DRIVE_SPEED, 3, -3, 3, -3, 3.0);
                sleep(1000);
            }
        }
    }
}
