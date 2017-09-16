
//FOR POTENIAL POTENTIOMETER: AnalogInput potentiometer = hardwareMap.analogInput.get("<WhateverNameYouGaveIt>");
//int value = potentiometer.getValue();
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BigRobotWallFollower", group="Psionics")
//@Disabled
public class BigRobotWallFollower extends LinearOpMode {

    private double ODsensorRF, ODsensorRB, dist, ODsensorLF, ODsensorLB, dist2;
    Psionics_Robot_Hardware       robot   = new Psionics_Robot_Hardware();

    OpticalDistanceSensor odsRF;
    OpticalDistanceSensor odsRB;
    OpticalDistanceSensor odsLF;
    OpticalDistanceSensor odsLB;



    // OpticalDistanceSensor odsSensor;
    // OpticalDistanceSensor odsSensor2;

    public double returnDistance(double sensorValue) {

        if(sensorValue >= .38)
        {
            dist = 1.0;
        }
        else if(sensorValue < .38 && sensorValue >=.132)
        {
            dist = 2.0;
        }

        else if(sensorValue < .132 && sensorValue >=.058)
        {
            dist = 3.0;
        }
        else if(sensorValue < .058 && sensorValue >=.039)
        {
            dist = 4.0;
        }
        else if(sensorValue < .039 && sensorValue >=.029)
        {
            dist = 5.0;
        }
        else if(sensorValue < .029 && sensorValue >=.024)
        {
            dist = 6.0;
        }
        else if(sensorValue < .024 && sensorValue >=.02)
        {
            dist = 8.0;
        }
        else
        {
            dist = 9.0;
        }

        return dist;
    }

    public double returnDistance2(double sensorValue2) {

        if(sensorValue2 >= .28)
        {
            dist2 = 1.0;
        }
        else if(sensorValue2 < .28 && sensorValue2 >=.11)
        {
            dist2 = 2.0;
        }

        else if(sensorValue2 < .11 && sensorValue2 >=.05865)
        {
            dist2 = 3.0;
        }
        else if(sensorValue2 < .05865 && sensorValue2 >=.048)
        {
            dist2 = 4.0;
        }
        else if(sensorValue2 < .048 && sensorValue2 >=.039)
        {
            dist2 = 5.0;
        }
        else if(sensorValue2 < .039 && sensorValue2 >=.029)
        {
            dist2 = 6.0;
        }
        else if(sensorValue2 < .029 && sensorValue2 >=.0195)
        {
            dist2 = 8.0;
        }
        else
        {
            dist2 = 9.0;
        }

        return dist2;
    }

    /* Declare OpMode members. */
    // Use a PsionicBot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        odsRF = hardwareMap.opticalDistanceSensor.get("odsRF");
        odsRB = hardwareMap.opticalDistanceSensor.get("odsRB");
        odsLF = hardwareMap.opticalDistanceSensor.get("odsLF");
        odsLB = hardwareMap.opticalDistanceSensor.get("odsLB");
        ODsensorRF = odsRF.getRawLightDetected();
        ODsensorRB = odsRB.getRawLightDetected();
        ODsensorLF = odsLF.getRawLightDetected();
        ODsensorLB = odsLB.getRawLightDetected();


        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Resetting Encoders"));

        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        waitForStart();

        encoderDrive(DRIVE_SPEED, -12,  -12, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED, -36,  -36, 10.0);
        encoderDrive(TURN_SPEED,   -12, 12, 5.0);
        followWall(ODsensorRF, ODsensorRB);

        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Complete"));
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            int currentLeftPosition = robot.leftBackMotor.getCurrentPosition();
            int currentRightPosition = robot.rightBackMotor.getCurrentPosition();
            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);

            //Send telemetry message to indicate starting position
            telemetry.addData("Psionics_Rover_Path", "Starting at %7d :%7d", currentLeftPosition, currentRightPosition);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Starting at %7d :%7d", currentLeftPosition, currentRightPosition));


            // send the info back to driver station using telemetry function.
            telemetry.addData("Raw", odsRF.getRawLightDetected());
            telemetry.addData("Normal", odsRB.getRawLightDetected());

            robot.leftBackMotor.setTargetPosition(newLeftTarget);
            robot.rightBackMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // Send telemetry message to indicate new target location
            telemetry.addData("Psionics_Rover_Path", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Running to %7d :%7d", newLeftTarget, newRightTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {


                currentLeftPosition = robot.leftBackMotor.getCurrentPosition();
                currentRightPosition = robot.rightBackMotor.getCurrentPosition();

                if (runtime.milliseconds() - TimeLastDbgMsglogged >= 100.0) {
                    DbgMsglogged = false;
                    TimeLastDbgMsglogged += 100.0;
                }
                if ((lastCurrentLeftPosition != currentLeftPosition) || (lastCurrentRightPosition != currentRightPosition)) {
                    DbgMsglogged = false;
                    lastCurrentLeftPosition = currentLeftPosition;
                    lastCurrentRightPosition = currentRightPosition;
                }

                if (DbgMsglogged == false) {
                    // Display it for the driver.
                    telemetry.addData("Psionics_Rover_Path", "Running at %7d :%7d", currentLeftPosition, currentRightPosition);
                    telemetry.update();
                    DbgLog.msg(String.format("Psionics_Rover : Debug - Currently at %7d :%7d", currentLeftPosition, currentRightPosition));
                    DbgMsglogged = true;
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void followWall(double sensorValue, double sensorValue2) throws InterruptedException {
        ODsensorRF = odsRF.getRawLightDetected();
        ODsensorRB = odsRB.getRawLightDetected();
        sensorValue = ODsensorRF;
        sensorValue2 = ODsensorRB;
        dist = returnDistance(sensorValue);
        dist2 = returnDistance2(sensorValue2);

        while(1==1)
        {
            telemetry.addData("Psionics_Rover: in while loop", + dist + " " + dist2);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover: in while loop"));
            // if (opModeIsActive()) {
            ODsensorRF = odsRF.getRawLightDetected();
            ODsensorRB = odsRB.getRawLightDetected();
            sensorValue = ODsensorRF;
            sensorValue2 = ODsensorRB;
            telemetry.addData("Sensorvalue", + sensorValue);
            telemetry.update();
            DbgLog.msg(String.format("SensorValuesInBeg: %5.7f: %5.7f", sensorValue, sensorValue2));
            dist = returnDistance(sensorValue);
            dist2 = returnDistance2(sensorValue2);
            telemetry.addData("dists", + dist + dist2);
            telemetry.update();
            DbgLog.msg(String.format("DistanceInBeg: %5.7f: %5.7f", dist, dist2));
            if (dist < 4.0) {
                encoderDrive(DRIVE_SPEED, -2, 2, 5.0);
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
                encoderDrive(DRIVE_SPEED, 1.75, -1.75, 5.0);
                // sleep(500);
                sensorValue = ODsensorRF;
                sensorValue2 = ODsensorRB;
                telemetry.addData("Sensorvalue", + sensorValue);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD<4: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD<4: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 4.0 && dist < 6.0) {
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                //sleep(500);
                sensorValue = ODsensorRF;
                sensorValue2 = ODsensorRB;
                telemetry.addData("Sensorvalues", + sensorValue + sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValue4<=D<6: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist + dist2);
                telemetry.update();
                DbgLog.msg(String.format("Distance4<=D<6: %5.7f: %5.7f", dist, dist2));

            } else if (dist >= 6.0 || dist2 > dist) {
                //encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
                encoderDrive(DRIVE_SPEED, -1.75, 1.75, 5.0);
                //sleep(500);
                sensorValue = ODsensorRF;
                sensorValue2 = ODsensorRB;
                telemetry.addData("Sensorvalues", + sensorValue +sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueD>=6: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist +dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceD>=6: %5.7f: %5.7f", dist, dist2));
            }
            else if (dist > (dist2+1))
            {
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                sensorValue = ODsensorRF;
                sensorValue2 = ODsensorRB;
                telemetry.addData("Sensorvalues", + sensorValue +sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueDist>Dist2: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist +dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceDist>Dist2: %5.7f: %5.7f", dist, dist2));
            }

            else if (dist2 > (dist+1))
            {
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                sensorValue = ODsensorRF;
                sensorValue2 = ODsensorRB;
                telemetry.addData("Sensorvalues", + sensorValue +sensorValue2);
                telemetry.update();
                DbgLog.msg(String.format("SensorValueDist2>Dist: %5.7f: %5.7f", sensorValue, sensorValue2));
                dist = returnDistance(sensorValue);
                dist2 = returnDistance2(sensorValue2);
                telemetry.addData("dists", + dist +dist2);
                telemetry.update();
                DbgLog.msg(String.format("DistanceDist2>Dist: %5.7f: %5.7f", dist, dist2));
            }
            idle();
        }
    }
}



