

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoTest4", group="Psionics")
@Disabled
public class AutoTest4 extends LinearOpMode {

    private double ODsensorLD, ODsensorRLD, dist;
    Psionics_Rover_Hardware       robot   = new Psionics_Rover_Hardware();



    public double returnDistance(double sensorValue) {
        DbgLog.msg(String.format("Got Distance"));
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

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Resetting Encoders"));

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

       // DbgLog.msg(String.format("Before WaitForStart"));
        waitForStart();
       // DbgLog.msg(String.format("Executed WaitForStart"));

       // DbgLog.msg(String.format("VaishnaviStart"));
        encoderDrive(DRIVE_SPEED, -12, -12, 5.0);  // S1: Forward 6 Inches with 5 Sec timeout
       // DbgLog.msg(String.format("Executed m12_m12"));
        encoderDrive(TURN_SPEED, 12, -12, 5.0);  // S2: Turn Right 12 Inches with 5 Sec timeout
       // DbgLog.msg(String.format("Executed 12_m12"));
        encoderDrive(DRIVE_SPEED, -36, -36, 10.0); //Drive forward for 3 ft
      //  DbgLog.msg(String.format("Executed m36_m36"));
        encoderDrive(TURN_SPEED, -12, 12, 5.0); //Turn left 12 Inches
      //  DbgLog.msg(String.format("Executed m12_12"));
        //keep a log message?
       // encoderDrive(DRIVE_SPEED, 0 , 0, 2.0); //Stop
     ////   DbgLog.msg(String.format("Executed z_z"));
     //   DbgLog.msg(String.format("Waiting for 2 sec when reached the wall"));
        //sleep(2000);
      //  DbgLog.msg(String.format("Waited for 2 sec when reached the wall "));
        //keep a log message?
     //   DbgLog.msg(String.format("Before FollowWall"));
        followWall(ODsensorRLD);
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            int currentLeftPosition = robot.leftMotor.getCurrentPosition();
            int currentRightPosition = robot.rightMotor.getCurrentPosition();
            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);

            //Send telemetry message to indicate starting position
            telemetry.addData("Psionics_Rover_Path", "Starting at %7d :%7d", currentLeftPosition, currentRightPosition);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Starting at %7d :%7d", currentLeftPosition, currentRightPosition));

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // Send telemetry message to indicate new target location
            telemetry.addData("Psionics_Rover_Path", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.update();
            DbgLog.msg(String.format("Psionics_Rover : Debug - Running to %7d :%7d", newLeftTarget, newRightTarget));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {


                currentLeftPosition = robot.leftMotor.getCurrentPosition();
                currentRightPosition = robot.rightMotor.getCurrentPosition();

                if (runtime.milliseconds() - TimeLastDbgMsglogged >= 100.0) {
                    DbgMsglogged = false;
                    TimeLastDbgMsglogged += 100.0;
                }
                if ((lastCurrentLeftPosition != currentLeftPosition) || (lastCurrentRightPosition != currentRightPosition)) {
                    DbgMsglogged = false;
                    lastCurrentLeftPosition = currentLeftPosition;
                    lastCurrentRightPosition = currentRightPosition;
                }

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void followWall(double sensorValue) throws InterruptedException {

        sensorValue = ODsensorRLD;
        DbgLog.msg(String.format("Got a sensor value in FOLLOW_WALL BEGINNING %5.7f", sensorValue));
        DbgLog.msg(String.format("Calculating Distance"));
        dist = returnDistance(sensorValue);
        DbgLog.msg(String.format("Calculated Distance"));
        DbgLog.msg(String.format("distance calculated is: %5.2f", dist ));

        while(1==1)
        {
            // if (opModeIsActive()) {
            dist = returnDistance(sensorValue);
            DbgLog.msg(String.format("distance reading in the WHILE loop: %5.2f", dist ));
            if (dist < 2.0) {
                DbgLog.msg(String.format("Dist_lt2"));
                //encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                encoderDrive(DRIVE_SPEED, 1, -1, 5.0);
                DbgLog.msg(String.format("Executed 1_m1 inside DIST_LT2"));
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                DbgLog.msg(String.format("Executed m3_m3 inside DIST_LT2"));
                encoderDrive(DRIVE_SPEED, -1, 1, 5.0);
                DbgLog.msg(String.format("Executed m1_1 inside DIST_LT2"));
                DbgLog.msg(String.format("Stopping in DIST_LT2"));
                encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                DbgLog.msg(String.format("Stopped in DIST_LT2"));
                DbgLog.msg(String.format("Wait for 2 sec in DIST_LT2"));
                sleep(2000);
                DbgLog.msg(String.format("Waited for 2 sec in DIST_LT2"));
                DbgLog.msg(String.format("Getting a sensor value in DIST_LT2"));
                sensorValue = ODsensorRLD;
                DbgLog.msg(String.format("Got a sensor value in DIST_LT2 %5.7f", sensorValue));
                dist = returnDistance(sensorValue);
                DbgLog.msg(String.format("New Dist in DIST_LT2 %5.2f", dist));
                DbgLog.msg(String.format("Waiting for 2 sec in Dist_lt2 "));
                sleep(2000);
                DbgLog.msg(String.format("Waited for 2 sec in Dist_lt2 "));
                DbgLog.msg(String.format("AFTER wait -- Dist_lt2 executed"));
            }
            else if (dist >= 2.0 && dist < 6.0) {
                DbgLog.msg(String.format("Inside Dist_BT26"));
                encoderDrive(DRIVE_SPEED, -3, -3, 5.0);
                DbgLog.msg(String.format("Executed m3_m3 inside DIST_BT26"));
                DbgLog.msg(String.format("Stopping in DIST_BT26"));
                encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                DbgLog.msg(String.format("Stopped in DIST_BT26"));
                DbgLog.msg(String.format("Getting a sensor value in DIST_BT26"));
                sensorValue = ODsensorRLD;
                DbgLog.msg(String.format("Got a sensor value in DIST_BT26 %5.7f", sensorValue));
                dist = returnDistance(sensorValue);
                DbgLog.msg(String.format("Got a dist value in DIST_BT26 %5.2f", dist));
                DbgLog.msg(String.format("BEFORE Dist_b2_6 wait 2s"));
                sleep(2000);
                DbgLog.msg(String.format("AFTER wait 2s -- Dist_b2_6 executed"));
            }
            else if (dist >= 6.0) {
                DbgLog.msg(String.format("Inside Dist_gt6"));
                //encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                encoderDrive(DRIVE_SPEED, 2, -2, 5.0);
                DbgLog.msg(String.format("Executed 2_m2 inside DIST_GT6"));
                encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
                DbgLog.msg(String.format("Executed m5_m5 inside DIST_GT6"));
                encoderDrive(DRIVE_SPEED, -1.75, 1.75, 5.0);
                DbgLog.msg(String.format("Executed m1p75_1p75 inside DIST_GT6"));
                DbgLog.msg(String.format("Stopping in DIST_GT6"));
                encoderDrive(DRIVE_SPEED, 0, 0, 5.0);
                DbgLog.msg(String.format("Stopped in DIST_GT6"));
                sensorValue = ODsensorRLD;
                DbgLog.msg(String.format("Got a sensor value in DIST_GT6 %5.7f", sensorValue));
                dist = returnDistance(sensorValue);
                DbgLog.msg(String.format("Got a dist value in DIST_GT6 %5.2f", dist));
                DbgLog.msg(String.format("BEFORE  wait 2sec in gt6"));
                sleep(2000);
                DbgLog.msg(String.format("AFTER wait 2 sec -- Dist_gt6 executed"));
            }
            DbgLog.msg(String.format("BEFORE IDLE in FOLLOW_WALL WHILE"));
            idle();
            DbgLog.msg(String.format("AFTER IDLE in FOLLOW_WALL WHILE"));
        }

        //sleep(2000);   // optional pause after each move
    }


}