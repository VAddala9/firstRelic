import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="LineFollower2", group="Psionics")
//@Disabled
public class LineFollowerTest extends LinearOpMode {

    //private double ODsensorLD, ODsensorRLD, dist, ODsensorLD2, ODsensorRLD2, dist2;
    private double ODSBL1,ODSBR1, ODSBL2, ODSBR2;
    //private String colorBL1, colorBR1, colorBL2, colorBR2;
    private String color, colorBL1, colorBR1, colorBL2, colorBR2;
    Psionics_Rover_Hardware robot   = new Psionics_Rover_Hardware();

    OpticalDistanceSensor odsBL1;
    OpticalDistanceSensor odsBR1;
    OpticalDistanceSensor odsBL2;
    OpticalDistanceSensor odsBR2;

    // OpticalDistanceSensor odsSensor;
    // OpticalDistanceSensor odsSensor2;

    public String returnColor(double sensorValue) {

        if(sensorValue >= .02)
        {
            color = "white";
        }
        else{
            color = "gray";
        }
        return color;
    }
    public void setSensors1(double sensorValueBL1, double sensorValueBR1)
    {
        ODSBL1 = odsBL1.getRawLightDetected();
        ODSBR1 = odsBR1.getRawLightDetected();
        sensorValueBL1 = ODSBL1;
        sensorValueBR1 = ODSBR1;

    }
    public void setSensors2(double sensorValueBL2, double sensorValueBR2)
    {
        ODSBL2 = odsBL2.getRawLightDetected();
        ODSBR2 = odsBR2.getRawLightDetected();
        sensorValueBL2 = ODSBL2;
        sensorValueBL2 = ODSBR2;
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

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        odsBL1 = hardwareMap.opticalDistanceSensor.get("odsBL1");
        odsBR1 = hardwareMap.opticalDistanceSensor.get("odsBR1");
        odsBL2 = hardwareMap.opticalDistanceSensor.get("odsBL2");
        odsBR2 = hardwareMap.opticalDistanceSensor.get("odsBR2");
        ODSBL1 = odsBL1.getRawLightDetected();
        ODSBR1 = odsBR1.getRawLightDetected();
        ODSBL2 = odsBL2.getRawLightDetected();
        ODSBR2 = odsBR2.getRawLightDetected();


        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting; //not real

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("Psionics_Rover : Debug - Resetting Encoders"));

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        waitForStart();

        encoderDrive(DRIVE_SPEED, -9,  -9, 9.0
        );  // S1: Forward 6 Inches with 5 Sec timeout
        followLine1(ODSBL1, ODSBR1);

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
            int currentLeftPosition = robot.leftMotor.getCurrentPosition();
            int currentRightPosition = robot.rightMotor.getCurrentPosition();
            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

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
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(2000);   // optional pause after each move
        }
    }

    public void followLine1(double w, double x) throws InterruptedException {
        setSensors1(w, x);

        while(1==1)
        {
            DbgLog.msg(String.format("Psionics_Rover: in while loop"));
            // if (opModeIsActive()) {
            setSensors1(w, x);
            DbgLog.msg(String.format("SensorValuesInBeg: %5.7f: %5.7f", w, x));
            colorBL1 = returnColor(w);
            colorBR1 = returnColor(x);
            telemetry.addData("color", color);
            telemetry.update();
            DbgLog.msg(String.format("Colors in Beg: %5.7f: %5.7f", colorBL1, colorBR1));
            if (colorBR1.equals("gray") && colorBL1.equals("gray")) {
                encoderDrive(DRIVE_SPEED, 0, 0, 0.0);
                setSensors1(w, x);
                DbgLog.msg(String.format("SensorValuesInGray/Gray: %5.7f: %5.7f", w, x));
                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);
                telemetry.addData("color", color);
                telemetry.update();
                DbgLog.msg(String.format("Colors in Beg: %5.7f: %5.7f", colorBL1, colorBR1));
            }
            else if(colorBR1.equals("white") && colorBL1.equals("gray"))
            {
                encoderDrive(DRIVE_SPEED, -5, -5, 10.0);
                setSensors1(w, x);
                DbgLog.msg(String.format("SensorValuesInGray/Gray: %5.7f: %5.7f", w, x));
                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);
                telemetry.addData("color", color);
                telemetry.update();
                DbgLog.msg(String.format("Colors in Gray/Gray: %5.7f: %5.7f", colorBL1, colorBR1));
            }
            else if(colorBR1.equals("gray") && colorBL1.equals("white"))
            {
                encoderDrive(DRIVE_SPEED, 5, -5, 10.0);
                setSensors1(w, x);
                DbgLog.msg(String.format("SensorValuesInGray/White: %5.7f: %5.7f", w, x));
                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);
                telemetry.addData("color", color);
                telemetry.update();
                DbgLog.msg(String.format("Colors in Gray/White: %5.7f: %5.7f", colorBL1, colorBR1));
            }
            else if(colorBR1.equals("white") && colorBL1.equals("white"))
            {
                encoderDrive(DRIVE_SPEED, 3, -3, 10.0);
                setSensors1(w, x);
                DbgLog.msg(String.format("SensorValuesInWhite/White: %5.7f: %5.7f", w, x));
                colorBL1 = returnColor(w);
                colorBR1 = returnColor(x);
                telemetry.addData("color", color);
                telemetry.update();
                DbgLog.msg(String.format("Colors in White/White: %5.7f: %5.7f", colorBL1, colorBR1));
            }
            sleep(2000);
            idle();
        }

    }
}



