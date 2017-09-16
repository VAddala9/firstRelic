import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="EncoderWheelTest", group="Psionics")
//@Disabled
public class EncoderWheelTest extends LinearOpMode {

    Psionics_Robot_Hardware       robot   = new Psionics_Robot_Hardware();

    // OpticalDistanceSensor odsSensor;
    // OpticalDistanceSensor odsSensor2;


    private ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double back_left_motor_pwr = 0.125;
        double back_right_motor_pwr = 0.125;
        double front_left_motor_pwr = 0.125;
        double front_right_motor_pwr = 0.125;
        double time_out = 1.0;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: Psionics_Rover : Debug - Resetting Encoders"));

        idle();
        waitForStart();

        encoderDrive(DRIVE_SPEED, 18,  18, 1.0);  // S1: Forward 6 Inches with 5 Sec timeout

        telemetry.addData("forward 9(1st)", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("V1DBG: forward 9(1st) - Complete"));

        sleep(1000);
        idle();

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException
    {
        telemetry.addData("Psionics_Rover_Path", "Complete");
        telemetry.update();
        DbgLog.msg(String.format("in encoderDrive"));
        int newLeftTarget;
        int newRightTarget;

        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            DbgLog.msg(String.format("V1DBG: opModeIsActive -- encoderDrive"));

            int currentLeftPosition = robot.leftFrontMotor.getCurrentPosition();
            int currentRightPosition = robot.rightFrontMotor.getCurrentPosition();


           /* DbgLog.msg(String.format("V1DBG: current postions left and right (front first then back): %10d : %10d : %10d : %10d :",
                    robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition(),
                    robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition()));*/

            telemetry.addData("Psionics_Rover_Path", robot.rightFrontMotor.getCurrentPosition());
            telemetry.update();
            idle();
            // DbgLog.msg(String.format("V1DBG: current postion back right %10d :", robot.rightFrontMotor.getCurrentPosition()));
            idle();

            int lastCurrentLeftPosition = currentLeftPosition;
            int lastCurrentRightPosition = currentRightPosition;

            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            idle();
            //  DbgLog.msg(String.format("V1DBG: current postions left and right (front first then back) after assigning: %10d : %10d : %10d : %10d",
            //      currentLeftPosition, currentRightPosition));
            // Determine new target position, and pass to motor controller
            newLeftTarget = currentLeftPosition + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = currentRightPosition + (int) (rightInches * COUNTS_PER_INCH);

            telemetry.addData("Psionics_Rover_Path", "Complete");
            telemetry.update();
            idle();
            // DbgLog.msg(String.format("V1DBG: new left and right targets: %10d: %10d", newLeftTarget, newRightTarget));

            //  robot.leftBackMotor.setTargetPosition(newLeftTarget);
            //  robot.rightBackMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            //   robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //  robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //initializing debug logging variables
            boolean DbgMsglogged = true;
            double TimeLastDbgMsglogged = 0.0;

            //set robot's speed
            robot.leftBackMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);

            telemetry.addData("V1DBG: after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: speeds of motors assigned"));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                idle();

            }

            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            telemetry.addData("after run to position", "Complete");
            telemetry.update();
            idle();
            DbgLog.msg(String.format("V1DBG: stopped motors"));
            idle();
            // Turn off RUN_TO_POSITION
        }
    }

}






