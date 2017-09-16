import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Psionics_Robot_Hardware_Regionals
{
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor flickerMotor  = null;
    public DcMotor  motor7   = null;
    public DcMotor  motor8  = null;




    public Servo buttonPusher = null;
    public Servo frontRightActuator = null;
    public Servo frontLeftActuator = null;
    public Servo backRightActuator = null;
    public Servo backLeftActuator = null;
    /*Servo servo6 = null;
    Servo servo7 = null;
    Servo servo8 = null;
    Servo servo9 = null;
    Servo servo10 = null;
    Servo servo11 = null;
    Servo servo12 = null;*/

    public OpticalDistanceSensor bottomRight = null;
    public OpticalDistanceSensor bottomLeft = null;
    //public OpticalDistanceSensor side_frontLeft = null;
    //public OpticalDistanceSensor side_backLeft = null;
    //public OpticalDistanceSensor side_frontRight = null;
    //public OpticalDistanceSensor side_backRight = null;

    public final static double BUTTONPUSHER_MIN_RANGE  = 1.00;
    public final static double BUTTONPUSHER_MAX_RANGE  = 0.37;
    public final static double BUTTONPUSHER_RATE_SLOW = 0.001;
    public final static double BUTTONPUSHHER_RATE_MEDIUM = 5*BUTTONPUSHER_RATE_SLOW;
    public final static double BUTTONPUSHER_RATE_FAST = 10*BUTTONPUSHHER_RATE_MEDIUM; //orig. 5
    public final static double BUTTONPUSHER_HOME = BUTTONPUSHER_MIN_RANGE*0.63;

    public final static double BACKLEFTACTUATOR_HOME = 0.2;
    public final static double BACKRIGHTACTUATOR_HOME = 0.2;
    public final static double FRONTLEFTACTUATOR_HOME = 0.15;
    public final static double FRONTRIGHTACTUATOR_HOME = 0.25;
    public final static double ACTUATORS_BUTTON_PUSH_DISTANCE = 0.59;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Psionics_Robot_Hardware_Regionals() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("front left motor");
        rightFrontMotor = hwMap.dcMotor.get("front right motor");
        leftBackMotor = hwMap.dcMotor.get("back left motor");
        rightBackMotor = hwMap.dcMotor.get("back right motor");
        intakeMotor = hwMap.dcMotor.get("intake motor");
        flickerMotor = hwMap.dcMotor.get("flicker motor");
        //motor7 = hwMap.dcMotor.get("motor 7");
        //motor8 = hwMap.dcMotor.get("motor 8");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        flickerMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        intakeMotor.setPower(0);
        flickerMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.


        buttonPusher = hwMap.servo.get("button pusher servo");
        backLeftActuator = hwMap.servo.get("back left actuator");
        backRightActuator = hwMap.servo.get("back right actuator");
        frontLeftActuator = hwMap.servo.get("front left actuator");
        frontRightActuator = hwMap.servo.get("front right actuator");
        //servo6 = hwMap.servo.get("servo 6");
        //servo7 = hwMap.servo.get("servo 7");
        //servo8 = hwMap.servo.get("servo 8");
        //servo9 = hwMap.servo.get("servo 9");
        //servo10 = hwMap.servo.get("servo 10");
        //servo11 = hwMap.servo.get("servo 11");
        //servo12 = hwMap.servo.get("servo 12");

        backLeftActuator.setPosition(BACKLEFTACTUATOR_HOME);
        backRightActuator.setPosition(BACKRIGHTACTUATOR_HOME);
        frontLeftActuator.setPosition(FRONTLEFTACTUATOR_HOME);
        frontRightActuator.setPosition(FRONTRIGHTACTUATOR_HOME);
        buttonPusher.setPosition(BUTTONPUSHER_HOME);

        bottomRight = hwMap.opticalDistanceSensor.get("ods_bottom_right");
        bottomLeft = hwMap.opticalDistanceSensor.get("ods_bottom_left");
        //side_frontLeft = hwMap.opticalDistanceSensor.get("frontLeft");
        //side_backLeft = hwMap.opticalDistanceSensor.get("backLeft");
        //side_frontRight = hwMap.opticalDistanceSensor.get("frontRight");
        //side_backRight = hwMap.opticalDistanceSensor.get("side back right");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
