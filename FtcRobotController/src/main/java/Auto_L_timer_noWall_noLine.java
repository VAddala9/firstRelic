import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Auto_L_timer_noWall_noLine", group="Psionics")
public class Auto_L_timer_noWall_noLine extends LinearOpMode {

    Psionics_Robot_Hardware robot = new Psionics_Robot_Hardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        /*
        go forward for x seconds
        stop
        shoot pre-loaded ball(s)
        turn hard left
        go forward y seconds
        turn hard right
        go forward z seconds
        turn hard left
        go forward a seconds
        push button
        go back a seconds
        turn hard right
        go forward w seconds
        turn hard left
        go forward a seconds
        push button
        go back a seconds
        turn hard right
        turn 135 degrees
        go forward b seconds
        stop

         */

    }
}
