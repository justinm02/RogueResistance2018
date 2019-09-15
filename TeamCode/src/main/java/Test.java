import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {
    private enum TeamColor { RED, BLUE };
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive, liftMotor, intakeLift, intakeSlide;
    private CRServo intakeServo;
    private Servo bail, teamMarker;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity; //might delete
    //THESE VALUES ARE NOT ACTUALLY ACCURATE YET. WAIT FOR WHEEL RATIO AND DIAMETER TO BE DETERMINED
    private final double GEAR_RATIO = 1.00000, WHEEL_DIAMETER = 4, WHEEL_TICKS_PER_REV = 560, LIFT_TICKS_PER_REV = 1120, LIFT_DIAMETER = 0.919, LIFT_GEAR_RATIO = 0.3846;
    private final double C = WHEEL_TICKS_PER_REV/(Math.PI*WHEEL_DIAMETER*GEAR_RATIO), STRAFE_COEFFICIENT = 1.12943302;
    private GoldAlignDetector detector;
    // The IMU sensor object



    public void runOpMode() {
        teamMarker = hardwareMap.servo.get("teamMarker");
        teamMarker.setDirection(Servo.Direction.FORWARD);

        teamMarker.setPosition(1);
        waitForStart();
        teamMarker.setPosition(0);
        pause(1);
        teamMarker.setPosition(1);
    }

    public void pause(double time) {
        double pause = runtime.time();
        while (runtime.time() - pause < time) {
            telemetry.addData("Paused","");
            telemetry.update();
        }
    }
}
