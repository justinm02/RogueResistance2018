package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import java.util.Locale;

/**
 * Created by Justin Milushev on 9/10/2018.
 */

@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {
    private enum TeamColor { RED, BLUE };
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive, liftMotor, intakeLift, intakeSlide;
    private CRServo intakeServo;
    private Servo teamMarker, trapdoor;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity; //might delete
    private DistanceSensor sensorBackLeftRange, sensorBackRightRange; //color distance sensors
    private DistanceSensor sensorRange; //distance sensor
    //THESE VALUES ARE NOT ACTUALLY ACCURATE YET. WAIT FOR WHEEL RATIO AND DIAMETER TO BE DETERMINED
    private final double GEAR_RATIO = 1.00000, WHEEL_DIAMETER = 4, WHEEL_TICKS_PER_REV = 560, LIFT_TICKS_PER_REV = 2240, LIFT_DIAMETER = 2.165, LIFT_GEAR_RATIO = 1;
    private final double C = WHEEL_TICKS_PER_REV/(Math.PI*WHEEL_DIAMETER*GEAR_RATIO), STRAFE_COEFFICIENT = 1.12943302;
    private GoldAlignDetector detector;
    // The IMU sensor object

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();


        //initialize motors and sensors and servos
        leftFrontDrive = hardwareMap.dcMotor.get("leftFrontDrive");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFrontDrive");
        leftRearDrive = hardwareMap.dcMotor.get("leftRearDrive");
        rightRearDrive = hardwareMap.dcMotor.get("rightRearDrive");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        intakeLift = hardwareMap.dcMotor.get("intakeLift");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeSlide = hardwareMap.dcMotor.get("intakeSlide");
        teamMarker = hardwareMap.servo.get("teamMarker");
        trapdoor = hardwareMap.servo.get("trapdoor");
        sensorBackLeftRange = hardwareMap.get(DistanceSensor.class, "sensorBackLeft_range");
        sensorBackRightRange = hardwareMap.get(DistanceSensor.class, "sensorBackRight_range");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLift.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        teamMarker.setDirection(Servo.Direction.FORWARD);
        trapdoor.setDirection(Servo.Direction.FORWARD);
        intakeSlide.setDirection(DcMotor.Direction.FORWARD);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize phone detecting gold mineral alignment
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        liftMotor.setPower(0);
        teamMarker.setPosition(1);
        trapdoor.setPosition(1);

        /*telemetry.addData("Status", "Booted hardware. Waiting for Start");
        telemetry.update();*/

        waitForStart();

        try {
            //descend down from lander +30 pts.
            lift(15, 1);
            pause(.25);

            //initialize gyro
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            pause(.25);

            /*telemetry.addData("func", imu.getSystemStatus());
            telemetry.update();*/

            //determine location of gold mineral and strafe distance required to get to it
            detector.enable();
            pause(1);
            telemetry.addData("x position", detector.getXPosition());
            telemetry.update();
            int extraDistance = 14; //default distance to get to gold mineral
            if (detector.getXPosition() == 0) //right
                extraDistance = 14;
            else if (detector.getXPosition() < 250 && detector.getXPosition() != 0) //left
                extraDistance = -16;
            else if (detector.getXPosition() > 250) //middle
                extraDistance = 0;
            telemetry.update();

            pause(.25);

            //knock over correct mineral +25 pts.
            move(-15, -.5); //move away from lander
            if (extraDistance == 0) { //mineral in center
                pause(.25);
                move(-10, -.5); //knock mineral
                pause(.5);
                move(10, .5); //move back from mineral
            }
            else if (extraDistance < 0) { //mineral to left
                strafe(extraDistance, .3, "left"); //strafe to mineral
                pause(.25);
                move(-10, -.5); //knock mineral
                pause(.5);
                move (10, .5); //move back from mineral
            }
            else if (extraDistance > 0) { //mineral to right
                strafe(-extraDistance, .3, "right"); //strafe to mineral
                pause(.25);
                move( -10, -.5); //knock mineral
                pause(.25);
                move(10, .5); //move back from mineral
            }
            //move away from minerals and towards depot
            pause(.25);
            turn (90, "left");
            move(-extraDistance -30, -.8);
            pause(.25);
            //align with wall
            turn(45, "right");
            pause(.25);
            strafe(25, .3, "right");
            pause(.25);
            //drive
            move(-37, -.8);
            //Release Team Marker
            teamMarker.setPosition(0);
            move(5, .8);
            pause(1);
            teamMarker.setPosition(1);
            pause(.25);
            strafe(5, .3, "right");
            move(58,.8);
            pause(.5);
            move(10, .2);
            /*
            intakeLift.setPower(.7);
            pause(.1);
            intakeSlide.setPower(-1);
            pause(.5);
            intakeSlide.setPower(0);
            intakeLift.setPower(0);*/

            //disable camera!
            detector.disable();
        }catch (InterruptedException e){}
    }

    public void move(double distance, double power) throws InterruptedException{
        int ticks = (int)(distance*C); //add encoders in initializing
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setTargetPosition(ticks);
        leftFrontDrive.setPower(power);
        leftRearDrive.setTargetPosition(ticks);
        leftRearDrive.setPower(power);
        rightFrontDrive.setTargetPosition(ticks);
        rightFrontDrive.setPower(power);
        rightRearDrive.setTargetPosition(ticks);
        rightRearDrive.setPower(power);
        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {heartbeat();}
        halt();
    }

    public void strafe(double distance, double power, String dir) throws InterruptedException{
        int ticks = (int)(distance*C*STRAFE_COEFFICIENT);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (dir.equals("right")) {
            leftFrontDrive.setTargetPosition(ticks);
            leftFrontDrive.setPower(power);
            leftRearDrive.setTargetPosition(-ticks);
            leftRearDrive.setPower(-power);
            rightFrontDrive.setTargetPosition(-ticks);
            rightFrontDrive.setPower(-power);
            rightRearDrive.setTargetPosition(ticks);
            rightRearDrive.setPower(power);
        }
        else if (dir.equals("left")) {
            leftFrontDrive.setTargetPosition(-ticks);
            leftFrontDrive.setPower(-power);
            leftRearDrive.setTargetPosition(ticks);
            leftRearDrive.setPower(power);
            rightFrontDrive.setTargetPosition(ticks);
            rightFrontDrive.setPower(power);
            rightRearDrive.setTargetPosition(-ticks);
            rightRearDrive.setPower(-power);
        }
        while (leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()) {heartbeat();}
        halt();
    }

    public void turn(int degree, String dir)throws InterruptedException{
        //left to pos
        //right to neg
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int deg = 0;
        if (dir.equals("right"))
            deg = -degree;
        else if (dir.equals("left"))
            deg = degree;
        telemetry.addData("desired angle", deg);
        telemetry.update();
        if (dir.equals("left")) {
            while ((int)angles.firstAngle < deg) {

                if ((int)angles.firstAngle < deg-30) {
                    leftFrontDrive.setPower(.5);
                    leftRearDrive.setPower(.5);
                    rightFrontDrive.setPower(-.5);
                    rightRearDrive.setPower(-.5);
                }
                else {
                    leftFrontDrive.setPower(.05);
                    leftRearDrive.setPower(.05);
                    rightFrontDrive.setPower(-.05);
                    rightRearDrive.setPower(-.05);
                }

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }
        else if (dir.equals("right")) {
            while ((int)angles.firstAngle > deg) {
                if ((int)angles.firstAngle > deg+30) {
                    leftFrontDrive.setPower(-.5);
                    leftRearDrive.setPower(-.5);
                    rightFrontDrive.setPower(.5);
                    rightRearDrive.setPower(.5);
                }
                else {
                    leftFrontDrive.setPower(-.05);
                    leftRearDrive.setPower(-.05);
                    rightFrontDrive.setPower(.05);
                    rightRearDrive.setPower(.05);
                }

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.addData("angle", angles.secondAngle);
                telemetry.addData("angle", angles.thirdAngle);
                telemetry.update();
                heartbeat();
            }
        }
        halt();
    }

    public void halt(){
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
    }

    public void lift (double distance, double power) throws InterruptedException{
        int ticks = (int)(distance*LIFT_TICKS_PER_REV/(Math.PI*LIFT_DIAMETER*LIFT_GEAR_RATIO));
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //telemetry.addData("Ticks", ticks);

        liftMotor.setTargetPosition(ticks);
        liftMotor.setPower(power);

        while (liftMotor.isBusy()) {heartbeat(); telemetry.addData("ticks", ticks); telemetry.update();}
        liftMotor.setPower(0);
        telemetry.update();
    }

    public void pause(double time) throws InterruptedException{
        double pause = runtime.time();
        while (runtime.time() - pause < time) {
            heartbeat();
            /*telemetry.addData("Paused","");
            telemetry.update();*/
        }
    }
    private void heartbeat() throws InterruptedException{
        if(!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}