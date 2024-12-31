package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;
@Autonomous(name="Skibidi Specimen")
public class karelAuton extends LinearOpMode {
    private DcMotor rightFront; //rightFront is the right front wheel of the bot
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor vSlide;
    private DcMotor hSlide;
    private Servo rhWrist;
    private Servo lhWrist;
    private Servo lvWrist;
    private Servo rvWrist;
    private Servo vClaw;
    private Servo hClaw;
    private IMU imu;
    int globalAngle = 0;
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        vSlide = hardwareMap.dcMotor.get("vSlide");
        hSlide = hardwareMap.dcMotor.get("hSlide");
        lvWrist = hardwareMap.servo.get("lvWrist");
        rvWrist = hardwareMap.servo.get("rvWrist");
        lhWrist = hardwareMap.servo.get("lhWrist");
        rhWrist = hardwareMap.servo.get("rhWrist");
        hClaw = hardwareMap.servo.get("hClaw");
        vClaw = hardwareMap.servo.get("vClaw");
        imu = hardwareMap.get(IMU.class, "imu");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        vSlide.setDirection(DcMotor.Direction.REVERSE);
        hSlide.setDirection(DcMotor.Direction.FORWARD);

        vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopResetEncoders();
        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunUsingEncoder();
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Gamepad karel = new Gamepad();
        Gamepad karelNow = new Gamepad();

        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParameters);

        vClaw.setPosition(1);

        waitForStart();
        imu.resetYaw();

        vSlidesUp(1550);
        //wrist down
        lvWrist.setPosition(.25);
        rvWrist.setPosition(-1);
        forward(1200, .5);
        //claw open
        vClaw.setPosition(0);

        vSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stopResetEncoders();

        vSlidesDown(1000);
        reverse(1200, .5);

    }
    public void vSlidesUp(int target){
        vSlide.setTargetPosition(target);
        vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide.setPower(.4);
        while(vSlide.getCurrentPosition() > -target){
            telemetry.addData("slides",vSlide.getCurrentPosition());
            telemetry.update();
        }
        vSlide.setPower(0);
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void vSlidesDown(int target){
        vSlide.setTargetPosition(-target);
        vSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vSlide.setPower(-.1);
        while(vSlide.getCurrentPosition() < target){
            telemetry.addData("slides",vSlide.getCurrentPosition());
            telemetry.update();
        }
        vSlide.setPower(0);
        vSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeRight(int target, double power){
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(-target);
        rightBack.setTargetPosition(target);
        leftBack.setTargetPosition(-target);

        setRunToPosition();

        leftFront.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);

        while(leftFront.isBusy()){
            telemetry.addData("strafing", "rizzy");
            telemetry.update();
        }

        setSpeed(0);
        setRunUsingEncoder();
    }

    public void strafeLeft(int target, double power){
        leftFront.setTargetPosition(-target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(-target);
        leftBack.setTargetPosition(target);

        setRunToPosition();

        leftFront.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);

        while(leftFront.isBusy()){
            telemetry.addData("strafing", "rizzy");
            telemetry.update();
        }

        setSpeed(0);
        setRunUsingEncoder();
    }
    public void reverse(int target, double speed) {
        target = -target;
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        setRunToPosition();
        setSpeed(speed);
        while(leftFront.isBusy()){
            telemetry.addData("forward","sigma");
            telemetry.update();
        }
        setSpeed(0);
        setRunUsingEncoder();
    }
    public void forward(int target, double speed) {
        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        setRunToPosition();
        setSpeed(-speed);

        while(leftFront.isBusy()){
            telemetry.addData("forward","sigma");
            telemetry.update();
        }
        setSpeed(0);
        setRunUsingEncoder();
    }
    public void setRunUsingEncoder(){
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setSpeed(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    public void stopResetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setRunToPosition() {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void turnLPD(int angle) {
        globalAngle = angle;
        double kd = .0025;
        double kp = .0025;
        double minSpeed = .075;
        double error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle) {
            double prevErr = error;
            error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double d = error - prevErr;
            double speed = error * kp + d * kd + minSpeed;
            if (speed > .4) speed = .4;

            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
            sleep(10);

            telemetry.addData("speed", speed);
            telemetry.addData("turning Left", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public void turnRPD(int angle) {
        globalAngle = angle;
        double kd = .0025;
        double kp = .0025;
        double minSpeed = .075;
        double error = angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle) {
            double prevErr = error;
            error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double d = error - prevErr;
            double speed = error * kp + d * kd + minSpeed;
            if (speed > .4) speed = .4;

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            sleep(10);

            telemetry.addData("speed", speed);
            telemetry.addData("turning Right", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
