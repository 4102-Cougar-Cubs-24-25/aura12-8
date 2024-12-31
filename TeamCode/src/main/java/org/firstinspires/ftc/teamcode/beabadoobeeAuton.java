/*
List of things robot should do:

-Get one sample in a high basket
-Grab another sample
-Put the next sample in the basket
-Park



 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;

@Autonomous(name="Skibidi Park")
public class beabadoobeeAuton extends LinearOpMode {
    private IMU imu;
    private DcMotor rightFront; //rightFront is the right front wheel of the bot
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor vSlide; //set to DcMotorEx later on
    private Servo vClaw;

    int globalAngle = 0;

    public void runOpMode() throws InterruptedException{

        vClaw = hardwareMap.servo.get("vClaw");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        imu = hardwareMap.get(IMU.class, "imu");
        vSlide = hardwareMap.dcMotor.get("vSlide");
        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(imuParameters);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vSlide.setDirection(DcMotor.Direction.FORWARD);

        vClaw.setPosition(1);
        waitForStart();
        imu.resetYaw();

        strafeRight(2500, 0.5);

    }

    public void strafeRight(int ms, double power){
        leftFront.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);

        sleep(ms);

        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
    }

    public void strafeLeft(int ms, double power){
        leftFront.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);

        sleep(ms);

        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
    }

    public void moveForward(int ms, double power){
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        sleep(ms);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    public void moveReverse(int ms, double power){
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);

        sleep(ms);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnRight(int ms, double power){
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);

        sleep(ms);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnLeft(int ms, double power){
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        sleep(ms);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnLPD(int angle){
        globalAngle = angle;
        double kd = .0025;
        double kp = .0025;
        double minSpeed = .075;
        double error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle){
            double prevErr = error;
            error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double d = error - prevErr;
            double speed = error * kp + d * kd + minSpeed;
            if(speed > .4) speed = .4;

            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(speed);
            rightBack.setPower(speed);
            sleep(10);

            telemetry.addData("speed", speed);
            telemetry.addData("turning Left",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void turnRPD(int angle){
        globalAngle = angle;
        double kd = .0025;
        double kp = .0025;
        double minSpeed = .075;
        double error = angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle){
            double prevErr = error;
            error = abs(angle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double d = error - prevErr;
            double speed = error * kp + d * kd + minSpeed;
            if(speed > .4) speed = .4;

            leftFront.setPower(speed);
            leftBack.setPower(speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            sleep(10);

            telemetry.addData("speed", speed);
            telemetry.addData("turning Right",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    //fix this
    public void reverse(int target, double speed){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(leftFront.getCurrentPosition() > target){
            leftFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightFront.setPower(-speed);
            rightBack.setPower(-speed);
            sleep(10);

            telemetry.addData("speed", speed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    public void forward(int target, double speed){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(target);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        leftFront.resetDeviceConfigurationForOpMode();

        while(leftFront.getCurrentPosition() > target){
            sleep(10);

            telemetry.addData("speed", speed);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void slidesUp(int ms){
        vSlide.setPower(-1);
        sleep(ms);
        vSlide.setPower(0);
    }
}
