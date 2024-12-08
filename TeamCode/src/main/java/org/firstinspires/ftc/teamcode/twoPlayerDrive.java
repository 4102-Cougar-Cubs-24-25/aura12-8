/*
4 servos
* flipping intake on r/l dpad
* flipping outtake on up/down dpad
* 1 for powering intake on x and b
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="sigma 6.5")

public class twoPlayerDrive extends LinearOpMode{
    //define motors and variables here
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
    private boolean hClawOpen = false;
    private boolean vClawOpen = false;

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
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        vSlide.setDirection(DcMotor.Direction.REVERSE);
        hSlide.setDirection(DcMotor.Direction.FORWARD);

        vSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad karel = new Gamepad();
        Gamepad karelNow = new Gamepad();

        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParameters);

        vClaw.setPosition(1);
        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {

            telemetry.addData("sigma", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("rizz", leftFront.getCurrentPosition());
            telemetry.addData("aura (h)", hClawOpen);
            telemetry.addData("fein (v)", vClawOpen);

            telemetry.update();

            karel.copy(karelNow);
            karelNow.copy(gamepad2);

            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1;
            double rx = gamepad1.right_stick_x;
            double div = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(Math.pow((y + x - (rx*.85)),3)/div);
            leftBack.setPower(Math.pow((y - x + (rx*.85)),3)/div);
            rightFront.setPower(Math.pow((y - x - (rx*.85)),3)/div);
            rightBack.setPower(Math.pow((y + x + (rx*.85)),3)/div);

            if(gamepad2.right_trigger > 0){
                vSlide.setPower(0.7);
            } else if(gamepad2.right_bumper){
                vSlide.setPower(-0.5);
            } else {
                vSlide.setPower(0);
            }

            if(gamepad2.left_trigger > 0){
                hSlide.setPower(0.7);
            } else if(gamepad2.left_bumper){
                hSlide.setPower(-0.5);
            } else {
                hSlide.setPower(0);
            }

            //all wrists and claws are normal (positional) servos
            //find the positions for all of them

            //vWrist up - test this
            if(gamepad2.dpad_up){
                lvWrist.setPosition(-1);
                rvWrist.setPosition(.25);
            }

            //vWrist down - test this
            if(gamepad2.dpad_down){
                lvWrist.setPosition(.25);
                rvWrist.setPosition(-1);
            }

            //hWrist up - test this
            if(gamepad2.dpad_left){
                rhWrist.setPosition(0);
                lhWrist.setPosition(1);
            }

            //hWrist down - test this
            if(gamepad2.dpad_right){
                rhWrist.setPosition(.6);
                lhWrist.setPosition(-.6);
            }

            //vClaw open/close toggle - needs testing
            if(karelNow.x && !karel.x){
                hClawOpen = !hClawOpen;

                if(hClawOpen == true){
                    hClaw.setPosition(0);
                } else if(hClawOpen == false){
                    hClaw.setPosition(1);
                }
            }

            //hClaw open/close toggle - this works
            if(karelNow.b && !karel.b){
                vClawOpen = !vClawOpen;

                if(vClawOpen == true){
                    vClaw.setPosition(0);
                } else if(vClawOpen == false) {
                    vClaw.setPosition(1);
                }
            }
        }
    }
}