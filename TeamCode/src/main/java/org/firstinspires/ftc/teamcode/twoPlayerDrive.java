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

@TeleOp(name="sigma 7.0")

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

        hSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad karel = new Gamepad();
        Gamepad karelNow = new Gamepad();

        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(imuParameters);

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {

            telemetry.addData("sigma (imu)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("rizz (lf)", leftFront.getCurrentPosition());
            telemetry.addData("aura (h)", hClawOpen);
            telemetry.addData("fein (v)", vClaw.getPosition());

            telemetry.update();

            karel.copy(karelNow);
            karelNow.copy(gamepad2);

            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double div = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(Math.pow((y + x - (rx*.85)),3)/div);
            leftBack.setPower(Math.pow((y - x + (rx*.85)),3)/div);
            rightFront.setPower(Math.pow((y - x - (rx*.85)),3)/div);
            rightBack.setPower(Math.pow((y + x + (rx*.85)),3)/div);

            if(gamepad2.right_trigger > 0){
                vSlide.setPower(-0.9);
            } else if(gamepad2.right_bumper){
                vSlide.setPower(0.7);
            } else {
                vSlide.setPower(-0.1);
            }

            if(gamepad2.left_trigger > 0){
                hSlide.setPower(0.9);
            } else if(gamepad2.left_bumper){
                hSlide.setPower(-0.7);
            } else {
                hSlide.setPower(0);
            }

            //all wrists and claws are normal (positional) servos
            if(gamepad2.dpad_right){
                lvWrist.setPosition(0);
                rvWrist.setPosition(.66);
            }

            if(gamepad2.dpad_left){
                lvWrist.setPosition(.66);
                rvWrist.setPosition(0);
            }

            if(gamepad2.dpad_down){
                rhWrist.setPosition(0);
                lhWrist.setPosition(.35);
            }

            if(gamepad2.dpad_up){
                rhWrist.setPosition(.35);
                lhWrist.setPosition(0);
            }

            if(karelNow.x && !karel.x){
                hClawOpen = !hClawOpen;

                if(hClawOpen){
                    hClaw.setPosition(0);
                } else if(!hClawOpen){
                    hClaw.setPosition(.8);
                }
            }

            if(karelNow.b && !karel.b){
                vClawOpen = !vClawOpen;
            }
            if(gamepad2.b) {
                if (vClawOpen) {
                    vClaw.setPosition(.8);
                } else if (!vClawOpen) {
                    vClaw.setPosition(0);
                }
            }
        }
    }
}
