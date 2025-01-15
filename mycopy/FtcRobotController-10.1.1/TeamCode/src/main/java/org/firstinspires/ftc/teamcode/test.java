package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class Test extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor arm = null; // the arm motor
    public DcMotor wrist = null; // the wrist servo

    @Override
    public void runOpMode() throws InterruptedException {

        CRServo intake = this.hardwareMap.crservo.get("intake");
        Servo claw = this.hardwareMap.servo.get("claw");

        waitForStart();


        while (opModeIsActive()) {

            double spin = Math.abs(gamepad1.left_trigger-0.5);

            if (gamepad1.left_trigger > 0){
                intake.setPower(spin);
                Thread.sleep(200);
            }
            else if (gamepad1.right_trigger > 0){
                intake.setPower(-spin);
                Thread.sleep(200);
            }
            else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger==0)){
                intake.setPower(0.0);
            }


            telemetry.addData("Power: ", intake.getPower());
            telemetry.update();


            if (gamepad1.circle){
                claw.setPosition(-1.0);

            }
            else if (gamepad1.square){
                claw.setPosition(0.0);
            }

            // telemetry.addData("Servo Pos: ", claw.getPosition());
            // telemetry.update();

        }
    }
}
