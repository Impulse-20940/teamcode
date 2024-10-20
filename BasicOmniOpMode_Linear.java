package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp_new_dev")

public class BasicOmniOpMode_Linear extends LinearOpMode {
    Robot R = new Robot();
    DcMotor lift = null;
    DcMotor manipulator = null;
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    public void runOpMode() {
        R.get_members();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double max;

            double axiall = gamepad2.left_stick_y*0.1;
            double axialm = gamepad2.right_stick_y*0.2;

            double axial   = -gamepad1.left_stick_y*0.5;
            double lateral =  gamepad1.left_stick_x*0.5;
            double yaw     =  gamepad1.right_stick_x*0.5;

            //ManipulatorFrontPower сделал по анологии с осталной подачей напряжения ток я незнаю что там суммировать поэтому суммировал только axialm
            double liftPower = axiall;
            double ManipulatorPower = axialm;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //Тут тоже сделал по анологии
            max = Math.max(max, Math.abs(ManipulatorPower));
            max = Math.max(max, Math.abs(liftPower));

            if (max > 1.0) {
                //И тут тоже по анологии
                liftPower /=max;
                ManipulatorPower /= max;
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            //И здесь тоже
            lift.setPower(liftPower);
            manipulator.setPower(ManipulatorPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //И тут
            telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
            telemetry.addData("reechniy_lift", "$4.2f", liftPower);
            telemetry.update();
        }
    }}
