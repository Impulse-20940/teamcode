package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.imu.IMU;
import org.firstinspires.ftc.teamcode.wheelbase.Regualizers;

public class superTeleOp extends Robot {
    Regualizers reg = new Regualizers();
    IMU imu = new IMU();
    superAuto auto = new superAuto();

    public void teleop() {
        get_members();
        double max;
        boolean iscontrol;
        double rt1 = gamepad2.right_trigger; //правый триггер для захватов и подъемов
        double axiall = gamepad2.left_stick_y * ((1 - rt1) * 0.75); //мотор качельки
        double axiall2 = -gamepad2.right_stick_y * ((1 - rt1) * 0.75); //мотор лифта
        //axialm = -gamepad2.right_stick_y*(1 - rt1)+0.05;
        double kles = gamepad2.left_trigger * 0.99; //клешня
        //rt - считывание правого триггера
        double rt = gamepad1.right_trigger; //правый триггер для кб
        boolean block = gamepad2.right_bumper;
        boolean up1 = gamepad2.dpad_up;
        boolean down1 = gamepad2.dpad_down;
        boolean control = gamepad1.y;
        boolean controlR90 = gamepad1.right_bumper;
        boolean gs = gamepad1.right_stick_button;
        boolean gsa = gamepad1.left_stick_button;
        boolean reset_states = gamepad1.a;
        boolean controlL90 = gamepad1.left_bumper;
        if (block) {
            if (!open_close) {
                klesh1.setPosition(0.8);
                open_close = true;
                delay(210);
                klesh1.close();
            } else {
                klesh1.setPosition(0);
                open_close = false;
                delay(210);
            }
        }
        //умножение на rt используется для уменьшения напряжения, подаваемого на моторы в зависимости от силы нажатия правого триггера
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if (right) {
            yaw = -0.25;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if (left) {
            yaw = 0.25;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if (up) {
            axial = 0.25;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if (down) {
            axial = -0.25;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if (down1) {
            axiall2 = -0.2;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if (up1) {
            axial = 0.25;
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        //переменные с напряжением, которое будет подаваться на моторы
        double liftPower = axiall;
        double lift2Power = axiall2;
        double kleshPower = kles;
        double kleshPower2 = kles1;
        //double ManPower = axialm;
        //балансировка напряжения моторов
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //подача напряжения на моторы
        lift.setPower(lift2Power);
        lift2.setPower(liftPower);
        klesh.setPosition(kleshPower);
        //man.setPower(ManPower);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(-rightBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
        telemetry.addData("l1", "$4.2f", liftPower);
        telemetry.update();
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}