package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.imu.IMU;
import org.firstinspires.ftc.teamcode.wheelbase.Regualizers;

public class superTeleOp extends Robot {
    Regualizers reg = new Regualizers();
    IMU imu = new IMU();
    superAuto auto = new superAuto();
    public void teleop_lift1() {
        get_members();
        double max;
        //axiall это axial для реечного лифта
        //axialm это axial для манипулятора(качельки)
        double rt1 = gamepad2.right_trigger;
        double axiall = gamepad2.left_stick_y*(1 - rt1)+0.03;
        double axiall2 = -gamepad2.left_stick_y*(1 - rt1)-0.03;
        double axialm = -gamepad2.right_stick_y * (1 - rt1) + 0.05;
        double kles = gamepad2.left_trigger*0.975;

        //rt - считывание правого триггера
        double rt = gamepad1.right_trigger;
        //умножение на rt используется для уменьшения напряжения, подаваемого на моторы
        double axial = -gamepad1.left_stick_y*(1 - rt);
        double lateral = gamepad1.left_stick_x*(1 - rt);
        double yaw = -gamepad1.right_stick_x*(1 - rt);

        //переменные с напряжением, которое будет подаваться на моторы
        double liftPower = axiall;
        double lift2Power = axiall2;
        double kleshPower = kles;
        double kleshPower2 = kles;
        double ManPower = axialm;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
        //балансировка напряжения моторов
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        //подача напряжения на моторы
        lift.setPower(liftPower);
        lift2.setPower(lift2Power);
        klesh.setPosition(kleshPower);
        klesh1.setPosition(kleshPower2);
        man.setPower(ManPower);
        reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
        telemetry.addData("l1", "$4.2f", liftPower);
        telemetry.update();
    }
    public void teleop_lift2() {
        get_members();
        double max;
        boolean iscontrol;
        double rt1 = gamepad2.right_trigger; //правый триггер для захватов и подъемов
        double axiall = gamepad2.left_stick_y*((1 - rt1)*0.75); //мотор качельки
        double axiall2 = -gamepad2.right_stick_y*((1 - rt1)*0.75); //мотор лифта
        //axialm = -gamepad2.right_stick_y*(1 - rt1)+0.05;
        double kles = gamepad2.left_trigger*0.99; //клешня
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
        if (block){
            if (!open_close){
                klesh1.setPosition(0.8);
                open_close = true;
                delay(210);
                klesh1.close();
            }
            else{
                klesh1.setPosition(0);
                open_close = false;
                delay(210);
            }
        }
        if(control){
            if(ic){
                ic180 = false;
                icL180 = false;
                ic = false;
                delay(250);
            }
            else{
                ic180 = false;
                icL180 = false;
                ic = true;
                delay(250);
            }
        }
        if(controlR90){
            if(ic180){
                ic = false;
                icL180 = false;
                ic180 = false;
                delay(250);
            }
            else{
                ic = false;
                icL180 = false;
                ic180 = true;
                delay(250);
            }
        }
        if(controlL90){
            if(icL180){
                ic = false;
                icL180 = false;
                ic180 = false;
                delay(250);
            }
            else{
                ic = false;
                icL180 = true;
                ic180 = false;
                delay(250);
            }
        }
        if (ic){
            axial = -gamepad1.left_stick_y*(1 - rt);
            yaw = -gamepad1.left_stick_x*(1 - rt);
            lateral = -imu.getTurnAngle()*0.012;
        }
        if (ic180){
            /*
            if (getTurnAngle() > 0){
                getangle = 180-getTurnAngle();
            }
            if (getTurnAngle() < 0){
                getangle = -180-getTurnAngle();
            }

             */
            getangle = -90-imu.getTurnAngle();
            axial = gamepad1.left_stick_x*(1 - rt);;
            lateral = getangle*0.012;
            yaw = -gamepad1.left_stick_y*(1 - rt);
        }
        if (icL180){
            /*
            if (getTurnAngle() > 0){
                getangle = 180-getTurnAngle();
            }
            if (getTurnAngle() < 0){
                getangle = -180-getTurnAngle();
            }

             */
            getangle = 90-imu.getTurnAngle();
            axial = gamepad1.left_stick_x*(1 - rt);;
            lateral = getangle*0.012;
            yaw = -gamepad1.left_stick_y*(1 - rt);
        }
        if(!ic180 && !ic){
            axial = -gamepad1.left_stick_y*(1 - rt);
            yaw = -gamepad1.left_stick_x*(1 - rt);
            lateral = -gamepad1.right_stick_x*(1 - rt);
        }
        if(gs){
            //get_sample();
            auto.auto_human(human_state);
            if (human_state == 0){
                human_state = 1;
            } else if (human_state==1) {
                human_state = 0;
            }
            delay(200);
        }
        if (gsa){
            auto.auto_app(apparacy_state);
            if (apparacy_state == 0){
                apparacy_state = 1;
            } else if (apparacy_state == 1) {
                apparacy_state = 0;
            }
            delay(200);
        }
        if(reset_states){
            human_state = 0;
            apparacy_state = 0;
        }
        //умножение на rt используется для уменьшения напряжения, подаваемого на моторы в зависимости от силы нажатия правого триггера
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if(right){
            yaw = -0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(left){
            yaw = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(up){
            axial = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(down){
            axial = -0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(down1){
            axiall2 = -0.2;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            reg.setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(up1){
            axial = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
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
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
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
