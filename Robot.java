package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot{
    //DcMotor lift = null;
    double x_er_last = 0;
    double y_er_last = 0;
    double x_p_reg = 0;
    double y_p_reg = 0;
    double axialm = 0;
    double dom = 0;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    DcMotor man = null;
    Servo klesh;
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode L;
    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode L) {
        //НЕ ТРОГАТЬ!
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
    }

    public void get_members() { //и это!
        //lift = hardwareMap.get(DcMotor.class, "reechniy_lift");
        man = hardwareMap.get(DcMotor.class, "m");
        klesh = hardwareMap.get(Servo.class, "kl");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //manipulator.setDirection(DcMotorSimple.Direction.FORWARD);
        klesh.setDirection(Servo.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void init_enc_motors() {
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void reset_using_motors() {
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void go_bytime(double axial, double lateral, double yaw, double time) {
        get_members();
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        runtime.reset();
        while (L.opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void go_byenc(double fb, double lr, double trn, double ticks) {
        get_members();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < ticks) | (rightBackDrive.getCurrentPosition() < ticks)) {
            double axial = 0;
            double lateral = 0;
            double yaw = 0;
            double enc1 = rightBackDrive.getCurrentPosition();
            double enc2 = Math.abs(rightFrontDrive.getCurrentPosition());
            double er = ticks-(enc1+enc2)/2;
            double kp = 0.0027;//here is coeff
            double p_reg = er*kp;
            double wh_er1 = enc1 - enc2;
            double wh_er2 = enc2 - enc1;
            double wh_preg1 = wh_er1 * kp;
            double wh_preg2 = wh_er2 * kp;

            if (fb == 1) {
                axial = p_reg;
                lateral = 0;
                yaw = 0;
            }
            if (fb == -1) {
                axial = p_reg;
                lateral = 0;
                yaw = 0;
            }
            if (lr == 1) {
                axial = 0;
                lateral = p_reg;
                yaw = 0;
            }
            if (lr == -1){
                axial = 0;
                lateral = p_reg;
                yaw = 0;
            }
            if (trn == 1) {
                axial = 0;
                lateral = 0;
                yaw = p_reg;
            }
            if (trn == -1){
                axial = 0;
                lateral = 0;
                yaw = p_reg;
            }

            if ((Math.abs(rightFrontDrive.getCurrentPosition()) - rightBackDrive.getCurrentPosition()) > 20) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - rightBackDrive.getCurrentPosition()) > 20) {
                    axial = 0;
                    lateral = 0;
                    yaw = wh_preg2;
                }
            }
            if ((rightBackDrive.getCurrentPosition() - Math.abs(rightFrontDrive.getCurrentPosition())) > 20) {
                while ((rightBackDrive.getCurrentPosition() - Math.abs(rightFrontDrive.getCurrentPosition())) > 20) {
                    axial = 0;
                    lateral = 0;
                    yaw = wh_preg1;
                }
            }

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
    public void go_byenc_x(double x, double napr) {
        get_members();
        init_enc_motors();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < x) | (rightBackDrive.getCurrentPosition() < x)) {
            double enc1 = rightBackDrive.getCurrentPosition();
            double enc2 = Math.abs(rightFrontDrive.getCurrentPosition());
            double kp = 0.0017;//here is coeff
            double kd = 0.0007; //differential coefficient
            reset_using_motors();
            while ((enc2 != x) | (enc1 != x)) {
                double x_er = x*napr - enc1*napr;
                x_p_reg = (x_er)*kp;
                double x_pd = x_p_reg + (x_er_last-x_er)*kd;
                axial = 0;
                lateral = x_pd;
                yaw = 0;

                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();

        }
    }
    public void go_byenc_y(double y, double napr) {
        get_members();
        init_enc_motors();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < y) | (rightBackDrive.getCurrentPosition() < y)) {
            double enc1 = rightBackDrive.getCurrentPosition();
            double enc2 = Math.abs(rightFrontDrive.getCurrentPosition());
            double kp = 0.0017;//here is coeff
            double kd = 0.0007; //differential coefficient
            while ((enc2 != y) | (enc1 != y)) {
                double y_er  = y*napr - enc2*napr;
                y_p_reg = (y_er)*kp;
                double y_pd = y_p_reg + (y_er_last-y_er)*kd;
                double axial = y_pd;
                double lateral = 0;
                double yaw = 0;

                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();

        }
    }
    public void stop_system(){
        get_members();
        double axial = 0;
        double lateral = 0;
        double yaw = 0;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void teleop() {
        get_members();
        double max;
        //axiall это axial для реечного лифта
        //axialm это axial для манипулятора(качельки)
        //double axiall = gamepad2.left_stick_y*0.1;
        double rt = gamepad2.right_trigger;
        // Мега сигма код на его разработку ушла целая недел
        while (rt > 0.5) {
            double axial = -gamepad1.left_stick_y*0.25;
            double lateral = gamepad1.left_stick_x*0.25;
            double yaw = -gamepad1.right_stick_x*0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

        }
        double axial = -gamepad1.left_stick_y*0.7;
        double lateral = gamepad1.left_stick_x*0.7;
        double yaw = -gamepad1.right_stick_x*0.7;
        double servo_position = gamepad2.left_stick_y-0.5;
        axialm = -gamepad2.right_stick_y*0.4+0.05;
        /*
        double lt = gamepad2.left_trigger;
        while (lt > 0){
            axialm = -gamepad2.right_stick_y;
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;
        }
         */
        //double liftPower = axiall;
        double ManPower = axialm;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(ManPower));
        //max = Math.max(max, Math.abs(liftPower));
        if (max > 1.0) {
            //liftPower /=max;
            ManPower /= max;
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;

        }

        //lift.setPower(liftPower);
        klesh.setPosition(servo_position);
        man.setPower(ManPower);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
        //telemetry.addData("reechniy_lift", "$4.2f", liftPower);
        telemetry.addData("servo", servo_position);
        telemetry.update();
    }
}
