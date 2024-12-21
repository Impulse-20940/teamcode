package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Robot{
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    DcMotor lift = null;
    DcMotor lift2 = null;
    DcMotor man = null;
    Servo klesh;
    Servo klesh1;
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

    double x_er_last = 0;
    double y_er_last = 0;
    double x_p_reg = 0;
    double y_p_reg = 0;
    double axialm = 0;
    double axial = 0;
    double lateral = 0;
    double yaw = 0;
    double Er_last = 0;
    double Er = 0;

    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode L) {
        //НЕ ТРОГАТЬ!
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
    }

    public void get_members() { //и это!
        lift = hardwareMap.get(DcMotor.class, "l1");
        lift2 = hardwareMap.get(DcMotor.class, "l2");
        man = hardwareMap.get(DcMotor.class, "m");
        klesh = hardwareMap.get(Servo.class, "kl");
        klesh1 = hardwareMap.get(Servo.class, "kl1");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        man.setDirection(DcMotor.Direction.FORWARD);
        klesh.setDirection(Servo.Direction.FORWARD);
        klesh1.setDirection(Servo.Direction.REVERSE);
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

            while (fb == 1) {
                axial = p_reg;
                lateral = 0;
                yaw = 0;
            }
            while (fb == -1) {
                axial = p_reg;
                lateral = 0;
                yaw = 0;
            }
            while (lr == 1) {
                axial = 0;
                lateral = p_reg;
                yaw = 0;
            }
            while (lr == -1){
                axial = 0;
                lateral = p_reg;
                yaw = 0;
            }
            while (trn == 1) {
                axial = 0;
                lateral = 0;
                yaw = p_reg;
            }
            while (trn == -1){
                axial = 0;
                lateral = 0;
                yaw = p_reg;
            }



            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            while ((Math.abs(rightFrontDrive.getCurrentPosition()) - rightBackDrive.getCurrentPosition()) > 100) {
                while ((Math.abs(rightFrontDrive.getCurrentPosition()) - rightBackDrive.getCurrentPosition()) > 100) {
                    axial = 0;
                    lateral = 0;
                    yaw = -0.5;
                    leftFrontPower = axial + lateral + yaw;
                    rightFrontPower = axial - lateral - yaw;
                    leftBackPower = axial - lateral + yaw;
                    rightBackPower = axial + lateral - yaw;

                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftBackDrive.setPower(leftBackPower);
                    rightBackDrive.setPower(rightBackPower);
                }
            }
            while ((rightBackDrive.getCurrentPosition() - Math.abs(rightFrontDrive.getCurrentPosition())) > 100) {
                while ((rightBackDrive.getCurrentPosition() - Math.abs(rightFrontDrive.getCurrentPosition())) > 100) {
                    axial = 0;
                    lateral = 0;
                    yaw = 0.5;
                    leftFrontPower = axial + lateral + yaw;
                    rightFrontPower = axial - lateral - yaw;
                    leftBackPower = axial - lateral + yaw;
                    rightBackPower = axial + lateral - yaw;

                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftBackDrive.setPower(leftBackPower);
                    rightBackDrive.setPower(rightBackPower);
                }
            }
            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
    public void go_byenc_x(double x, double napr) {
        get_members();
        init_enc_motors();
        reset_using_motors();
        x *= napr;
        while (rightBackDrive.getCurrentPosition() < x) {
            double enc1 = rightBackDrive.getCurrentPosition();
            double kp = 0.0017;//here is coeff
            double kt = 0.0007;
            //double kd = 0.007; //differential coefficient
            double x_er = x*napr - enc1;
            x_p_reg = (x_er)*kp;
            double getangle = getTurnAngle();
            //double x_er_d = x_er - x_er_last;
            //double x_d_reg = kd*x_er_d*(1/x_er);
            //double x_pd = x_p_reg + x_d_reg;
            //x_er_last = x_er;

            axial = 0;
            lateral = x_p_reg;
            //lateral = x_pd;
            yaw = getangle*kt;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Now is", "%7d :%7d",
                    Math.abs(rightBackDrive.getCurrentPosition()),
                    Math.abs(rightFrontDrive.getCurrentPosition()));
            telemetry.addData("Angle is:", getangle);
            telemetry.update();

        }
        setMPower(0, 0, 0, 0);
    }
    public void go_byenc_y(double y, double napr) {
        get_members();
        init_enc_motors();
        reset_using_motors();
        while ((Math.abs(rightFrontDrive.getCurrentPosition()) < y) | (rightBackDrive.getCurrentPosition() < y)) {
            double enc1 = rightBackDrive.getCurrentPosition();
            double enc2 = Math.abs(rightFrontDrive.getCurrentPosition());
            double kp = 0.0017;//here is coeff
            //double kd = 0.0007; //differential coefficient
            double y_er  = y*napr - (enc1*napr+enc2*napr)/2;
            y_p_reg = (y_er)*kp;
            //double y_d_reg = (y_er - y_er_last)*kd;
            //double y_pd = y_p_reg + y_d_reg;
            //y_er_last = y_er;

            axial = y_p_reg;
            lateral = 0;
            yaw = -getTurnAngle()*kp;

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
    public void go_byenc_xy(double x, double y){
        get_members();
        init_enc_motors();
        reset_using_motors();
        while ((rightFrontDrive.getCurrentPosition() < x) | (Math.abs(rightBackDrive.getCurrentPosition()) < y)) {
            double enc1 = Math.abs(rightBackDrive.getCurrentPosition());
            double enc2 = rightFrontDrive.getCurrentPosition();
            double kp = 0.0017;//here is coeff
            //double kd = 0.0007; //differential coefficient
            double x_er  = x - enc1;
            double y_er = y - enc2;
            x_p_reg = x_er*kp;
            y_p_reg = y_er*kp;
            //double x_d_reg = (x_er - x_er_last)*kd;
            //double y_d_reg = (y_er - y_er_last)*kd;
            //double x_pd = x_p_reg + x_d_reg;
            //double y_pd = y_p_reg + y_d_reg;
            x_er_last = x_er;
            y_er_last = y_er;

            axial = y_p_reg;
            lateral = x_p_reg;
            yaw = -getTurnAngle()*kp;



            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Now is", "%7d :%7d :%7d",
                    Math.abs(rightBackDrive.getCurrentPosition()),
                    rightFrontDrive.getCurrentPosition(),
            getTurnAngle());
            telemetry.update();

        }
    }
    public void turn(double angle){
        get_members();
        while (Math.abs(angle+7) > Math.abs(getTurnAngle())  && L.opModeIsActive()){
            if (angle < 0){
                Er = (angle+6) - (getTurnAngle());
            } else if (angle > 0) {
                Er = (angle-6) - (getTurnAngle());
            }
            double kp = 0.0012;
            double P = kp * Er;
            double kd = -0.0005;
            double er_d = Er - Er_last;
            double D = kd*er_d*(1/Er);
            Er_last = Er;

            setMPower(0, -P+D, 0, P+D);
            telemetry.addData("getAngle()", getTurnAngle());
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
        double rt2 = gamepad2.right_trigger;
        double axiall = gamepad2.left_stick_y*(1-rt2)+0.03;
        double axiall2 = -gamepad2.left_stick_y-0.03;
        axialm = -gamepad2.right_stick_y*0.37*(1-rt2)+0.07;
        double rt1 = gamepad1.right_trigger;
        double kles = gamepad2.left_trigger*0.975;
        double axial = -gamepad1.left_stick_y*(1 - rt1);
        double lateral = gamepad1.left_stick_x*(1 - rt1);
        double yaw = -gamepad1.right_stick_x*(1 - rt1);
        double liftPower = axiall;
        double lift2Power = axiall2;
        double kleshPower = kles;
        double kleshPower2 = kles;
        double ManPower = axialm;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max = Math.max(max, Math.abs(ManPower));
        max = Math.max(max, Math.abs(liftPower));
        max = Math.max(max, Math.abs(kleshPower));
        max = Math.max(max, Math.abs(kleshPower2));
        if (max > 1.0) {
            liftPower /=max;
            ManPower /= max;
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
            kleshPower /= max;
            kleshPower2 /= max;

        }

        lift.setPower(liftPower);
        lift2.setPower(lift2Power);
        klesh.setPosition(kleshPower);
        klesh1.setPosition(kleshPower2);
        man.setPower(ManPower);
        setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        //telemetry.addData("manipulator", "%4.2f" , ManipulatorPower);
        telemetry.addData("l1", "$4.2f", liftPower);
        telemetry.update();
    }
    double getTurnAngle() {
        get_members();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }
    void setMPower(double rb,double rf,double lf,double lb){
        get_members();//Устоновить мощность на моторы
        rightFrontDrive.setPower(rf);
        leftFrontDrive.setPower(lf);
        rightBackDrive.setPower(rb);
        leftBackDrive.setPower(lb);
    }
}
