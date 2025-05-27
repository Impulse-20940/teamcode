package org.firstinspires.ftc.teamcode.wheelbase;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.imu.IMU;

public class Regualizers extends Robot {
    IMU imu = new IMU();
    public void go_bytime(double axial, double lateral, double time) {
        //езда по времени
        get_members();
        double getangle = imu.getTurnAngle();
        double yaw = -getangle * 0.012;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(-leftBackPower);
        rightBackDrive.setPower(-rightBackPower);
        runtime.reset();
        while (L.opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    public void go_byenc_simple(double a, double l, double rast) {
        get_members();
        //rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int)a);
        //rightFrontDrive.setTargetPosition(-rightFrontDrive.getCurrentPosition() + (int)-l);
        reset_using_motors();
        while (L.opModeIsActive() && ((rightFrontDrive.getCurrentPosition() < rast) | (-rightBackDrive.getCurrentPosition() < rast))) {
            //езда по времени
            double getangle = imu.getTurnAngle();
            double axial = a;
            double lateral = (45-getangle)*0.012;
            yaw = l;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);

            telemetry.addData("Now is", "%7d :%7d",
                    rightBackDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
            telemetry.addData("Angle is:", getangle);
            telemetry.update();
        }
    }
    public void go_byenc(double x, double y) {
        //езда по энкодеру
        get_members();
        reset_using_motors();
        init_enc_motors();
        while ((-rightFrontDrive.getCurrentPosition() < x && rightBackDrive.getCurrentPosition() < y) | L.opModeIsActive()){
            double enc1 = -rightFrontDrive.getCurrentPosition();
            double enc2 = rightBackDrive.getCurrentPosition();
            //double kp = 0.0007;//here is coeff
            //double kd = 0.00109;
            double kp = 0.0019;//here is coeff
            double kt = 0.012;
            double kd = 0.0039; //differential coefficient

            double x_er = x - enc1;
            double x_p_reg = (x_er)*kp;

            double y_er = y - enc2;
            double y_p_reg = (y_er)*kp;

            double getangle = imu.getTurnAngle();
            double x_er_d = x_er - x_er_last;
            double x_d_reg = kd*x_er_d*(1/x_er);
            double x_pd = x_p_reg + x_d_reg;

            double y_er_d = y_er - y_er_last;
            double y_d_reg = kd*y_er_d*(1/y_er);
            double y_pd = y_p_reg + y_d_reg;


            double axial = y_pd;
            double lateral = -getangle*kt;
            double yaw = x_pd;

            x_er_last = x_er;
            y_er_last = y_er;

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
    }
    public void go_byenc_x(double stable, double x) {
        get_members();
        reset_using_motors();
        init_enc_motors();
        while (L.opModeIsActive() && (Math.abs(-rightFrontDrive.getCurrentPosition()) < Math.abs(x))) {
            //езда по энкодеру
            double enc1 = -rightFrontDrive.getCurrentPosition();
            double kp = 0.0039;//here is coeff
            double kt = 0.012;
            //double kd =  0.0004; //differential coefficient
            double x_er = x - enc1;
            double x_p_reg = x_er*kp;
            double getangle = stable-imu.getTurnAngle();
            //double x_er_d = x_er - x_er_last;
            //double x_d_reg = kd*x_er_d*(1/x_er);
            //double x_pd = x_p_reg + x_d_reg;
            //x_er_last = x_er;;

            double axial = 0;
            double lateral = getangle*kt;
            double yaw = x_p_reg;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);

            telemetry.addData("Now is", "%7d :%7d",
                    rightBackDrive.getCurrentPosition(),
                    -rightFrontDrive.getCurrentPosition());
            telemetry.addData("Angle is:", getangle);
            telemetry.update();
        }
        setMPower(0, 0, 0, 0);
    }
    public void go_byenc_y(double stable, double y, double kp) {
        //езда по энкодеру
        get_members();
        reset_using_motors();
        init_enc_motors();
        if (y > 0){
            while ((-rightFrontDrive.getCurrentPosition() < y) && L.opModeIsActive()){
                double enc2 = -rightFrontDrive.getCurrentPosition();
                double kt = 0.012;
                //double kd = 0.0004; //differential coefficient
                double y_er = y - enc2;
                double y_p_reg = y_er*kp;
                double getangle = stable-imu.getTurnAngle();
                //double y_er_d = y_er - y_er_last;
                //double y_d_reg = kd*y_er_d*(1/y_er);
                //double y_pd = y_p_reg + y_d_reg;
                //y_er_last = y_er;

                double axial = y_p_reg;
                double lateral = getangle*kt;
                double yaw = 0;

                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);

                telemetry.addData("Now is", "%7d",
                        -rightFrontDrive.getCurrentPosition());
                telemetry.addData("Angle is:", getangle);
                telemetry.update();

                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (y < 0) {
            while ((-rightFrontDrive.getCurrentPosition() > y) && L.opModeIsActive()){
                double enc2 = -rightFrontDrive.getCurrentPosition();
                double kt = 0.012;
                //double kd = 0.0004; //differential coefficient
                double y_er = y - enc2;
                double y_p_reg = y_er*kp;
                double getangle = stable-imu.getTurnAngle();
                //double y_er_d = y_er - y_er_last;
                //double y_d_reg = kd*y_er_d*(1/y_er);
                //double y_pd = y_p_reg + y_d_reg;
                //y_er_last = y_er;

                double axial = y_p_reg;
                double lateral = getangle*kt;
                double yaw = 0;

                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);

                telemetry.addData("Now is", "%7d",
                        -rightFrontDrive.getCurrentPosition());
                telemetry.addData("Angle is:", getangle);
                telemetry.update();

                lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        setMPower(0, 0, 0, 0);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void turn(double angle, double kp){
        //функция поворота по гироскопу
        get_members();
        while (Math.abs(angle) > Math.abs(imu.getTurnAngle())  && L.opModeIsActive()){
            if (angle < 0){
                Er = (angle+6) - (imu.getTurnAngle());
            } else if (angle > 0) {
                Er = (angle-6) - (imu.getTurnAngle());
            }
            //double kp = 0.0012;
            double P = kp * Er;
            //double kd = -0.0005;
            //double er_d = Er - Er_last;
            //double D = kd*er_d*(1/Er);
            //Er_last = Er;

            setMPower(0, -P, 0, P);
            telemetry.addData("getAngle()", imu.getTurnAngle());
            telemetry.update();
        }
        setMPower(0, 0, 0, 0);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void stop_system(){
        get_members();
        setMPower(0, 0,0, 0);
    }
    public void stable(double stable, long time, double kt){
        runtime.reset();
        while(L.opModeIsActive() && runtime.seconds() < time){
            double getangle = stable-imu.getTurnAngle();
            double axial = 0;
            double lateral = getangle*kt;
            double yaw = 0;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);
        }
        setMPower(0, 0, 0, 0);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void stable180(double stable, long time, double kt){
        runtime.reset();
        double getangle = 0;
        while(L.opModeIsActive() && runtime.seconds() < time){
            if (imu.getTurnAngle() > 0){
                getangle = stable-imu.getTurnAngle();
            }
            if (imu.getTurnAngle() < 0){
                getangle = -stable-imu.getTurnAngle();
            }
            double axial = 0;
            double lateral = getangle*kt;
            double yaw = 0;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(-leftBackPower);
            rightBackDrive.setPower(-rightBackPower);
        }
        setMPower(0, 0, 0, 0);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setMPower(double rb, double rf, double lf, double lb){
        //подача напряжения на моторы
        get_members();//Устоновить мощность на моторы
        rightFrontDrive.setPower(rf);
        leftFrontDrive.setPower(lf);
        rightBackDrive.setPower(rb);
        leftBackDrive.setPower(lb);
    }
}
