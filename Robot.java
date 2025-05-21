package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    //инициализация всех переменных
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

    double x_er_last;
    double y_er_last;
    double x_p_reg;
    double y_p_reg;
    double axialm;
    double axial;
    double lateral;
    double yaw;
    double Er_last;
    double Er;
    boolean open_close = false;
    boolean ic;
    boolean ic180;
    boolean icL180;
    double kles1;
    double getangle = 0;
    public static int human_state = 0;
    int apparacy_state = 0;
    public void init_classes(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode L) {
        //НЕ ТРОГАТЬ!
        //инициализация классов
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
    }

    public void get_members() {//и это!
        //инициализация всех используемых устройств
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
        //инициализация моторов, используемых энкодер
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset_using_motors() {
        //сброс моторов
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void go_bytime(double axial, double lateral, double time) {
        //езда по времени
        get_members();
        double getangle = getTurnAngle();
        yaw = -getangle*0.012;
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
            double getangle = getTurnAngle();
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

            double getangle = getTurnAngle();
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
            double getangle = stable-getTurnAngle();
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
                double getangle = stable-getTurnAngle();
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
                double getangle = stable-getTurnAngle();
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
        while (Math.abs(angle) > Math.abs(getTurnAngle())  && L.opModeIsActive()){
            if (angle < 0){
                Er = (angle+6) - (getTurnAngle());
            } else if (angle > 0) {
                Er = (angle-6) - (getTurnAngle());
            }
            //double kp = 0.0012;
            double P = kp * Er;
            //double kd = -0.0005;
            //double er_d = Er - Er_last;
            //double D = kd*er_d*(1/Er);
            //Er_last = Er;

            setMPower(0, -P, 0, P);
            telemetry.addData("getAngle()", getTurnAngle());
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

    public void teleop_lift1() {
        get_members();
        double max;
        //axiall это axial для реечного лифта
        //axialm это axial для манипулятора(качельки)
        double rt1 = gamepad2.right_trigger;
        double axiall = gamepad2.left_stick_y*(1 - rt1)+0.03;
        double axiall2 = -gamepad2.left_stick_y*(1 - rt1)-0.03;
        axialm = -gamepad2.right_stick_y*(1 - rt1)+0.05;
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
        setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);

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
            lateral = -getTurnAngle()*0.012;
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
            getangle = -90-getTurnAngle();
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
            getangle = 90-getTurnAngle();
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
            auto_human(human_state);
            if (human_state == 0){
                human_state = 1;
            } else if (human_state==1) {
                human_state = 0;
            }
            delay(200);
        }
        if (gsa){
            auto_app(apparacy_state);
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
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(left){
            yaw = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(up){
            axial = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(down){
            axial = -0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(down1){
            axiall2 = -0.2;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
        }
        if(up1){
            axial = 0.25;
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            setMPower(rightBackPower, rightFrontPower, leftFrontPower, leftBackPower);
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
    double getTurnAngle() {
        //получить текущий угол поворота
        get_members();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        return angles.firstAngle;
    }
    void setMPower(double rb,double rf,double lf,double lb){
        //подача напряжения на моторы
        get_members();//Устоновить мощность на моторы
        rightFrontDrive.setPower(rf);
        leftFrontDrive.setPower(lf);
        rightBackDrive.setPower(rb);
        leftBackDrive.setPower(lb);
    }
    void delay(long millis){
        try{
            Thread.sleep(millis);
        } catch (InterruptedException ex){
            Thread.currentThread().interrupt();
        }
    }
    void k_up(double power, long time){
        get_members();
        lift2.setPower(power);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(time);
        lift2.setPower(0);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void lift_up(double power, long time){
        get_members();
        lift.setPower(power);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(time);
        lift.setPower(0);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void stable(double stable, long time, double kt){
        runtime.reset();
        while(L.opModeIsActive() && runtime.seconds() < time){
            double getangle = stable-getTurnAngle();
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
            if (getTurnAngle() > 0){
                getangle = stable-getTurnAngle();
            }
            if (getTurnAngle() < 0){
                getangle = -stable-getTurnAngle();
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
    void calibrate_imu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "SensorBNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();
    }
    void get_sample(){
        go_byenc_y(0, -270, 0.0012);
        klesh.setPosition(1);
        k_up(-0.45, 500);
        lift_up(0.55, 1800);
        k_up(-0.45, 500);
    }
    void auto_human(int state){
        if (state == 0){
            double stop = 0;
            while(stop != 1 && !gamepad1.x){
                stable(0, 2, 0.012);
                go_byenc_y(0, 430, 0.0012);
                go_byenc_x(0, 600);
                stop = 1;
            }
        }
        if (state == 1){
            double stop = 0;
            while (stop != 1 && !gamepad1.x) {
                stable(-90, 2, 0.012);
                go_byenc_x(-90, -800);
                delay(100);
                go_byenc_y(-90, 400, 0.0012);
                k_up(-0.55, 1000);
                lift_up(0.55, 1800);
                stop = 1;
            }
        }
    }
    void auto_app(int state){
        if (state == 0){

        }
        if (state == 1){

        }
    }
}
