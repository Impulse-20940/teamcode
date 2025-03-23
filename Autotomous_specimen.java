package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name="Autonomous_specimen")
public class Autotomous_specimen extends LinearOpMode {
    BNO055IMU imu;
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
    double x_er_last;
    public static long time1;
    public static long time;
    public static double kp;
    @Override
    public void runOpMode() {
        Robot R = new Robot();
        R.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);

        //*************Не трогать!****************
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
        //*****************************************

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "SensorBNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            sleep(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        // Wait for the game to start (driver presses START)
        waitForStart();
        //***********Main code*************
        //*****p1
        klesh.setPosition(0);
        klesh1.setPosition(0);
        R.delay(500);
        R.k_up(-0.45, 1000);
        R.delay(500);
        klesh1.setPosition(1);
        R.delay(500);
        klesh1.close();
        R.k_up(0.55, 1100);
        R.delay(500);
        R.lift_up(0.5, 1490);
        R.delay(500);
        //R.go_byenc_y(0, 1026);
        //R.go_byenc_y(0, 978);
        R.go_byenc_y(0, 956, kp); //0.0012
        R.go_byenc_simple(-0.2, 0, -150);
        //R.go_byenc_y(0, rast);
        //R.k_up(-0.5, 500);
        R.delay(1000);
        R.lift_up(-0.5, time); //170
        R.k_up(0.55, time1);//1500
        R.delay(2000);
        R.go_bytime(-0.45, 0, 2);
        R.delay(1000);
        R.go_bytime(-0.8, 0, 2);
        /*
        R.k_up(-0.55, 427);
        R.delay(2500);
        R.lift_up(-0.55, 1800);
        R.k_up(0.55, 500);
        R.delay(500);
         */
        klesh.setPosition(1);
        R.delay(500);
        //********p2
        //R.go_byenc_y(0, -100);
        R.go_byenc_y(0, -500, 0.0012);
        /*
        R.delay(300);
        R.stable(-90, 2, 0.012);
        R.delay(300);
        R.go_byenc_x(0, -500);
        R.delay(300);
        R.go_byenc_y(-90, rast1);
        R.delay(300);
        klesh.setPosition(0);
        R.k_up(-0.47, 1000);
        R.lift_up(0.55, 1800);
        klesh.setPosition(1);
        R.k_up(0.55, 1000);
        R.lift_up(-0.55, 1800);
        //*********************************
        R.go_byenc_y(-90, -580);
        R.delay(300);
        R.go_byenc_x(-90, 300);
        R.delay(300);
        R.stable(0, 3, 0.012);
        R.go_byenc_y(0, -310);
        //**************p3
        R.lift_up(0.5, 1450);
        R.delay(500);
        //R.go_byenc_y(0, 1026);
        R.go_byenc_y(0, 996);
        //R.k_up(-0.5, 500);
        R.delay(1000);
        R.lift_up(-0.5, 170);
        R.k_up(-0.55, 450);
        R.delay(2500);
        R.lift_up(-0.55, 1800);
        R.delay(500);
        klesh.setPosition(1);
        R.delay(500);

         */
    }
}