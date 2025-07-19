package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
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
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public DcMotor lift = null;
    public DcMotor lift2 = null;
    public DcMotor man = null;
    public Servo klesh;
    public Servo klesh1;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;
    public LinearOpMode L;

    public double x_er_last;
    public double y_er_last;
    double x_p_reg;
    double y_p_reg;
    double axialm;
    public double axial;
    public double lateral;
    public double yaw;
    double Er_last;
    public double Er;
    public boolean open_close = false;
    public boolean ic;
    public boolean ic180;
    public boolean icL180;
    public double kles1;
    public double getangle = 0;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        klesh = hardwareMap.get(Servo.class, "kl");
        klesh1 = hardwareMap.get(Servo.class, "kl1");
        lift = hardwareMap.get(DcMotor.class, "l1");
        lift2 = hardwareMap.get(DcMotor.class, "l2");
        man = hardwareMap.get(DcMotor.class, "m");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        lift.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        klesh.setDirection(Servo.Direction.FORWARD);
        klesh1.setDirection(Servo.Direction.REVERSE);
        man.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    public void delay(long millis){
        try{
            Thread.sleep(millis);
        } catch (InterruptedException ex){
            Thread.currentThread().interrupt();
        }
    }

}
