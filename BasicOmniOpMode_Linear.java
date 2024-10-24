package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp_newDev")

public class BasicOmniOpMode_Linear extends LinearOpMode {
    //Манипулятор это качелька
    //private Servo klesh = null;
    //private DcMotor lift = null;
    private DcMotor manipulator = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    public void runOpMode() {
        Robot R = new Robot();
        R.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);
        //klesh = hardwareMap.get(Servo.class, "kleahnya")
        //lift = hardwareMap.get(DcMotor.class, "reechniy_lift");
        //manipulator = hardwareMap.get(DcMotor.class, "manipulator");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //manipulator.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            R.teleop();
        }
    }}
