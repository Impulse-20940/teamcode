package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.superTeleOp;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot R = new Robot();
        superTeleOp tel = new superTeleOp();
        tel.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);
        R.init_classes(hardwareMap, telemetry, gamepad1, gamepad2, this);

    }
}
