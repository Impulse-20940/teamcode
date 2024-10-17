package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous_byEnc")
public class Autonomous_byEnc extends LinearOpMode {
    Robot R = new Robot();
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontDrive = null;
    DcMotor leftBackDrive = null;
    DcMotor rightFrontDrive = null;
    DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {
        R.get_members();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Выводим телеметрию с удачным сбросом значений энкодера на консоль Driver Station
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());

        telemetry.update();
        // Wait for the game to start (driver presses START)
        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);

        waitForStart();
        runtime.reset();
        R.go_byenc(1000);
    }
}
