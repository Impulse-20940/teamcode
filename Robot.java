package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Robot{
    //DcMotor lift = null;
    //DcMotor manipulator = null;
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
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.L = L;
    }
    public void get_members() {
        //lift = hardwareMap.get(DcMotor.class, "reechniy_lift");
        //manipulator = hardwareMap.get(DcMotor.class, "manipulator");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        //lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //manipulator.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void go_bytime(double axial, double lateral, double yaw, double time) {
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

    public void go_byenc_right(double ticks) {
        while ((rightFrontDrive.getCurrentPosition() < ticks) | (Math.abs(rightBackDrive.getCurrentPosition()) < ticks)) {
            double axial = 0;
            double lateral = 1;
            double yaw = 0;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            /*
            if ((rightFrontDrive.getCurrentPosition() - rightBackDrive.getCurrentPosition()) > 300) {
                while (rightFrontDrive.getCurrentPosition() > rightBackDrive.getCurrentPosition()) {
                    axial = 0;
                    lateral = 0;
                    yaw = -1;
                }
            }
            if ((rightBackDrive.getCurrentPosition() - rightFrontDrive.getCurrentPosition()) > 300) {
                while (rightBackDrive.getCurrentPosition() > rightFrontDrive.getCurrentPosition()) {
                    axial = 0;
                    lateral = 0;
                    yaw = 1;
                }
            }
             */
            telemetry.addData("Now is", "%7d :%7d",
                    rightFrontDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
