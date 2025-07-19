package org.firstinspires.ftc.teamcode.programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="test_teleop")
public class test1 extends LinearOpMode {
    DcMotor left_front;
    DcMotor left_back;
    DcMotor right_front;
    DcMotor right_back;
    Servo srv;
    double axial;
    double lateral;
    double yaw;
    boolean servo_sost;
    @Override
    public void runOpMode(){
        left_front = hardwareMap.get(DcMotor.class, "lf");
        left_back = hardwareMap.get(DcMotor.class, "lb");
        right_front = hardwareMap.get(DcMotor.class, "rf");
        right_back = hardwareMap.get(DcMotor.class, "rb");
        srv = hardwareMap.get(Servo.class, "srv");
        Robot R = new Robot();
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                axial = 0.2;
            }
            else if(gamepad1.dpad_down){
                axial = -0.2;
            }
            else if(gamepad1.dpad_left){
                lateral = -0.2;
            }
            else if(gamepad1.dpad_right){
                lateral = 0.2;
            }
            else{
                axial = gamepad1.left_stick_y;
                lateral = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
            }

            if(gamepad1.right_bumper){
                servo_sost = !servo_sost;
                R.delay(50);
            }
            if(servo_sost){
                srv.setPosition(0.5);
            }
            else{
                srv.setPosition(0);
            }

            double left_front_power = axial + lateral + yaw;
            double right_front_power = axial - lateral - yaw;
            double left_back_power = axial + lateral - yaw;
            double right_back_power = axial - lateral + yaw;
            left_front.setPower(left_front_power);
            left_back.setPower(left_back_power);
            right_front.setPower(right_front_power);
            right_back.setPower(right_back_power);
        }
    }
}
