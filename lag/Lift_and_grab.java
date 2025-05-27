package org.firstinspires.ftc.teamcode.lag;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.wheelbase.Regualizers;

public class Lift_and_grab extends Robot {
    public void k_up(double power, long time){
        get_members();
        lift2.setPower(power);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(time);
        lift2.setPower(0);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void lift_up(double power, long time){
        get_members();
        lift.setPower(power);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delay(time);
        lift.setPower(0);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
