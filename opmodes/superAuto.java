package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lag.Lift_and_grab;
import org.firstinspires.ftc.teamcode.wheelbase.Regualizers;

public class superAuto extends Robot {
    Regualizers reg = new Regualizers();
    Lift_and_grab lag = new Lift_and_grab();
    void get_sample(){
        get_members();
        reg.go_byenc_y(0, -270, 0.0012);
        klesh.setPosition(1);
        lag.k_up(-0.45, 500);
        lag.lift_up(0.55, 1800);
        lag.k_up(-0.45, 500);
    }
    public void auto_human(int state){
        get_members();
        if (state == 0){
            double stop = 0;
            while(stop != 1 && !gamepad1.x){
                reg.stable(0, 2, 0.012);
                reg.go_byenc_y(0, 430, 0.0012);
                reg.go_byenc_x(0, 600);
                stop = 1;
            }
        }
        if (state == 1){
            get_members();
            double stop = 0;
            while (stop != 1 && !gamepad1.x) {
                reg.stable(-90, 2, 0.012);
                reg.go_byenc_x(-90, -800);
                delay(100);
                reg.go_byenc_y(-90, 400, 0.0012);
                lag.k_up(-0.55, 1000);
                lag.lift_up(0.55, 1800);
                stop = 1;
            }
        }
    }
    public void auto_app(int state){
        get_members();
        if (state == 0){

        }
        if (state == 1){

        }
    }
}
