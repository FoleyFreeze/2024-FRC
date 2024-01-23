package frc.robot.subsystems.drive;

import frc.robot.Robot;
import frc.robot.cals.DriveCals.WheelCal;

public class Wheel {

    WheelCal k;

    private final WheelIO io;
    private final WheelIOInputsAutoLogged inputs = new WheelIOInputsAutoLogged();

    public Wheel(){
        io = new WheelIO() {};
    }
}
