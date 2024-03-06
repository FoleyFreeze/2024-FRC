package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.FileManager;

public class DriveIO_HW implements DriveIO{
    
    AHRS navX;
    FileManager fm;

    public DriveIO_HW (){
        navX = new AHRS(SPI.Port.kMXP, (byte) 200);
        fm = new FileManager("/home/lvuser/WheelEncoderOffsets.txt");
    }

    @Override
    public void updateInputs (DriveIOInputs inputs){
        inputs.navXconnected = navX.isConnected();
        inputs.yaw = new Rotation2d(MathUtil.angleModulus(Math.toRadians(-navX.getYaw())));
        inputs.angle = new Rotation2d(MathUtil.angleModulus(Math.toRadians(-navX.getAngle())));
        inputs.fusedHeading = new Rotation2d(MathUtil.angleModulus(Math.toRadians(-navX.getFusedHeading())));
        inputs.pitch = Rotation2d.fromDegrees(-navX.getPitch());
        inputs.roll = Rotation2d.fromDegrees(-navX.getRoll());
        inputs.yawVelocity = Units.degreesToRadians(-navX.getRate());        
    }

    @Override
    public void updateFileInputs(FileIOInputs inputs) {
        inputs.rotationOffsets = new double[4];

        try{
            if(fm.exists()){
                System.out.println("Reading wheel positions file:");
                for(int i=0;i<4;i++) {
                    double offset = Double.parseDouble(fm.readLine());
                    System.out.print(offset + " ");
                    inputs.rotationOffsets[i] = offset;
                }
                fm.close();
                System.out.println();
            }
            
        }catch(Exception e){
            System.out.println(e.toString());
            e.printStackTrace();
            //if there was an error, reset to cal value
            System.out.println("Error reading file, defaulting to 0");
            for(int i = 0; i < 4; i++) {
                //double v = wheels[i].absEncoder.getVoltage();
                inputs.rotationOffsets[i] = 0;
                //System.out.println(k.wheelCals[i].name + "  :)  " + v);
            }
        }
    }

    @Override
    public void writeOffsets(double offsets[]) {
        try {
            for (double num:offsets){
               fm.writeLine(Double.toString(num));
            }
            fm.close();
        }catch(Exception e){
            System.out.println("Error while saving swerve offsets to file");
            e.printStackTrace();
        }
    }
}
