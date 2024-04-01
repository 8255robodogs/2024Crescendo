package frc.robot.subsystems;

import java.util.Random;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTestSystem extends SubsystemBase {
    PIDController pid = new PIDController(0.05, 0, 0.1);
    Random rand = new Random();
    public double targetValue=0;
    public double currentValue=0;
    public double currentSpeed =0;

    public PIDTestSystem(){

    }

    @Override
    public void periodic() {
        
        //small entropy
        currentValue += rand.nextDouble(0.4)-0.2;
        
        //occasional big bumps of entropy
        if(rand.nextInt(100)== 0){
            currentValue+=rand.nextDouble(5)-2.5;
        }

        //FullAccelerationTowardsTarget(0.1);
        AccelerateTowardsTargetWithBraking(0.1, 0.15);
        //accelerateWithPid();


        currentValue += currentSpeed;
        Logger.recordOutput("currentValue", currentValue);
        Logger.recordOutput("targetValue", targetValue);
        Logger.recordOutput("currentSpeed",currentSpeed);

        super.periodic();
    }

    private void FullAccelerationTowardsTarget(double accelerationRate){
        if(currentValue < targetValue){
            currentSpeed+=accelerationRate;
        }
        if(currentValue > targetValue){
            currentSpeed-=accelerationRate;
        }
    }

    private void AccelerateTowardsTargetWithBraking(double accelerationRate, double brakingRate){
        if(currentValue < targetValue){
            //below the target
            if(currentSpeed >=0)currentSpeed+=accelerationRate;
            if(currentSpeed <=0)currentSpeed+=brakingRate;
        }else{
            //above the target
            if(currentSpeed>=0)currentSpeed-=brakingRate;
            if(currentSpeed<=0)currentSpeed-=accelerationRate;
        }   
    }

    private void accelerateWithPid(){
        double pidOutput=pid.calculate(currentValue,targetValue);
        pidOutput*=0.3;
        if(pidOutput>1)pidOutput=1;
        if(pidOutput<-1)pidOutput=-1;
        Logger.recordOutput("pidOutput",pidOutput);
        currentSpeed += pidOutput;
    }



}
