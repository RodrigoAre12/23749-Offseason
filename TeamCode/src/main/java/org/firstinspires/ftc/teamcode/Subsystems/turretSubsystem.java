package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class turretSubsystem extends SubsystemBase {

    DcMotorEx TurretMotor;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    PIDFCoefficients turretPID;

    public turretSubsystem(Telemetry telemetry, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        turretPID = new PIDFCoefficients(1,0.00,0,5);

        TurretMotor = hardwareMap.get(DcMotorEx.class, "shooterIzq");
        TurretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, turretPID);
        TurretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        TurretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

}