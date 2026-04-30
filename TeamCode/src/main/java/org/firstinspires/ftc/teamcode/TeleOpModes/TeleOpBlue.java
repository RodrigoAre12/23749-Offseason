package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Pedro Pathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.BlackBox;

// Tus Subsistemas del repositorio
import org.firstinspires.ftc.teamcode.Subsystems.shooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.gateSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.visionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.turretSubsystem;

@TeleOp(name = "TeleOp Pedro Blue", group = "TeleOp")
public class TeleOpBlue extends CommandOpMode {

    private intakeSubsystem intake;
    private visionSubsystem vision;
    private shooterSubsystem shooter;
    private gateSubsystem gate;
    private turretSubsystem turret;

    private Follower follower;
    private GamepadEx chassisDriver, subsystemDriver;

    @Override
    public void initialize() {
        vision = new visionSubsystem(hardwareMap, telemetry);
        shooter = new shooterSubsystem(telemetry, hardwareMap);
        gate = new gateSubsystem(telemetry, hardwareMap);
        intake = new intakeSubsystem(telemetry, hardwareMap);
        turret = new turretSubsystem(telemetry, hardwareMap);

        chassisDriver = new GamepadEx(gamepad1);
        subsystemDriver = new GamepadEx(gamepad2);

        // 2. Inicializar Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // CARGAMOS LA POSICIÓN DEL AUTÓNOMO (La Black Box)
        follower.setStartingPose(BlackBox.currentPose);
        follower.startTeleOpDrive();

        // 3. Drive Command (Movimiento continuo)
        // Usamos RunCommand para que Pedro siempre esté escuchando los joysticks
        schedule(new RunCommand(() -> {
            follower.setTeleOpDrive(
                    -chassisDriver.getLeftY(),  // Adelante/Atrás
                    -chassisDriver.getLeftX(),  // Strafe
                    -chassisDriver.getRightX(), // Giro
                    false                       //Robot Centric
            );
            follower.update(); // Mueve motores y lee Pinpoint
        }));
        
        // Reset de Heading (por si se descalibra)
        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(() -> {
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                }));

        // Intake y Gate

        chassisDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(() -> intake.setPower(1)))
                .whenPressed(gate::close)
                .whenReleased(new InstantCommand(() -> intake.setPower(0)));

        chassisDriver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(() -> intake.setPower(-0.65)))
                .whenReleased(new InstantCommand(() -> intake.setPower(0)));

        // Shooter (Usando las velocidades de tu clase shooterSubsystem)
        chassisDriver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(1150);
                    shooter.setVelLejos(); }));

        chassisDriver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(1000);
                    shooter.setVelMedia(); }));

        chassisDriver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> { shooter.shoot(775);
                    shooter.setVelCerca(); }));

        chassisDriver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> shooter.shoot(0)));

        new Trigger(() -> chassisDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25)
                .whileActiveContinuous(new RunCommand(() -> {
                    if (shooter.rpmReady) {
                        gate.feed();
                        intake.setPower(1);
                    } else {
                        gate.close();
                        intake.setPower(0);
                    }
                }));

        // 5. Telemetría de Pedro
        schedule(new RunCommand(() -> {
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading Deg", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }));
    }
}