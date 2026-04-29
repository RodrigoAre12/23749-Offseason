package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing con Waits", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Variables para el control de tiempo (Waits)
    private long waitTimer;
    private final long WAIT_TIME = 750; // Tiempo de espera en milisegundos

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Posición inicial (debe coincidir con el inicio del Path 1)
        follower.setStartingPose(new Pose(60, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "¡Robot Listo!");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Estado Actual", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Ejecuta Path 1 solo
                follower.followPath(paths.chain1);
                pathState = 1;
                break;

            case 1: // ¿Terminó Path 1? -> Espera 750ms
                if (!follower.isBusy()) {
                    waitTimer = System.currentTimeMillis();
                    pathState = 2;
                }
                break;

            case 2: // Espera entre 1 y 2
                if (System.currentTimeMillis() - waitTimer > WAIT_TIME) {
                    follower.followPath(paths.chain2); // Arranca 2, 3 y 4 seguidos
                    pathState = 3;
                }
                break;

            case 3: // ¿Terminó Path 4? -> Espera 750ms
                if (!follower.isBusy()) {
                    waitTimer = System.currentTimeMillis();
                    pathState = 4;
                }
                break;

            case 4: // Espera entre 4 y 5
                if (System.currentTimeMillis() - waitTimer > WAIT_TIME) {
                    follower.followPath(paths.chain3); // Arranca 5, 6, 7 y 8 seguidos
                    pathState = 5;
                }
                break;

            case 5: // ¿Terminó Path 8? -> Espera 750ms
                if (!follower.isBusy()) {
                    waitTimer = System.currentTimeMillis();
                    pathState = 6;
                }
                break;

            case 6: // Espera entre 8 y 9
                if (System.currentTimeMillis() - waitTimer > WAIT_TIME) {
                    follower.followPath(paths.chain4); // Arranca 9, 10 y 11 seguidos
                    pathState = 7;
                }
                break;

            case 7: // ¿Terminó Path 11? -> Espera 750ms
                if (!follower.isBusy()) {
                    waitTimer = System.currentTimeMillis();
                    pathState = 8;
                }
                break;

            case 8: // Espera entre 11 y 12
                if (System.currentTimeMillis() - waitTimer > WAIT_TIME) {
                    follower.followPath(paths.chain5); // Arranca 12 y 13 seguidos
                    pathState = 9;
                }
                break;

            case 9: // Fin de todo
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }

    public static class Paths {
        public PathChain chain1, chain2, chain3, chain4, chain5;

        public Paths(Follower follower) {
            // BLOQUE 1: Solo Path 1
            chain1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.0, 8.0), new Pose(60.0, 12.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(119))
                    .build();

            // BLOQUE 2: Del 2 al 4 (Fluyen sin pausa)
            chain2 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(60.0, 12.0), new Pose(59.4, 26.9), new Pose(50.9, 34.6), new Pose(42.0, 36.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(119), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(42.0, 36.0), new Pose(12.0, 36.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(new BezierCurve(new Pose(12.0, 36.0), new Pose(37.3, 30.8), new Pose(54.7, 25.2), new Pose(60.0, 12.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(119))
                    .build();

            // BLOQUE 3: Del 5 al 8 (Fluyen sin pausa)
            chain3 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(60.0, 12.0), new Pose(59.3, 35.5), new Pose(55.4, 60.8), new Pose(42.0, 60.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(119), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(42.0, 60.0), new Pose(16.0, 60.0)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(new BezierCurve(new Pose(16.0, 60.0), new Pose(14.9, 63.9), new Pose(11.5, 63.5)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .addPath(new BezierCurve(new Pose(11.5, 63.5), new Pose(33.2, 56.5), new Pose(43.6, 33.0), new Pose(60.0, 12.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(119))
                    .build();

            // BLOQUE 4: Del 9 al 11 (Fluyen sin pausa)
            chain4 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(60.0, 12.0), new Pose(45.1, 15.5), new Pose(29.5, 9.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(119), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(29.5, 9.0), new Pose(8.0, 9.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(new BezierCurve(new Pose(8.0, 9.0), new Pose(44.2, 16.3), new Pose(60.0, 12.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(119))
                    .build();

            // BLOQUE 5: Del 12 al 13
            chain5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.0, 12.0), new Pose(30.0, 9.0)))
                    .setLinearHeadingInterpolation(Math.toRadians(119), Math.toRadians(180))
                    .addPath(new BezierLine(new Pose(30.0, 9.0), new Pose(8.0, 9.0)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}