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

@Autonomous(name = "Auto Lejos", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    // Variable para saber de qué lado empezamos
    public boolean esAzul = true;

    // Variables para el control de tiempo (Waits)
    private long waitTimer;
    private final long WAIT_TIME = 750; // Tiempo de espera en milisegundos

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        panelsTelemetry.debug("Status", "Iniciando... ¡Presiona X (Azul) o B (Rojo)!");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Leemos el control 1 mientras esperamos a darle Play
        if (gamepad1.x) {
            esAzul = true;
        } else if (gamepad1.b) {
            esAzul = false;
        }

        // Le avisamos al driver en la pantalla para no regarla en el match
        panelsTelemetry.debug("Azul = X Roja = B");
        panelsTelemetry.debug("ALIANZA SELECCIONADA: ", esAzul ? "AZUL (Boton X)" : "ROJA (Boton B)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Hasta que le damos Play, configuramos la pose inicial y armamos la ruta
        // Usamos nuestro método "e" (espejo) para voltear la pose inicial si es necesario
        Pose startPoseAzul = new Pose(60, 8, Math.toRadians(90));

        // Magia para voltear el heading en la pose inicial
        double startHeading = esAzul ? startPoseAzul.getHeading() : -startPoseAzul.getHeading();
        double startY = esAzul ? startPoseAzul.getY() : 144.0 - startPoseAzul.getY();

        follower.setStartingPose(new Pose(startPoseAzul.getX(), startY, startHeading));

        // Le pasamos la bandera "esAzul" para que construya el lado correcto
        paths = new Paths(follower, esAzul);
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
                    // Guardamos en la Black Box para el TeleOp
                    BlackBox.currentPose = follower.getPose();
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }

    public static class Paths {
        public PathChain chain1, chain2, chain3, chain4, chain5;

        private Pose e(Pose p, boolean azul) {
            if (azul) return p;
            return new Pose(p.getX(), 144.0 - p.getY());
        }

        private double eAng(double ang, boolean azul) {
            if (azul) return ang;
            return -ang;
        }

        public Paths(Follower follower, boolean esAzul) {
            // BLOQUE 1
            chain1 = follower.pathBuilder()
                    .addPath(new BezierLine(e(new Pose(60.0, 8.0), esAzul), e(new Pose(60.0, 12.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(90), esAzul), eAng(Math.toRadians(119), esAzul))
                    .build();

            // BLOQUE 2
            chain2 = follower.pathBuilder()
                    .addPath(new BezierCurve(e(new Pose(60.0, 12.0), esAzul), e(new Pose(59.4, 26.9), esAzul), e(new Pose(50.9, 34.6), esAzul), e(new Pose(42.0, 36.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(119), esAzul), eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierLine(e(new Pose(42.0, 36.0), esAzul), e(new Pose(12.0, 36.0), esAzul)))
                    .setConstantHeadingInterpolation(eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierCurve(e(new Pose(12.0, 36.0), esAzul), e(new Pose(37.3, 30.8), esAzul), e(new Pose(54.7, 25.2), esAzul), e(new Pose(60.0, 12.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(180), esAzul), eAng(Math.toRadians(119), esAzul))
                    .build();

            // BLOQUE 3
            chain3 = follower.pathBuilder()
                    .addPath(new BezierCurve(e(new Pose(60.0, 12.0), esAzul), e(new Pose(59.3, 35.5), esAzul), e(new Pose(55.4, 60.8), esAzul), e(new Pose(42.0, 60.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(119), esAzul), eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierLine(e(new Pose(42.0, 60.0), esAzul), e(new Pose(16.0, 60.0), esAzul)))
                    .setConstantHeadingInterpolation(eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierCurve(e(new Pose(16.0, 60.0), esAzul), e(new Pose(14.9, 63.9), esAzul), e(new Pose(11.5, 63.5), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(180), esAzul), eAng(Math.toRadians(160), esAzul))
                    .addPath(new BezierCurve(e(new Pose(11.5, 63.5), esAzul), e(new Pose(33.2, 56.5), esAzul), e(new Pose(43.6, 33.0), esAzul), e(new Pose(60.0, 12.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(160), esAzul), eAng(Math.toRadians(119), esAzul))
                    .build();

            // BLOQUE 4
            chain4 = follower.pathBuilder()
                    .addPath(new BezierCurve(e(new Pose(60.0, 12.0), esAzul), e(new Pose(45.1, 15.5), esAzul), e(new Pose(29.5, 9.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(119), esAzul), eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierLine(e(new Pose(29.5, 9.0), esAzul), e(new Pose(8.0, 9.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(180), esAzul), eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierCurve(e(new Pose(8.0, 9.0), esAzul), e(new Pose(44.2, 16.3), esAzul), e(new Pose(60.0, 12.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(180), esAzul), eAng(Math.toRadians(119), esAzul))
                    .build();

            // BLOQUE 5
            chain5 = follower.pathBuilder()
                    .addPath(new BezierLine(e(new Pose(60.0, 12.0), esAzul), e(new Pose(30.0, 9.0), esAzul)))
                    .setLinearHeadingInterpolation(eAng(Math.toRadians(119), esAzul), eAng(Math.toRadians(180), esAzul))
                    .addPath(new BezierLine(e(new Pose(30.0, 9.0), esAzul), e(new Pose(8.0, 9.0), esAzul)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}