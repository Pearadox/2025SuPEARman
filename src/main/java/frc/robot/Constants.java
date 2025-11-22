// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double LOOP_PERIOD = 0.02; // 20ms
    public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
    public static final double NOMINAL_VOLTAGE = 12;

    public static final class AlignConstants {
        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0);
        public static final double REEF_ALIGN_TZ = Units.inchesToMeters(22);

        public static final Translation2d LEFT_BRANCH_OFFSET = new Translation2d(REEF_ALIGN_TZ, -BRANCH_SPACING);
        public static final Translation2d RIGHT_BRANCH_OFFSET = new Translation2d(REEF_ALIGN_TZ, BRANCH_SPACING);
        public static final Translation2d MID_OFFSET = new Translation2d(REEF_ALIGN_TZ, 0.0);
        public static final Translation2d STATION_OFFSET = new Translation2d(Units.inchesToMeters(18), 0.0);

        public static final double DRIVE_kP = 5.0; // m/s per m error
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;
        public static final double MAX_DRIVE_VELOCITY = 3.0; // m/s
        public static final double MAX_DRIVE_ACCELERATION = 10; // m/s^2

        public static final double ROT_kP = 5.0; // rad/s per rad error
        public static final double ROT_kI = 0.0;
        public static final double ROT_kD = 0.0;
        public static final double MAX_ROT_VELOCITY = 8; // rad/s
        public static final double MAX_ROT_ACCELERATION = 20; // rad/s^2

        public static final double ALIGN_ROT_TOLERANCE = Units.degreesToRadians(3);
        public static final double ALIGN_TRANSLATION_TOLERANCE = Units.inchesToMeters(2);

        public static final double NET_ALIGN_TZ = Units.inchesToMeters(32);
        public static final double MIN_DIST_TO_WALL = Units.inchesToMeters(28);
        public static final double[] NET_RED_Y = {MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH / 2.0 - MIN_DIST_TO_WALL
        };
        public static final double[] NET_BLUE_Y = {
            FieldConstants.FIELD_WIDTH / 2.0 + MIN_DIST_TO_WALL, FieldConstants.FIELD_WIDTH - MIN_DIST_TO_WALL
        };
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
        public static final double FIELD_WIDTH = Units.inchesToMeters(317);

        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
        public static final int[] ALL_REEF_TAG_IDS = {18, 19, 20, 21, 22, 17, 7, 6, 11, 10, 9, 8};

        public static final List<Pose2d> CORAL_STATIONS = new ArrayList<>();

        static {
            for (int tag : BLUE_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
            for (int tag : RED_CORAL_STATION_TAG_IDS) {
                CORAL_STATIONS.add(
                        VisionConstants.aprilTagLayout.getTagPose(tag).get().toPose2d());
            }
        }

        public static final Pose3d[] REEF_TAG_POSES = new Pose3d[RED_REEF_TAG_IDS.length + BLUE_REEF_TAG_IDS.length];

        static {
            int i = 0;
            for (int tag : FieldConstants.RED_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
            for (int tag : FieldConstants.BLUE_REEF_TAG_IDS) {
                REEF_TAG_POSES[i++] =
                        VisionConstants.aprilTagLayout.getTagPose(tag).get();
            }
        }

        public static final List<Pose2d> REEF_TAGS = new ArrayList<>();

        static {
            for (Pose3d tag : REEF_TAG_POSES) {
                REEF_TAGS.add(tag.toPose2d());
            }
        }

        public static final Transform3d HIGH_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(39.575), Rotation3d.kZero);
        public static final Transform3d LOW_ALGAE_TRANSFORM =
                new Transform3d(Units.inchesToMeters(-6), 0, Units.inchesToMeters(23.675), Rotation3d.kZero);

        public static final Pose3d[] REEF_ALGAE_POSES = new Pose3d[REEF_TAG_POSES.length];

        static {
            for (int i = 0; i < REEF_ALGAE_POSES.length; i++) {
                REEF_ALGAE_POSES[i] = REEF_TAG_POSES[i].plus(i % 2 == 0 ? HIGH_ALGAE_TRANSFORM : LOW_ALGAE_TRANSFORM);
            }
        }

        public static final double BARGE_X = FIELD_LENGTH / 2.0;
        public static final double BARGE_WIDTH = Units.inchesToMeters(40) / 2.0;
        public static final double BARGE_HEIGHT = Units.inchesToMeters(74 + 8);
        public static final double BARGE_HEIGHT_TOLERANCE = Units.inchesToMeters(12);

        public static final Pose2d BLUE_PROCESSOR =
                VisionConstants.aprilTagLayout.getTagPose(16).get().toPose2d();
        public static final Pose2d RED_PROCESSOR =
                VisionConstants.aprilTagLayout.getTagPose(3).get().toPose2d();

        public static final double TRANSLATIONAL_TOLERANCE = Units.inchesToMeters(16);
        public static final double DROP_COOLDOWN = 2.0;
    }

    public static final class VisualizerConstants {
        public static final Translation3d STAGE0_ZERO = new Translation3d(-0.26035, 0, 0.32385);
        public static final Translation3d WRIST_ZERO = new Translation3d(0.368313, 0, 0.196875);
        public static final Translation3d WRIST_OFFSET = WRIST_ZERO.minus(STAGE0_ZERO);

        public static final Transform3d CORAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(5.25), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(45), 0)),
                new Rotation3d(0, -Units.degreesToRadians(125), 0));

        public static final Transform3d HORIZONTAL_CORAL_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(10.5), 0, 0)
                        .rotateBy(new Rotation3d(0, -Units.degreesToRadians(96), 0)),
                new Rotation3d(0, 0, Units.degreesToRadians(90)));

        public static final Transform3d ALGAE_TRANSFORM = new Transform3d(
                new Translation3d(Units.inchesToMeters(16), 0, 0).rotateBy(CORAL_TRANSFORM.getRotation()),
                Rotation3d.kZero);

        public static final Translation3d M0_ZERO = new Translation3d(0.0, -0.174625, 0.0);
        public static final Translation3d M1_ZERO = new Translation3d(0.0, -0.071544, 0.368300);
        public static final Translation3d M2_ZERO = new Translation3d(0.0, -0.009525, 0.0);
        public static final Translation3d M3_ZERO = new Translation3d(0.0, 0.136351, 0.193383);
        public static final Translation3d M4_ZERO = new Translation3d(0.0, 0.215676, 0.118053);
        public static final Translation3d M5_ZERO = new Translation3d(0.0, 0.382877, 0.324873);
        public static final Translation3d M5_OFFSET = M5_ZERO.minus(M3_ZERO);
    }
}
