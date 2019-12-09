package org.usfirst.frc.team449.robot.util;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Util {
    private Util() {
    }

    public static double defaultIfNull(@Nullable Double value, double defaultVal) {
        return value == null ? defaultVal : value;
    }

    public static boolean defaultIfNull(@Nullable Boolean value, boolean defaultVal) {
        return value == null ? defaultVal : value;
    }

    @NotNull
    public static <T> T defaultIfNull(@Nullable T value, @NotNull T defaultVal) {
        return value == null ? defaultVal : value;
    }
}
