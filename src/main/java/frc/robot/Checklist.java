package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum Checklist {
    limelightFrontConnected,
    limelightLeftConnected,
    limelightRightConnected,
    MotorsOK,
    shooterMotorsOK,
    odoReady,
    odoAccurate,
    allianceGot;
    // autoChosen;

    private boolean val;
    private BooleanSupplier condition = () -> this.val;

    private static final Checklist[] ITEMS = values();

    private Checklist() {
        val = false;
    }

    public boolean get() {
        return val;
    }

    public Checklist set(boolean val) {
        this.val = val;
        return this;
    }

    public Checklist set(BooleanSupplier condition) {
        this.condition = condition;
        return this;
    }

    public static void updateAll() {
        for (var item : ITEMS) {
            item.val = item.condition.getAsBoolean();
        }
    }

    public static boolean isAllPrepared() {
        for (var check : ITEMS) {
            if (check.val == false)
                return false;
        }
        return true;
    }

    public static void putAll() {
        for (var check : ITEMS) {
            check.val = check.condition.getAsBoolean();
            SmartDashboard.putBoolean(check.name(), check.val);
        }
        // FIXME 如果这个灯不绿，绝对绝对不能开始比赛！！！
        SmartDashboard.putBoolean("Prepared", isAllPrepared()); 
    }

}
