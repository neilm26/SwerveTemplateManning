package frc.robot.Subsystems.SwerveModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveConstructor {

    private SwerveDriveKinematics kinematics;
    private SwerveModule[] modules;
    private Translation2d[] offsets;
    private int moduleCount = 0;
    private Map<Integer, Translation2d> identityMap;


    public SwerveConstructor(Supplier<SwerveModule> moduleSupplier, Tuple2[] offsets) {
        this.moduleCount= offsets.length;

        this.modules = new SwerveModule[moduleCount];
        this.offsets = new Translation2d[moduleCount];
        this.identityMap = new HashMap<Integer, Translation2d>();

        for (int i=0;i<moduleCount;i++) {
            SmartDashboard.putNumber(Integer.toString(i), (Double) offsets[i].get_0());
            this.modules[i] = moduleSupplier.get();

            this.offsets[i] = new Translation2d((Double) offsets[i].get_0(), (Double) offsets[i].get_1());

            this.identityMap.put(i, this.offsets[i]);
        }

        kinematics = new SwerveDriveKinematics(this.offsets);
    }

    public Translation2d[] GetModuleOffsets() {
        return this.offsets;
    }

    public Translation2d GetModuleOffset(int id) {
        return this.identityMap.getOrDefault(id, null);
    }

    public Map<Integer, Translation2d> GetIDMap() {
        return this.identityMap;
    }
}