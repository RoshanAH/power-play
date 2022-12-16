package org.firstinspires.ftc.teamcode.opmodes.jerky
import com.acmerobotics.dashboard.config.Config

@Config
class PSVA {
  companion object {
    @JvmField var yP = 0.0
    @JvmField var yS = 0.0
    @JvmField var yV = 0.0
    @JvmField var yA = 0.0

    @JvmField var xP = 0.0
    @JvmField var xS = 0.0
    @JvmField var xV = 0.0
    @JvmField var xA = 0.0

    @JvmField var rP = 0.0
    @JvmField var rS = 0.0
    @JvmField var rV = 0.0
    @JvmField var rA = 0.0

    @JvmField var fl = 1.0
    @JvmField var fr = 1.0
    @JvmField var bl = 1.0
    @JvmField var br = 1.0
  }
}
