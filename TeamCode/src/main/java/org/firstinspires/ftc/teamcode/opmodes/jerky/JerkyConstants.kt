package org.firstinspires.ftc.teamcode.opmodes.jerky
import com.acmerobotics.dashboard.config.Config
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.utils.DriveValues

@Config
class JerkyConstants {
  companion object {
    @JvmField var yP = 0.0001
    @JvmField var yS = 0.05
    @JvmField var yV = 0.01
    @JvmField var yA = 0.0015

    @JvmField var xP = 0.0001
    @JvmField var xS = 0.1
    @JvmField var xV = 0.013
    @JvmField var xA = 0.002

    @JvmField var rP = 0.0001
    @JvmField var rS = 0.1
    @JvmField var rV = 0.015
    @JvmField var rA = 0.001

    @JvmField var fl = 1.0
    @JvmField var fr = 1.0
    @JvmField var bl = 1.1
    @JvmField var br = 1.1
    
    @JvmField var maxAccel = 100.0
    @JvmField var maxVel = 80.0

    @JvmField var trackRadius = 9.528 * 0.5 

    val forwardPSVA: PSVAConstants
      get() = PSVAConstants(yP, yS, yV, yA)
    val strafePSVA: PSVAConstants
      get() = PSVAConstants(xP, xS, xV, xA)
    val turnPSVA: PSVAConstants
      get() = PSVAConstants(rP, rS, rV, rA)

    val scalar: DriveValues
      get() = DriveValues(fl, fr, bl, br)

    val constants: DriveConstants 
      get() = JerkyConstants.run { DriveConstants(maxVel, maxAccel, trackRadius, forwardPSVA, strafePSVA, turnPSVA, scalar) }
  }
}
