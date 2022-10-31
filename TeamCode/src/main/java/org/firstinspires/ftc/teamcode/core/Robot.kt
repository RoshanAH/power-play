package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

abstract class Robot {
    val components = mutableListOf<Component>()

    fun addComponents(vararg components: Component){
        this.components.addAll(components);
    }

    abstract fun mapHardware(map: HardwareMap);
}
