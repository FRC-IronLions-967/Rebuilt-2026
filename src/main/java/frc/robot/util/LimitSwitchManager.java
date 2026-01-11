// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Mode;
import edu.wpi.first.wpilibj.SPI.Port;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class LimitSwitchManager {

  private SPI port;
  private byte[] latestBytes = new byte[2];
  private static Boolean[] switchStates = new Boolean[16];
  private final int kBoardSize = 16;

  public LimitSwitchManager() {
    port = new SPI(Port.kOnboardCS0);
    port.setClockRate(500000);
    port.setMode(Mode.kMode0);
    port.setChipSelectActiveHigh();
  }

  public static BooleanSupplier getSwitch(int port) {
    return (() -> switchStates[port]);
  }

  public void periodic() {
    port.read(true, latestBytes, kBoardSize / 8);
    for (int i = 0; i < kBoardSize / 8; i++) {
      switchStates[i * 8] = ((latestBytes[i] & 0x01) != 0);
      switchStates[i * 8 + 1] = ((latestBytes[i] & 0x02) != 0);
      switchStates[i * 8 + 2] = ((latestBytes[i] & 0x04) != 0);
      switchStates[i * 8 + 3] = ((latestBytes[i] & 0x08) != 0);
      switchStates[i * 8 + 4] = ((latestBytes[i] & 0x10) != 0);
      switchStates[i * 8 + 5] = ((latestBytes[i] & 0x20) != 0);
      switchStates[i * 8 + 6] = ((latestBytes[i] & 0x40) != 0);
      switchStates[i * 8 + 7] = ((latestBytes[i] & 0x80) != 0);
    }
  }

  public void simulationPeriodic() {
    // TODO add simulation support
  }
}
