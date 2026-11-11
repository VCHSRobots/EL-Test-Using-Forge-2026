package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.forge.Identity;
import frc.robot.hardware.KnownUnits;
import frc.robot.hardware.Unit;
import org.junit.jupiter.api.Test;

class IdentityTest {
  @Test
  void theBuildIdentityIsPublishedUnderForge() {
    NetworkTableInstance nt = NetworkTableInstance.create();
    try {
      Identity.publish(nt, "simulation", KnownUnits.SIMULATION);
      assertEquals(BuildConstants.COMMIT, nt.getEntry("/Forge/Build/Commit").getString("?"));
      assertEquals(BuildConstants.PROGRAM, nt.getEntry("/Forge/Build/Program").getString("?"));
      assertEquals(BuildConstants.DIRTY, nt.getEntry("/Forge/Build/Dirty").getBoolean(!BuildConstants.DIRTY));
      assertEquals("simulation", nt.getEntry("/Forge/Serial").getString("?"));
      assertEquals("simulation", nt.getEntry("/Forge/Unit").getString("?"));
    } finally {
      nt.close();
    }
  }

  @Test
  void theSerialSelectsTheUnit() {
    assertTrue(KnownUnits.forSerial("simulation").known());
    Unit stand = KnownUnits.forSerial("031be686");
    assertTrue(stand.known());
    assertEquals("elevator-stand", stand.name());
    Unit stranger = KnownUnits.forSerial("0000BEEF");
    assertFalse(stranger.known());
    assertEquals("0000BEEF", stranger.serial());
    assertTrue(Identity.json("0000BEEF", stranger).contains("\"serial\":\"0000BEEF\""));
  }

  @Test
  void theBuildIdentityIsWritten() {
    assertFalse(BuildConstants.BUILT_AT.isEmpty());
    assertTrue(BuildConstants.PROGRAM.equals("robot") || BuildConstants.PROGRAM.equals("idle"));
  }
}
