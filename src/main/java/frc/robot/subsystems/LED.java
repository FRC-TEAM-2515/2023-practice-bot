/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class LED extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public LED(){
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  public void setYellowLED(){
    for (var i=0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 255, 255, 0);
    }
    setData();
  }

  public void setPurpleLED(){
    for (var i=0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, 216, 191, 216);
    }    
    setData();
  }

  public void setData(){
    m_led.setData(m_ledBuffer);
  }
  public void stop(){
    m_led.stop();
  }

  public void rainbow() {
    // For every pixel
    int m_rainbowFirstPixelHue=0;
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
