package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.utilities.PicoColorSensor;
import frc.robot.utilities.PicoColorSensor.RawColor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants.kSensors;
import frc.robot.Constants.GamePiece;

public class Sensors extends SubsystemBase{
    private boolean enableLeds;
    private boolean enableColor;
    private boolean enableThroughBore;

    private AddressableLED m_LedStrip;
    private AddressableLEDBuffer m_LedData;

    private PicoColorSensor m_colorSensor;

    private Encoder m_relativeEncoder;
    private DutyCycleEncoder m_absEncoder;

    public Sensors(boolean LEDs, boolean ColorSensor, boolean ThroughBore){
        enableLeds = LEDs;
        enableColor = ColorSensor;
        enableThroughBore = ThroughBore;

        if (enableLeds){
            m_LedStrip = new AddressableLED(kSensors.ledPort);
            m_LedData = new AddressableLEDBuffer(kSensors.ledLength);
            m_LedStrip.setData(m_LedData);
            m_LedStrip.start();
        }
        if (enableColor){
            m_colorSensor = new PicoColorSensor();
            m_colorSensor.setDebugPrints(false);
        }
        if (enableThroughBore){
            m_relativeEncoder = new Encoder(kSensors.encoderAPort, kSensors.encoderBPort);
            m_absEncoder = new DutyCycleEncoder(kSensors.encoderAbsPort);

            m_relativeEncoder.setDistancePerPulse(kSensors.distancePerPulse);
            m_absEncoder.setDutyCycleRange(1, 1024);
            m_absEncoder.setDistancePerRotation(kSensors.distancePerRotation);
        }
    }

    public RawColor getRawColor(){
        if(!enableColor)  return null;
        return m_colorSensor.getRawColor0();
    }

    public int getProximity(){
        if(!enableColor) return -1;
        return m_colorSensor.getProximity0();
    }

    public boolean getConnected(){
        return m_colorSensor.isSensor0Connected();
    }

    public GamePiece getGamePiece(){
        int proximity = m_colorSensor.getProximity0();
        RawColor color = m_colorSensor.getRawColor0();

        if(proximity > kSensors.proximityThreshold){
            double colorRatio = (double)color.blue/(double)color.green;
            if(4 < colorRatio && colorRatio < 8) return GamePiece.CONE;
            else if(0 < colorRatio && colorRatio < 2) return GamePiece.KUBE;
        }
        return GamePiece.NONE;
    }
}
