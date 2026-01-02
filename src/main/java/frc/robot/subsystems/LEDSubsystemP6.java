package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.utils.Types.LEDState;

public class LEDSubsystemP6 extends SubsystemBase {
    private CANdle candle;
    private CANdleConfiguration cfg;

    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    private static final int kSlot0StartIdx = 0; // 8;
    private static final int kSlot0EndIdx = LEDConstants.LED_Count;

    // private static final int kSlot1StartIdx = 38;
    // private static final int kSlot1EndIdx = 67;
    private RGBWColor currentAlliance = new RGBWColor(Color.kRed);

    public enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    public LEDSubsystemP6() {
        candle = new CANdle(LEDConstants.CANdleID, "rio");
        cfg = new CANdleConfiguration();

        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 1;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

        candle.getConfigurator().apply(cfg);
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        setLEDOff();
    }

    public void setAllLEDColor(RGBWColor color) {

        candle.setControl(
                new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(color));
    }

    public void setLEDOff() {

        candle.setControl(
                new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(new RGBWColor(Color.kBlack)));
    }

    public void animate(AnimationType toAnimate, RGBWColor color) {
        switch (toAnimate) {
            case ColorFlow:
                candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            case Fire:
                candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
                break;
            case Larson:
                candle.setControl(
                        new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            case Rainbow:
                candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
                break;
            case RgbFade:
                candle.setControl(
                        new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
                break;
            case SingleFade:
                candle.setControl(
                        new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            case Strobe:
                candle.setControl(
                        new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            case Twinkle:
                candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            case TwinkleOff:
                candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(color));
                break;
            default:
                setLEDOff();
                break;
        }
    }

    public void SetLEDState(LEDState state) {
        switch (state) {
            case IN_RANGE:
            case SEE_TAG:
                setAllLEDColor(new RGBWColor(Color.kGreen));
                break;
            case HAVE_PIECE:
                setLEDOff();
                setAllLEDColor(new RGBWColor(Color.kPurple));
                break;
            case NO_PIECE:
            case NONE:
                setAllLEDColor(currentAlliance);
                break;
        }

    }

    @Override
    public void periodic() {
        var alliance = DriverStation.getAlliance();
        var lastColor = currentAlliance;
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                currentAlliance = new RGBWColor(Color.kBlue);
            } else if (alliance.get() == DriverStation.Alliance.Red) {
                currentAlliance = new RGBWColor(Color.kRed);
            } else {
                currentAlliance = new RGBWColor(Color.kMaroon);
            }
        } else {
            currentAlliance = new RGBWColor(Color.kMaroon);
        }

        // if the alliance color changed, update it.
        if (!lastColor.equals(currentAlliance)) {
            setAllLEDColor(currentAlliance);
        }
    }

    public Command SetLEDOffInstantCommand() {
        return this.runOnce(() -> setLEDOff());
    }

    public Command SetLEDOn() {
        return new FunctionalCommand(
                () -> System.out.println("LED on"),
                () -> {
                    setAllLEDColor(new RGBWColor(Color.kRed));
                },
                (interrupted) -> setLEDOff(),
                () -> false, this);
    }

    public Command LedAnimate(AnimationType toAnimate, RGBWColor color) {
        return new FunctionalCommand(
                () -> System.out.println("Animate Start"),
                () -> animate(toAnimate, color),
                (interrupted) -> setLEDOff(),
                () -> false, this);
    }
}