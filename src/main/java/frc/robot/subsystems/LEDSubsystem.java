package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  public enum FixedPalettePattern {
    Rainbow_Rainbow(-.99),
    Rainbow_Party(-.97),
    Rainbow_Ocean(-.95),
    Rainbow_Lava(-.93),
    Rainbow_Forest(-.91),
    Rainbow_Glitter(-0.89),
    Confetti(0.87),
    Shot_Red(-0.85),
    Shot_Blue(-0.83),
    Shot_White(-0.81),
    Sinelon_Rainbow(-0.79),
    Sinelon_Party(-0.77),
    Sinelon_Ocean(-0.75),
    Sinelon_Lava(-0.73),
    Sinelon_Forest(-0.71),
    BPM_Rainbow(-0.69),
    BPM_Party(-0.67),
    BPM_Ocean(-0.65),
    BPM_Lava(-0.63),
    BPM_Forest(-0.61),
    Fire_Medium(-0.59),
    Fire_Large(-0.57),
    Twinkles_Rainbow_Palette(-.55),
    Twinkles_Party_Palette(-.53),
    Twinkles_Ocean_Palette(-.51),
    Twinkles_Lava_Palette(-.49),
    Twinkles_Forest_Palette(-.47),
    Color_Waves_Rainow_Palette(-.45),
    Color_Waves_Party_Palette(-.43),
    Color_Wavs_Ocean_Palette(-.41),
    Color_Waves_Lava_Palette(-.39),
    Color_Waves_Forest_Palette(-.37),
    Larson_Scanner_Red(-.35),
    Larson_Scanner_Gray(-.33),
    Light_Chase_Red(-.31),
    Light_Chase_Blue(-.29),
    Light_Chase_Gray(-.27),
    Heartbeat_Red(-.25),
    Heartbeat_Blue(-.23),
    Heartbeat_White(-.21),
    Heartbeat_Gray(-.19),
    Breath_Red(-.17),
    Breath_Blue(-.15),
    Breach_Gray(-.13),
    Strobe_Red(-.11),
    Strobe_Blue(-.09),
    Strobe_Gold(-.07),
    Strobe_White(-.05);

    private double value;

    FixedPalettePattern(double value) {
      this.value = value;
    }
  }

  public enum Color1Pattern {
    End_to_End_Blend_to_Black(-.03),
    Larson_Scanner(-.01),
    Light_CHase(.01),
    Heartbeat_Slow(.03),
    Heartbeat_Medium(.05),
    Heartbeat_Fast(.07),
    Breath_Slow(.09),
    Breath_Fast(.11),
    Shot(.13),
    Strobe(.15);

    private double value;

    Color1Pattern(double value) {
      this.value = value;
    }
  }

  public enum Color2Pattern {
    End_to_End_Blend_to_Black(.17),
    Larson_Scanner(.19),
    Light_CHase(.21),
    Heartbeat_Slow(.23),
    Heartbeat_Medium(.25),
    Heartbeat_Fast(.27),
    Breath_Slow(.29),
    Breath_Fast(.31),
    Shot(.33),
    Strobe(.35);

    private double value;

    Color2Pattern(double value) {
      this.value = value;
    }
  }

  public enum Color1and2Pattern {
    Sparkle_Color1_on_Color2(.37),
    Sparkle_Color2_on_Color1(.39),
    Color_Gradient_Color1_and_2(.41),
    Beats_per_Minute_Color1_and_2(.43),
    End_to_End_Blend_Color1_to_2(.45),
    End_to_End_Blend(.45),
    Color1_and_Color2_no_blending(.49),
    Twinkles_Color1_and_2(.51),
    Color_Waves_Color1_and_2(.53),
    Sinelon_Color1_and_2(.55);

    public double value;

    Color1and2Pattern(double value) {
      this.value = value;
    }
  }

  public enum SolidColors {
    Hot_Pink(.57),
    Dark_Red(.59),
    Red(.61),
    Red_Orange(.63),
    Orange(.65),
    Gold(.67),
    Yellow(.69),
    Lawn_Green(.71),
    Lime(.73),
    Dark_Green(.75),
    Green(.77),
    Blue_Green(.79),
    Aqua(.81),
    Sky_Blue(.83),
    Dark_Blue(.85),
    Blue(.87),
    Blue_Violet(.89),
    Violet(.91),
    White(.93),
    Gray(.95),
    Dark_Gray(.97),
    Black(.99);

    public double value;

    SolidColors(double value) {
      this.value = value;
    }
  }

  public Spark m_led = new Spark(0);

  public Command setFixedPalletePattern(FixedPalettePattern p) {
    return run(() -> m_led.set(p.value));
  }

  public Command setColor1Pattern(Color1Pattern c1p) {
    return run(() -> m_led.set(c1p.value));
  }

  public Command setColor2Pattern(Color2Pattern c2p) {
    return run(() -> m_led.set(c2p.value));
  }

  public Command setColor1and2Pattern(Color1and2Pattern c1a2p) {
    return run(() -> m_led.set(c1a2p.value));
  }

  public Command setSolidColors(SolidColors sc) {
    return run(() -> m_led.set(sc.value));
  }
}
