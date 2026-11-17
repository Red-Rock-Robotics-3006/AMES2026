// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Seconds;

// import javax.lang.model.util.ElementScanner14;

// import org.littletonrobotics.junction.AutoLogOutputManager;
// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import redrocklib.logging.SmartDashboardNumber;

// public class LED extends SubsystemBase{

//     private static LED instance = null;

//     private AddressableLED control = new AddressableLED(9);//TODO FILLER

//     private AddressableLEDBuffer buffer = new AddressableLEDBuffer(69);//TODO FILLER

//     private final Color WHITE = new Color(255, 255, 255);
//     private final Color BLACK = new Color(0, 0, 0);
//     private final Color GREEN = new Color(0, 255, 0);
//     private final Color BLUE = new Color(0, 0, 255);
//     private final Color RED = new Color(255, 0, 0);
//     private final Color ORANGE = new Color(255, 128, 0);
//     private final LEDPattern ORANGE_BLINK = LEDPattern.solid(ORANGE).blink(Seconds.of(0.3));
//     private final LEDPattern WHITE_BLINK = LEDPattern.solid(WHITE).blink(Seconds.of(0.3));

//     SmartDashboardNumber blinkFreq = new SmartDashboardNumber("LED/LoopFreq", 90);

//     int loopControl = 0;
    
//     private LED(){
//         super("LED");
        
//         AutoLogOutputManager.addObject(this);
//         control.setLength(buffer.getLength());
//         control.setColorOrder(AddressableLED.ColorOrder.kRGB);
//         control.setData(buffer);
//         control.start();
//     }

//     public void setLights(Color color){
//         for(int i = 0; i < buffer.getLength(); i++){
//             buffer.setLED(i, color);
//         }
//     }

//     public void setLights(Color col1, Color col2, Color col3){
//         for(int i = 0; i < buffer.getLength()/2; i+=1){
//             if(i % 3 == 0){
//                 buffer.setLED(2*i, col1);
//                 buffer.setLED(2*i+1, col1);
//             }
//             else if(i % 3 == 1){
//                 buffer.setLED(2*i, col2);
//                 buffer.setLED(2*i+1, col2);
//             }
//             else{
//                 buffer.setLED(2*i, col3);
//                 buffer.setLED(2*i+1, col3);
//             };
//         }
//     }


//     public void blink(Color col1, Color col2, int freq){
//         int fullCycle = freq * 2;
//         int segment = loopControl / fullCycle;
//         int cycle = loopControl % fullCycle;

//         double brightness = (cycle < freq)
//             ? (double) cycle / freq
//             : (double) (fullCycle - cycle) / freq;

//         Color active = (segment % 2 == 0) ? col1 : col2;

//         setLights(new Color(
//             active.red * brightness,
//             active.green * brightness,
//             active.blue * brightness
//         ));
//     }

//     public void blinkSolid(Color col1, Color col2, int freq){
//         if(loopControl%freq*2 < freq){
//             setLights(col1);
//             return;
//         }
//         setLights(col2);
//     }

//     public void scrollLights(Color col1, Color col2, double speed){

//     }


//     @Override
//     public void periodic(){
//         loopControl++;
//         this.control.setData(buffer);
//     }

//     public static LED getInstance(){
//         if(LED.instance == null) 
//             LED.instance = new LED();
//         return LED.instance;
//     }
// }