#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <PIDController.h>

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>

ros::NodeHandle nh;

// Message definition
std_msgs::Float32 encMsg;

// Variables
// const uint8_t cpu_freq = 20; //MHz

typedef struct
{
  float P;
  float I;
  float D;
} PID_t;

typedef struct
{
  float a0x0;
  float a1x1;
} MVS_t;

MVS_t mvs[3];
uint32_t ticksLastPID;
int32_t curr_enc;
int32_t last_enc;
float getRPM = 0.;
float setRPM;
float getPWM;
float getPIDVal;
float lpf_factor=.1;
float rpm_lpf;
float pwm_lpf;
volatile uint32_t ticksMostSig;

int32_t EncoderPos;
int8_t mvsBalance0;
int8_t mvsBalance1;

volatile uint8_t cache, cacheOld;

PID_t PID = {0.100,
             5.000,
             0.000};

static PIDController pidCon = PIDController(PID.P, PID.I, PID.D, 127, -127);


// Subscriber Callback
void setSpeedCB(const std_msgs::Float32& msg)
{
    setRPM = msg.data;
}

// Publisher 
ros::Publisher pub("encoderSpeed", &encMsg);

// Subscriber
ros::Subscriber<std_msgs::Float32> sub("setSpeed", setSpeedCB);

void setup() {
    controllerSetup();
    nh.getHardware()->setBaud(93600);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
}

void loop() {
    // Motor Control Stuff here:
    controllerLoop();

    // Publisher stuff here:

    // Handle the ROS stuff
    nh.spinOnce();
}

void controllerSetup()
{
  /* SETUP */
  pidCon.resetController();

  /// Pin - setup: PC 0-3 to input (PCINT8 - 11)
  DDRC &= 0b11110011; // 0 => input / 1 => leave state

  /// Pin - setup: PD3, PD5, PD6, PB3 as output
  DDRD |= 0b01101001;
  DDRB |= 0b00001000;

  /// Timer0 / 2 [PWM-Setup]
  TCCR0A = TCCR2A = 0xF3; // Fast-PWM (inverting mode)
/*
  TCCR0B = TCCR2B = 0x01; // start Timer prescaler clk/1 => F_CPU/255
*/
  OCR0A = OCR0B = OCR2A = OCR2B = 0;
  /// EXTERNAL INTERUPT
  PCICR |= (1 << PCIE1);    // Enable Pin Change Interrupt
  PCMSK1 = 0b00001100;      // Enable Pins PCINT8...PCINT11 (PC0...PC3) for Pin Change Interrupts
  /// Timer1 [Clock-Setup]
  OCR1A = 0xFFFF;           // Set Output Compare Register A of Timer 1 to 0xFFFF = 65536 = 16 bit
  TCCR1B = (1 << WGM12);    // Mode 4: Clear Timer on Compare Match (CTC): if(TCNT1==OCR1A){TCNT=0;}
  TIMSK1 |= (1 << OCIE1A);  // Enable Interrupt on Output Compare Match 
  TCCR1B |= (1 << CS10);    // Set Prescaler to 1 and start the timer 
/*
                            // [original comment: set prescaler to 8 and start the timer (OSC)]
  /* END SETUP */
  //wdt_enable(WDTO_500MS);
  // wdt_enable(WDTO_15MS);    // Enable Watch Dog Timer with Time Out = 15 ms
  sei();                    // Enable Interrupts
}

void controllerLoop()
{
    // wdt_reset(); // Reset Watchdog Timer
    uint16_t ticks = micros(); // Read current value of TCNT1 
    uint16_t dtTick = ticks - ticksLastPID;
    if (dtTick >= 4000) // ~0.001s => 1kHz control-loop  ///TODO: overflow?!?!?!?
    {
        float dt = (dtTick) / (1000000.0f);
        ticksLastPID = ticks;

        // calc vals
        curr_enc = EncoderPos;
        int16_t d_enc = (curr_enc - last_enc);
        last_enc = curr_enc;

        getRPM = ((float)d_enc) / dt;

        // calc lpf-vals
        rpm_lpf += (getRPM - rpm_lpf) * lpf_factor;
        pwm_lpf += (getPWM - pwm_lpf) * lpf_factor;

        encMsg.data = rpm_lpf;
        pub.publish(&encMsg);

        float pidVal = pidCon.runController(rpm_lpf, setRPM, dt);
        float pwm = pidVal; // + vorsteuerung(setRPM, mvsBalance0, mvsBalance1);

        getPIDVal += (pidVal - getPIDVal) * lpf_factor;  //debug

        if (setRPM * pwm < 0)
        {
            pwm = 0; //speed in wrong dir
        }

        if (pwm > 255)
        {
            pwm = 255;
        }
        if (pwm < -255)
        {
            pwm = -255;
        }

        getPWM = pwm;
        
        setMotor(getPWM, 1);

        /*
        Serial.print("PWM:");
        Serial.print(getPWM, 0);
        Serial.print("\t");
        //Serial.print(pwm_lpf, 0);
        //Serial.print("\t");
        Serial.print("RPM:");
        Serial.print(setRPM, 0);
        Serial.print("\t");
        //Serial.print(rpm_lpf, 0);
        //Serial.print("\t");

        Serial.println();
        */
    }
}


// TIMER things
ISR(TIMER1_COMPA_vect)
{
  ticksMostSig++;
}

ISR(PCINT1_vect)
{
    cache = PINC; // Save Pin states of Pin PC0...PC6 to cache

    // ^ = Bitwise exclusive OR (XOR)
    //    0 ^ 0 = 0
    //    0 ^ 1 = 1
    //    1 ^ 0 = 1
    //    1 ^ 1 = 0
    uint8_t x1 = (cache ^ (cacheOld >> 1)); // Bitshift cacheOld to the right and XOR with cache
    uint8_t x2 = (cacheOld ^ (cache >> 1)); // Bitshift cache to the right and XOR with cacheOld

    /*
    // ENCODER A (PC0 and PC1)


    // && = Logical AND   & = Bitwise AND
    //      0 &&   0 = 0      0 & 0 = 0
    //      0 && >=1 = 0      0 & 1 = 0
    //    >=1 &&   0 = 0      1 & 0 = 0
    //    >=1 && >=1 = 1      1 & 1 = 1
    if ((x1 & 0b00000001) && !(x2 & 0b00000001))
    {
    EncoderPos--;
    }
    if (!(x1 & 0b00000001) && (x2 & 0b00000001))
    {
    EncoderPos++;
    }
    */


    // ENCODER B (PC2 and PC3)
    if ((x1 & 0b00000100) && !(x2 & 0b00000100))
    {
        EncoderPos++;
    }
    if (!(x1 & 0b00000100) && (x2 & 0b00000100))
    {
        EncoderPos--;
    }
    cacheOld = cache;
}

// Functions
float vorsteuerung(float tarRPM, uint8_t mvsBalance0, uint8_t mvsBalance1)
{
  float ret[3];
  float absTarRPM = abs(tarRPM);
  for (int i = 0; i < 3; ++i)
  {
    ret[i] = mvs[i].a0x0 + (absTarRPM * mvs[i].a1x1);
    if (tarRPM < 0)
    {
      ret[i] *= -1;
    }
    if (tarRPM == 0)
    {
      ret[i] = 0;
    }
  }
  float divider0 = (float)mvsBalance0 / 255;
  float divider1 = (float)mvsBalance1 / 255;

  float out = divider0 * ret[0];                   // VSx
  out += (1 - divider0) * divider1 * ret[1];       // VSy
  out += (1 - divider0) * (1 - divider1) * ret[2]; // VSyaw
  return out;
}

inline uint16_t getTicks(void)
{
  return TCNT1;
}

void setMotor(float speed, uint8_t motor)
{
  uint16_t speed_int = abs(speed);
  if (motor == 0)
  {
    if ((speed >= 0))
    {
      OCR0B = (uint8_t)0;
      OCR0A = (uint8_t)speed_int;
    }
    else
    {
      OCR0B = (uint8_t)speed_int;
      OCR0A = (uint8_t)0;
    }
  }
  else
  {
    if ((speed >= 0))
    {
      OCR2B = (uint8_t)0;
      OCR2A = (uint8_t)speed_int;
    }
    else
    {
      OCR2B = (uint8_t)speed_int;
      OCR2A = (uint8_t)0;
    }
  }
}
