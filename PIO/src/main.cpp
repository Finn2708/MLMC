#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <mlmc_msgs/PID.h>

#include <PIDController.h>

#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>

ros::NodeHandle nh;

// Message definition
std_msgs::Float32 encMsg;
mlmc_msgs::PID pidMsg;

// Variables

typedef struct
{
  float P;
  float I;
  float D;
} PID_t;

typedef struct
{
  float a0;
  float a1;
} FFD_t;

FFD_t ffd[3] = {{0}, {0}, {0}};
uint32_t ticksLastPID;
int32_t currentEncoderPos;
int32_t lastEncoderPos;
float getRPM = 0.;
float setRPM;
float getPWM;
float getPIDVal;
float lpfFactor=.1;
float lpfRPM;
float lpfPWM;

volatile uint32_t ticksMostSig;

int32_t encoderPos;
int8_t ffdBalance0 = 0;
int8_t ffdBalance1;

volatile uint8_t cache, cacheOld;

PID_t pid = {0.100,
             5.000,
             0.000};

static PIDController pidCtr = PIDController(pid.P, pid.I, pid.D, 127, -127);


// Subscriber Callbacks
void setSpeedCB(const std_msgs::Float32& msg)
{
    setRPM = msg.data;
}


void setPIDCB(const mlmc_msgs::PID& msg)
{
    pidCtr.tuneGains(msg.p, msg.i, msg.d, 127, -127);
    ffdBalance0 = msg.ffd0;
    ffdBalance1 = msg.ffd1;
}


// Publisher 
ros::Publisher pub("encoderSpeed", &encMsg);

// Subscribers
ros::Subscriber<std_msgs::Float32> sub("setSpeed", setSpeedCB);
ros::Subscriber<mlmc_msgs::PID> pidSub("setPID", setPIDCB);

// Functions
float ffdControl(float tarRPM, uint8_t ffdBalance0, uint8_t ffdBalance1)
{
  float ret[3];
  float absTarRPM = abs(tarRPM);
  for (int i = 0; i < 3; ++i)
  {
    ret[i] = ffd[i].a0 + (absTarRPM * ffd[i].a1);
    if (tarRPM < 0)
    {
      ret[i] *= -1;
    }
    if (tarRPM == 0)
    {
      ret[i] = 0;
    }
  }
  float divider0 = (float)ffdBalance0 / 255;
  float divider1 = (float)ffdBalance1 / 255;

  float out = divider0 * ret[0];                   // VSx
  out += (1 - divider0) * divider1 * ret[1];       // VSy
  out += (1 - divider0) * (1 - divider1) * ret[2]; // VSyaw
  return out;
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


void controllerSetup()
{
  // SETUP
  pidCtr.resetController();

  // Pin - setup: PC 0-3 to input (PCINT8 - 11)
  DDRC &= 0b11110011;       // 0 => input / 1 => leave state

  // Pin - setup: PD3, PD5, PD6, PB3 as output
  DDRD |= 0b01101001;
  DDRB |= 0b00001000;

  // Timer0 / 2 [PWM-Setup]
  TCCR0A = TCCR2A = 0xF3;   // Fast-PWM (inverting mode)

  OCR0A = OCR0B = OCR2A = OCR2B = 0;

  // EXTERNAL INTERUPT
  PCICR |= (1 << PCIE1);    // Enable Pin Change Interrupt
  PCMSK1 = 0b00001100;      // Enable Pins PCINT8...PCINT11 (PC0...PC3) for Pin Change Interrupts

  // Timer1 [Clock-Setup]
  OCR1A = 0xFFFF;           // Set Output Compare Register A of Timer 1 to 0xFFFF = 65536 = 16 bit
  TCCR1B = (1 << WGM12);    // Mode 4: Clear Timer on Compare Match (CTC): if(TCNT1==OCR1A){TCNT=0;}
  TIMSK1 |= (1 << OCIE1A);  // Enable Interrupt on Output Compare Match 
  TCCR1B |= (1 << CS10);    // Set Prescaler to 1 and start the timer 

  sei();                    // Enable Interrupts
}


void controllerLoop()
{
    uint16_t ticks = micros(); // Read current value of TCNT1 
    uint16_t dtTick = ticks - ticksLastPID;
    if (dtTick >= 1415) // 700 Hz control-loop
    {
        float dt = (dtTick) / (1000000.0f);
        ticksLastPID = ticks;

        // calc vals
        currentEncoderPos = encoderPos;
        int16_t d_enc = (currentEncoderPos - lastEncoderPos);
        lastEncoderPos = currentEncoderPos;

        getRPM = ((float)d_enc) / dt;

        // calc lpf-vals
        lpfRPM += (getRPM - lpfRPM) * lpfFactor;
        lpfPWM += (getPWM - lpfPWM) * lpfFactor;

        encMsg.data = lpfRPM;
        pub.publish(&encMsg);

        float pidVal = pidCtr.runController(lpfRPM, setRPM, dt);
        float pwm = pidVal + ffdControl(setRPM, ffdBalance0, ffdBalance1);

        getPIDVal += (pidVal - getPIDVal) * lpfFactor;  //debug

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
        //Serial.print(lpfPWM, 0);
        //Serial.print("\t");
        Serial.print("RPM:");
        Serial.print(setRPM, 0);
        Serial.print("\t");
        //Serial.print(lpfRPM, 0);
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
    encoderPos--;
    }
    if (!(x1 & 0b00000001) && (x2 & 0b00000001))
    {
    encoderPos++;
    }
    */


    // ENCODER B (PC2 and PC3)
    if ((x1 & 0b00000100) && !(x2 & 0b00000100))
    {
        encoderPos++;
    }
    if (!(x1 & 0b00000100) && (x2 & 0b00000100))
    {
        encoderPos--;
    }
    cacheOld = cache;
}


inline uint16_t getTicks(void)
{
  return TCNT1;
}


void setup() {
    controllerSetup();
    nh.getHardware()->setBaud(93600);
    nh.initNode();

    nh.subscribe(sub);
    nh.subscribe(pidSub);
    nh.advertise(pub);
}


void loop() {
    // Motor Control Stuff here:
    controllerLoop();

    // Publisher stuff here:

    // Handle the ROS stuff
    nh.spinOnce();
}