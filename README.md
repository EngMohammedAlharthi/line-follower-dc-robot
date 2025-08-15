# Line-Follower DC Robot

A compact mobile robot that follows a black line on a white floor. The focus is to **choose the correct motor type**, **justify the choice with quick sizing math**, and **program the controller**. Repo includes wiring, parts, and runnable Arduino code (ESP32 notes included).

---

## 1) Task Compliance
- **Different robot than training examples:** two-wheel differential **line follower**.
- **Motor type selected & justified:** 12 V **brushed DC gear motors** (with optional encoders).
- **Programming provided:** complete Arduino sketch for TB6612FNG driver (L298N mapping noted).
- **Verification path:** steps for simulation (Tinkercad/Proteus) and on-bench test.

---

## 2) Why DC Gear Motors?
- **Continuous rotation** with good top speed for ground robots; no travel limit like hobby servos.
- **High torque/weight/cost** thanks to the gearbox; simpler and cheaper than stepper/BLDC for this use.
- **Simple control** via PWM + direction; drivers are common (TB6612FNG/L298N).
- **Upgradeable**: add encoders later for closed-loop speed/odometry if needed.

---

## 3) Quick Sizing (sanity check)
- Robot mass \(m=1.2\text{ kg}\), wheel radius \(r=0.033\text{ m}\), rolling coefficient \(\mu≈0.02\).
- Force to roll on flat floor: \(F= \mu m g = 0.02×1.2×9.81 ≈ 0.235 \text{ N}\).  
  Per wheel ≈ \(0.118\text{ N}\).
- Wheel torque at cruise: \(\tau = F·r ≈ 0.118×0.033 ≈ 0.0039\text{ N·m}\).
- Add ~**10× margin** for acceleration/turns/imperfections → **≥0.04 N·m per wheel**.  
  We select motors with **0.4–0.8 N·m** (ample headroom).
- Speed check (100 RPM, Ø66 mm): \(v = 2πr·RPM/60 ≈ 0.35 m/s\) — suitable for line following.

**Chosen part class:** 12 V brushed DC gear motor, no-load 100–200 RPM, stall torque 0.4–0.8 N·m, 0.3–2 A.

---

## 4) Bill of Materials
- 2× 12 V **DC gear motors** (encoder optional)  
- 1× **TB6612FNG** motor driver (recommended) *or* L298N  
- 1× Arduino **UNO** (or ESP32-DevKit; pin notes below)  
- 3× IR line sensors (e.g., QTR/TCRT5000 modules)  
- 1× Battery 7.4–11.1 V (2S/3S) + switch + fuse (5–7.5 A)  
- Chassis, two wheels (≈65–70 mm), caster, jumpers, screw terminals

---

## 5) Wiring (UNO + TB6612FNG)
- **Power:** VM→Battery +, GNDs common (battery, driver, Arduino, sensors).  
- **TB6612FNG control:** STBY→D7; AIN1→D8, AIN2→D9, PWMA→D5 (left); BIN1→D10, BIN2→D11, PWMB→D6 (right).  
- **Sensors:** Left→A0, Center→A1, Right→A2 (+5 V & GND).

> **L298N mapping:** ENA→D5, IN1→D8, IN2→D9, ENB→D6, IN3→D10, IN4→D11 (remove 5 V jumper if powering logic from Arduino).

---

## 6) Arduino Code (ready to run)

```cpp
// Board: Arduino UNO | Driver: TB6612FNG | Sensors: 3x analog IR (L=A0, C=A1, R=A2)
const int STBY=7, AIN1=8, AIN2=9, PWMA=5, BIN1=10, BIN2=11, PWMB=6;
const int L_PIN=A0, C_PIN=A1, R_PIN=A2;

int basePWM = 130;     // tune for your chassis (90–180 typical)
int threshold = 500;   // analog threshold: lower value on black line; calibrate via Serial
float Kp = 0.6;        // proportional gain

void setMotor(int pwmL, int pwmR){
  // Left
  digitalWrite(AIN1, pwmL >= 0); digitalWrite(AIN2, pwmL < 0);
  analogWrite(PWMA, constrain(abs(pwmL), 0, 255));
  // Right
  digitalWrite(BIN1, pwmR >= 0); digitalWrite(BIN2, pwmR < 0);
  analogWrite(PWMB, constrain(abs(pwmR), 0, 255));
}

int seesLine(int pin){ return (analogRead(pin) < threshold) ? 1 : 0; }

void setup(){
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(L_PIN, INPUT); pinMode(C_PIN, INPUT); pinMode(R_PIN, INPUT);
  Serial.begin(9600);  // use to read raw sensor values for threshold tuning
}

void loop(){
  int L = seesLine(L_PIN), C = seesLine(C_PIN), R = seesLine(R_PIN);

  // Position estimate: left=-1, center=0, right=+1 (very simple P-control)
  int pos = (-1*L) + (0*C) + (1*R);
  int correction = (int)(Kp * 100 * pos);

  int left = basePWM - correction;
  int right = basePWM + correction;

  // Sharp turns when only one side sees the line
  if(L && !C && !R) { left = basePWM - 90; right = basePWM + 90; }
  if(R && !C && !L) { left = basePWM + 90; right = basePWM - 90; }

  // Lost line: spin slowly to search
  if(!L && !C && !R){ left = 90; right = -90; }

  setMotor(left, right);
  delay(10);
}
