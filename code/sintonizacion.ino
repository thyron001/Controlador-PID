//////////////////////////
//		 PINES
//////////////////////////
const int in_pot = A3;   //Entra voltaje
const int PWM_out = 9;  //Sale PWM

const int input_1_M1 = 5;  //Controla sentido giro motor
const int input_2_M1 = 6;  //Controla sentido giro motor

const int pin_InEncoder = 2;  //Recibe pulsos encoder




//////////////////////////////////////////////////
//Variables para la medición de velocidad
int pulsos = 0;
float T = 50;
unsigned long previousMillis = 0;


//////Variables para el controlador PID
float e[3];
float u = 0;

float Kp;
float Ti;
float Td;



///
/////////////////////////////
//		FUNCIONES
/////////////////////////////

//Covertir el valor del potenciometro a RPMS
float convertV(int val) {
  return (float)(val * 4400.00) / (float)1023;
}

//Mapear PWM
int Map_PWM(int analog_val) {

  int analog2PWM;
  analog2PWM = map(analog_val, 0, 1023, 0, 255);
  return analog2PWM;
}

//Función para contar pulsos
void ContarPulsos() {
  pulsos = pulsos + 1;
}
//////////////////

void setup() {
  //previousMillis = millis();
  attachInterrupt(digitalPinToInterrupt(pin_InEncoder), ContarPulsos, RISING);

  pinMode(in_pot, INPUT);
  pinMode(pin_InEncoder, INPUT_PULLUP);
  digitalWrite(input_1_M1, LOW);
  digitalWrite(input_2_M1, HIGH);
  pinMode(PWM_out, OUTPUT);
  Serial.begin(9600);

  // inicialización de variables del PID
  Kp = 0.25;
  Ti = 0.15;
  Td = 0.0375;

  e[2] = 0;
  e[1] = 0;
  e[3] = 0;

}


void loop() {

  //=======================================================================
  //				Modificar la velocidad
  //=======================================================================
  int potVal = analogRead(in_pot);  //Leer el valor del potenciometro
  float ref = convertV(potVal);     //Valor analogico del POT a RMPS
  int PWM_val = Map_PWM(potVal);    //Transformar a PWM
  //analogWrite(PWM_out, PWM_val);    //Enviar PWM al motor


  

  //=====================================================================
  //			Medir la velocidad del motor
  //=====================================================================

  if ((millis() - previousMillis) >= T) {

    //-----------------------------------------------------
    //					MEDIR RPMS
    float rpms = (60 * pulsos) / (48 * T * 0.001);
    pulsos = 0;
    previousMillis = millis();
    //-----------------------------------------------------

    //-----------------------------------------------------
    // FUNCION DE RECURRENCIA
    //-----------------------------------------------------
    e[2] = e[1];
    e[1] = e[0];
    e[0] = ref - rpms;
    u += Kp * ( e[0] - e[1] + 0.05 / Ti * e[0] + Td / 0.05 * (e[0] - 2 * e[1] + e[2]) );
    

    //u += deltaU;  // u anterior + deltaU

    if (u > 255)
      u = 255;

    if (u < 0)
      u = 0;

    analogWrite(PWM_out, int(u));  //u debe ser entero

    //Serial.print(potVal);
    //Serial.print(',');
    //Serial.print(PWM_val);
    //Serial.print(',');
    //Serial.println(ref);

    Serial.print(ref);
    Serial.print(',');
    Serial.print(rpms);
    Serial.print(',');
    Serial.println(int(u));
  }
}