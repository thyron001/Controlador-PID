//////////////////////////
//     PINES
//////////////////////////
const int PWM_out = 9;  //Sale PWM

const int pin_InEncoder = 2;  //Recibe pulsos encoder

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
int rpm;
float rpm_real;




/////////////////////////////
//    FUNCIONES
/////////////////////////////

//Covertir el valor del potenciometro a RPMS
float convertV(int val) {
  return (float)(val * 4400.00) / (float)10;
}


//Función para contar pulsos
void ContarPulsos() {
  pulsos = pulsos + 1;
}
//////////////////


void setup() {
  //previousMillis = millis();
  attachInterrupt(digitalPinToInterrupt(pin_InEncoder), ContarPulsos, RISING);
  pinMode(pin_InEncoder, INPUT_PULLUP);
  pinMode(PWM_out, OUTPUT);

  e[2] = 0;
  e[1] = 0;
  e[3] = 0;
  
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  Serial1.print("page1.rpm_real.val="); 
  Serial1.print(int(rpm_real));    
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff); 
  
  
  
  if (Serial1.available() > 0){
    
    String Received = Serial1.readString();
        
    if (Received[0] == 'b'){
//    Kp = float(Received[1]);
//    Ti = float(Received[5]);
//    Td = float(Received[9]);
      
      rpm = int(Received[13]);
      } 
      
     
    
    
  }
   Kp = 0.25;
   Ti = 0.15;
   Td = 0.0375;
 
  
  //=======================================================================
  //        Modificar la velocidad
  //=======================================================================
  
  float ref = convertV(rpm);     //Valor analogico del POT a RMPS
  //float PWM_val = rpm * 255 / 10 ;            //Transformar a PWM
  //analogWrite(PWM_out, PWM_val);    //Enviar PWM al motor

  //=====================================================================
  //      Medir la velocidad del motor
  //=====================================================================

  if ((millis() - previousMillis) >= T) {

    //-----------------------------------------------------
    //          MEDIR RPMS
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
    rpm_real = rpms * 255 / 4400;
    
    Serial.print(ref);
    Serial.print(',');
    Serial.print(int(rpms));
    Serial.println(',');

  } 
}
