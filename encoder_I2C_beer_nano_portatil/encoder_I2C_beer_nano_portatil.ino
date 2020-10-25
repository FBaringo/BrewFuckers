/**
 * Name:     Arduino - Menu for LCD I2C + Encoder
 * Autor:    Paco Baringo
 * Web:      http://www.imagensubmarina.es
 * Date:     2020/08/23
 *
 * Arduino RF-NANO V3 pinout:
 *         _______________
 *        |      USB      |
 *        |D13         D12|
 *        |3V3         D11|
 *        |REF         D10|
 *    SO  |A0           D9|
 *    CS  |A1           D8|
 *   SCK  |A2           D7|
 *        |A3           D6| ENCODER CLK S2
 * LCD SDA|A4           D5| ENCODER DT  S1
 * LCD SCL|A5           D4| ENCODER SW  KEY
 *        |A6           D3| 
 *        |A7           D2| Salida Rele
 *        |5V          GND|
 *        |RST         RST|
 *        |GND          RX|
 *        |VIN          TX|
 *        |_______________|
 *
*/
 
 
/**
 *  LIBRERIAS NECESARIAS PARA EL FUNCIONAMIENTO DEL CODIGO
 */
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <max6675.h>


 
 
/**
 *  OBJETOS DE LAS LIBRERIAS
 */
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Configuracion del LCD I2C (puede ser necesario cambiar el primer valor con la direccion del LCD)
 
 
/**
 *  MACROS, CONSTANTES, ENUMERADORES, ESTRUCTURAS Y VARIABLES GLOBALES
 */
#define COUNT(x) sizeof(x)/sizeof(*x)                   // Macro para contar el numero de elementos de un array
const byte pENCO_SW   = 4;                              // Pin encoder SW
const byte pENCO_DT   = 5;                              // Pin encoder DT
const byte pENCO_CLK  = 6;                              // Pin encoder CLK
const byte rowsLCD    = 4;                              // Filas del LCD
const byte columnsLCD = 20;                             // Columnas del LCD
const byte iARROW     = 0;                              // ID icono flecha
const byte bARROW[]   = {                               // Bits icono flecha
    B00000, B00100, B00110, B11111,
    B00110, B00100, B00000, B00000
};

double Setpoint; //temperatura a seguir 
double Input = 0; //temperatura del termopar
double Output; //tiempo en milisegundos que calcula el PID
double Kp=1, Ki=0.05, Kd=0.25; // conservador
//double Kp=40, Ki=0.5, Kd=1; 
int thermoDO = A0;
int thermoCS = A1;
int thermoCLK = A2;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

unsigned long tiempoActual= millis(); // tiempo para ir recogiendo los datos de temperatura
unsigned long tInicial = millis(); // inicio para contar el tiempo total del proceso
unsigned long tC = 0; // tiempo inicio del tramo
unsigned long tiempoActual_lcd= millis();
//unsigned long tiempoTemp= millis();
bool estado;
int ciclo = 0;

int minutos;
int n_ciclos=4;
  double array_setpoint[] = {0.0, 0.0, 0.0, 0.0}; // Selecciona la tempeperatura a seguir tramo
  int array_minutos[] = {0, 0, 0, 0}; // cantidad de minutos a mantener la temperatura tramo
  boolean array_flag[] = {true, true, true, true}; // flag para que empiece a contar el tiempo una vez que ha alcanzado la temperatura marcada

float milisegundos;

 
enum Button{ Unknown, Ok, Left, Right } btnPressed;     // Enumerador con los diferentes botones disponibles
enum Screen{ Menu1, Menu2, Flag, Number };              // Enumerador con los distintos tipos de submenus disponibles
 
const char *txMENU[] = {                                // Los textos del menu principal, la longitud maxima = columnsLCD-1, rellenar caracteres sobrantes con espacios.
    "Temperatura 1      ",
    "Tiempo 1           ",
    "Temperatura 2      ",
    "Tiempo 2           ",
    "Temperatura 3      ",
    "Tiempo 3           ",
    "Temperatura 4      ",
    "Tiempo 4           ",
    "Guardar y salir    ",
    "Salir              ",
    "Apagar             "
};
const byte iMENU = COUNT(txMENU);                       // Numero de items/opciones del menu principal
 
enum eSMENU1{ Milliseconds, Seconds, Minutes, Hours };  // Enumerador de las opciones disponibles del submenu 1 (tienen que seguir el mismo orden que los textos)
const char *txSMENU1[] = {                              // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
    "   Milisegundos   ",
    "     Segundos     ",
    "     Minutos      ",
    "      Horas       "
};
 
enum eSMENU2{ GradeC, GradeF };                         // Enumerador de las opciones disponibles del submenu 2 (tienen que seguir el mismo orden que los textos)
const char *txSMENU2[] = {                              // Textos del submenu 1, longitud maxima = columnsLCD-2, rellenar caracteres sobrantes con espacios
    "     Grados C     ",
    "     Grados F     "
};
 
/* ESTRUCTURAS CONFIGURACION */
struct MYDATA{                                          // Estructura STRUCT con las variables que almacenaran los datos que se guardaran en la memoria EEPROM
    int initialized;
   // int n_ciclos;
    int temperatura_1;
    int tiempo_1;
    int temperatura_2;
    int tiempo_2;
    int temperatura_3;
    int tiempo_3;
    int temperatura_4;
    int tiempo_4;
};
union MEMORY{                                           // Estructura UNION para facilitar la lectura y escritura en la EEPROM de la estructura STRUCT
    MYDATA d;
    byte b[sizeof(MYDATA)];
}
memory;
 int WindowSize = 1000; //tiempo máximo de encendido en milisegundos
unsigned long windowStartTime;
 
/**
 * INICIO Y CONFIGURACION DEL PROGRAMA
 */
void setup()
{
    windowStartTime = millis(); //tiempo para ir calculando el tiempo transcurrido en cada ciclo de control PID
    Serial.begin(9600);
    pinMode(pENCO_SW,  INPUT_PULLUP);
    pinMode(pENCO_DT,  INPUT_PULLUP);
    pinMode(pENCO_CLK, INPUT_PULLUP);

    pinMode(2, OUTPUT); // salida rele
    //pinMode(3, OUTPUT); // salida rele
 
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
 
    // Carga la configuracion de la EEPROM, y la configura la primera vez:
    readConfiguration();
 
    // Inicia el LCD:
    lcd.begin(columnsLCD, rowsLCD);
    lcd.createChar(iARROW, bARROW);
 
    // Imprime la informacion del proyecto:
    lcd.setCursor(0,0); lcd.print("Menu Beer  Castellar");
    lcd.setCursor(0,1); lcd.print("     BrewFaker      ");
    lcd.setCursor(0,2); lcd.print(" Ver 2.0  2020/09   ");
    lcd.setCursor(0,4);
    for( int i=0 ; i<columnsLCD ; i++ )
    {
        lcd.print(".");
        delay(150);
    }
    lcd.clear();
    tInicial = millis();
    lcd.setCursor(0,0); lcd.print(" Pulsa para ajustar");
    lcd.setCursor(0,1); lcd.print("Temperatura duracion");
    lcd.setCursor(0,3); lcd.print("Temp actual "); lcd.print(thermocouple.readCelsius());lcd.print("C");
}
 
 
/**
 * PROGRAMA PRINCIPAL
 */
void loop()
{
  btnPressed = Button::Left;  
  while (ciclo==0 and btnPressed != Button::Ok){
    btnPressed = readButtons();  
  }
    lcd.clear();
    openMenu();
 
  
  
 while (ciclo<n_ciclos) {
  
  btnPressed = readButtons();
    if( btnPressed == Button::Ok ){
        openMenu();
        ciclo=0;
        tC = 0;
    }
    readConfiguration();
    Setpoint = array_setpoint[ciclo]; // Selecciona la tempeperatura a seguir
    if (Input - Setpoint > 0 && array_flag[ciclo]){ //cuando alcance la temperatura inicializa el tiempo de la rampa
      tC = millis();
      array_flag[ciclo] = false;
      minutos=array_minutos[ciclo];
      milisegundos = array_minutos[ciclo] * 60000 + tC;
   }

    if (millis() > milisegundos){ //cuando alcance el tiempo de duracion de la rampa pasa al siguiente ciclo
      ciclo = ciclo+1;
      tC=0;
    
  }
  
  
 if (millis()>tiempoActual + WindowSize){
      Input = thermocouple.readCelsius(); // se recoje una medida de Temp cada WindowSize
//    Serial.print("Temp: ");
//    Serial.print(Input);
//    Serial.print("   ");
//    Serial.print("ciclo:  ");
//    Serial.print(ciclo);
//    Serial.print("   ");
//    Serial.print(" Setpoint: ");
//    Serial.print(Setpoint);
//    Serial.print("   ");
//    Serial.print("tiempo de comienzo del tramo  ");
//    Serial.print(tC);
//    Serial.print("   ");
//    Serial.print("tiempo actual  ");
//    Serial.print(millis());
//    Serial.print(" tiempo restante:  ");
//    Serial.print("   ");
//    if(tC > 0) {Serial.println(trestante);} else {Serial.println("Esperando");};
//   for( int i=0 ; i < sizeof(memory.d) ; i++  ){
//     Serial.print(i);
//     Serial.print("   ");
//     Serial.println(memory.b[i]); 
//    }
    tiempoActual = millis();
  }
  
  if (millis() > tiempoActual_lcd + 1000) { // muestra el display 1 vez x seg
    tiempoActual_lcd = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(Input);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("Ciclo:");
    lcd.print(ciclo+1);
    lcd.print(" Tem_F:");
    lcd.print(Setpoint);
    lcd.print("C");
    lcd.setCursor(0, 2);
    lcd.print("Duracion:");
    lcd.print(minutos);
    lcd.print(" M");
    lcd.setCursor(0, 3);
    lcd.print("Tiempo rest:");
    if (tC > 0) {lcd.print((milisegundos - millis())/1000/60);lcd.print(" M");}
      else {lcd.print("Espera");}
    }

  myPID.Compute();

  /**********************************************
   * control del dispositivo
   ************************************************/
    Serial.print("Output: ");
    Serial.print(Output);
    Serial.print("   ");
    Serial.print("diferncia tiempo:  ");
    Serial.print(millis() - windowStartTime);
    Serial.print("   ");
    Serial.print("windowStartTime:  ");
    Serial.println(windowStartTime);

 
 if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > millis() - windowStartTime) digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);


//     
//  if (millis() - tiempoActual < Output){
//    digitalWrite(2, HIGH);
//    digitalWrite(3, HIGH);
//    estado = HIGH;
//  }
//  if (millis() - tiempoActual > Output){
//    digitalWrite(2, LOW);
//    digitalWrite(3, LOW);
//    estado = LOW;
//  }
} 
 if (ciclo >= n_ciclos){
    lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Proceso Terminado");
      lcd.setCursor(0, 1);
      lcd.print("Tiempo total:");
      lcd.print((millis()-tInicial)/1000/60);
      lcd.print(" M");
      lcd.setCursor(4, 2);
      lcd.print("Enjoy");
      lcd.setCursor(0, 3);
      lcd.print("Temp:");
      lcd.print(Input);
      lcd.print(" C");
      Serial.print("   ");
      Serial.print("Proceso terminado  ");
      Serial.print("   ");
      Serial.print("tiempo total minutos ");
      Serial.println((millis()-tInicial)/1000/60);
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      estado = LOW;
      delay(1000);
      while (btnPressed != Button::Ok){
        btnPressed = readButtons(); 
        ciclo=0; 
      }
      
   }
}

/**
 *  MUESTRA EL MENU PRINCIPAL EN EL LCD.
 */
void openMenu()
{
    byte idxMenu       = 0;
    boolean exitMenu   = false;
    boolean forcePrint = true;
 
    lcd.clear();
 
    while( !exitMenu )
    {
        btnPressed = readButtons();
 
        if( btnPressed == Button::Left && idxMenu-1 >= 0 )
        {
            idxMenu--;
        }
        else if( btnPressed == Button::Right && idxMenu+1 < iMENU )
        {
            idxMenu++;
        }
        else if( btnPressed == Button::Ok )
        {
            switch( idxMenu )
            {
                case 0: openSubMenu( idxMenu, Screen::Number, &memory.d.temperatura_1,    0, 100      ); break;
                case 1: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempo_1,    0, 240           ); break;
                case 2: openSubMenu( idxMenu, Screen::Number, &memory.d.temperatura_2,    0, 100      ); break;
                case 3: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempo_2,    0, 240           ); break;
                case 4: openSubMenu( idxMenu, Screen::Number, &memory.d.temperatura_3,    0, 100      ); break;
                case 5: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempo_3,    0, 240           ); break;
                case 6: openSubMenu( idxMenu, Screen::Number, &memory.d.temperatura_4,    0, 100      ); break;
                case 7: openSubMenu( idxMenu, Screen::Number, &memory.d.tiempo_4,    0, 240           ); break;
                case 8: writeConfiguration(); exitMenu = true;                                           break; //Salir y guardar
                case 9: readConfiguration();  exitMenu = true;                                           break; //Salir y cancelar cambios
                case 10: digitalWrite(2, LOW); asm("jmp 0x0000");                                         break; //Reset
            }
            forcePrint = true;
        }
 
 
        if( !exitMenu && (forcePrint || btnPressed != Button::Unknown) )
        {
            forcePrint = false;
 
            static const byte endFor1 = (iMENU+rowsLCD-1)/rowsLCD;
            int graphMenu     = 0;
 
            for( int i=1 ; i<=endFor1 ; i++ )
            {
                if( idxMenu < i*rowsLCD )
                {
                    graphMenu = (i-1) * rowsLCD;
                    break;
                }
            }
 
            byte endFor2 = graphMenu+rowsLCD;
 
            for( int i=graphMenu, j=0; i< endFor2 ; i++, j++ )
            {
                lcd.setCursor(1, j);
                lcd.print( (i<iMENU) ? txMENU[i] : "                    " );
            }
 
            for( int i=0 ; i<rowsLCD ; i++ )
            {
                lcd.setCursor(0, i);
                lcd.print(" ");
            }
            lcd.setCursor(0, idxMenu % rowsLCD );
            lcd.write(iARROW);
        }
    }
 
    lcd.clear();
}
 
 
/**
 * MUESTRA EL SUBMENU EN EL LCD.
 *
 * @param menuID    ID del menu principal para usarlo como titulo del submenu
 * @param screen    Segun el tipo, se representara el submenu de una forma u otra.
 * @param value     Puntero a la variable que almacena el dato, y que se modificara.
 * @param minValue  Valor minimo que puede tener la variable.
 * @param maxValue  Valor maximo que puede tener la variable.
 */
void openSubMenu( byte menuID, Screen screen, int *value, int minValue, int maxValue )
{
    boolean exitSubMenu = false;
    boolean forcePrint  = true;
 
    lcd.clear();
 
    while( !exitSubMenu )
    {
        btnPressed = readButtons();
 
        if( btnPressed == Button::Ok )
        {
            exitSubMenu = true;
        }
        else if( btnPressed == Button::Left && (*value)-1 >= minValue )
        {
            (*value)--;
        }
        else if( btnPressed == Button::Right && (*value)+1 <= maxValue )
        {
            (*value)++;
        }
 
 
        if( !exitSubMenu && (forcePrint || btnPressed != Button::Unknown) )
        {
            forcePrint = false;
 
            lcd.setCursor(0,0);
            lcd.print(txMENU[menuID]);
 
            lcd.setCursor(0,1);
            lcd.print("<");
            lcd.setCursor(columnsLCD-1,1);
            lcd.print(">");
 
            if( screen == Screen::Menu1 )
            {
                lcd.setCursor(1,1);
                lcd.print(txSMENU1[*value]);
            }
            else if( screen == Screen::Menu2 )
            {
                lcd.setCursor(1,1);
                lcd.print(txSMENU2[*value]);
            }
            else if( screen == Screen::Flag )
            {
                lcd.setCursor(columnsLCD/2-1, 1);
                lcd.print(*value == 0 ? "NO" : "SI");
            }
            else if( screen == Screen::Number )
            {
                lcd.setCursor(columnsLCD/2-1, 1);
                lcd.print(*value);
                lcd.print(" ");
            }
        }
 
    }
 
    lcd.clear();
}
 
 
/**
 *  LEE (Y CONFIGURA LA PRIMERA VEZ) LA MEMORIA EEPROM CON LA CONFIGURACION DE USUARIO
 */
void readConfiguration()
{
    for( int i=0 ; i < sizeof(memory.d) ; i++  ){
        memory.b[i] = EEPROM.read(i); 
    }
    

    if( memory.d.initialized != 'Y' )
    {
        memory.d.initialized = 'Y';
        memory.d.temperatura_1   = 0;
        memory.d.tiempo_1        = 0;
        memory.d.temperatura_2   = 0;
        memory.d.tiempo_2        = 0;
        memory.d.temperatura_3   = 0;
        memory.d.tiempo_3        = 0;
        memory.d.temperatura_4   = 0;
        memory.d.tiempo_4        = 0;
        writeConfiguration();
    }
//array_setpoint[0]= EEPROM.read(2); se alamacena en estas posciones ??
//array_minutos[0]= EEPROM.read(4);
//array_setpoint[1]= EEPROM.read(6);
//array_minutos[1]= EEPROM.read(8);
//array_setpoint[2]= EEPROM.read(10);
//array_minutos[2]= EEPROM.read(12);
//inicializo los arrays de temperatura y duración 
    int i=2;
    int k=0;
    while (i < sizeof(memory.d)){
        array_setpoint[k]= EEPROM.read(i);
        i=i+2;
        array_minutos[k]= EEPROM.read(i);
        k=k+1;
        i=i+2;
      }
    
    
}
 
 
/**
 *  ESCRIBE LA MEMORIA EEPROM CON AL CONFIGURACION DE USUARIO
 */
void writeConfiguration()
{
    for( int i=0 ; i<sizeof(memory.d) ; i++  )
        EEPROM.write( i, memory.b[i] );
}
 
 
/**
 *  LEE LOS DISTINTOS BOTONES DISPONIBLES Y DEVUELVE EL QUE HAYA SIDO PULSADO
 *      Este bloque de codigo varia dependiendo del tipo de teclado conectado, en mi blog estan
 *      disponibles los ejemplos para teclados digitales, analogicos, y este para encoder rotatorio.
 *      Al cambiar de tipo de teclado hay que adaptar tambien el resto de codigo para que haga uso de cada boton segun queramos.
 */
Button readButtons()
{
    static boolean oldA = HIGH;
    static boolean newA = LOW;
    static boolean newB = LOW;
 
    btnPressed = Button::Unknown;
    newA = digitalRead(pENCO_DT);
    newB = digitalRead(pENCO_CLK);
 
    if( !oldA && newA )
    {
        btnPressed = !newB ? Button::Left : Button::Right;
        delay(50);
    }
    else if( !digitalRead(pENCO_SW) )
    {
        while(!digitalRead(pENCO_SW));
        btnPressed = Button::Ok;
        delay(50);
    }
 
    oldA = newA;
    return btnPressed;
}
