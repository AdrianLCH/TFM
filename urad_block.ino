#include "Arduino.h"
#include "SPI.h"
//#include "uRAD.h"



#define pin_SS 5
#define pin_ON_OFF 32
#define ACK 170
#define NtarMax 5
#define f0Min 5
#define f0Max 195
#define f0Max_CW 245
#define BWMin 50
#define BWMax 240
#define NsMin 51
#define NsMax 200
#define RmaxMax 10
#define VmaxMax 63

// uRAD codes
const uint8_t code_conf = 227, code_isready = 204, code_results = 199, code_I = 156, code_Q = 51;

// input parameters
const uint8_t mode = 2;   // saw mode
const uint8_t f0 = 5;     // starting at 24.005 GHz
const uint8_t BW = 50;   // using all the BW available = 240 MHz
const uint8_t Ns = 51;    // samples (min 50, max 200)
const uint8_t Ntar = 0;   // 2 targets of interest
const uint8_t Rmax = 1;   // searching along the full distance range
const uint8_t MTI = 0;    // MTI disabled because we want information of static and moving targets
const uint8_t Mth = 0;    // parameter not used because "movement" is not requested

int Np = 8;              // Number of pulses, max 342 with Ns=200 (it exhausts memory!)
uint8_t waitIQ1 = 15000;
uint8_t waitIQ2 = 10;

uint8_t conf[8];

uint8_t compute_crc(uint8_t vec[]) {
    uint8_t crc = 0;
    for (int i=0; i<7; i++)
        crc += vec[i];
    return crc;
}

void loadConfiguration(uint8_t mode, uint8_t f0, uint8_t BW, uint8_t Ns, uint8_t Ntar, uint8_t Rmax, uint8_t MTI, uint8_t Mth) {

    // Check correct values
    if ((mode == 0) || (mode > 4)) {
        mode = 3;
    }
    if ((f0 > f0Max) && (mode != 1)) {
        f0 = f0Min;
    } else if ((f0 > f0Max_CW) && (mode == 1)) {
        f0 = f0Min;
    } else if (f0 < f0Min) {
        f0 = f0Min;
    }
    uint8_t BW_available = BWMax - f0 + f0Min;
    if ((BW < BWMin) || (BW > BW_available)) {
        BW = BW_available;
    }
    if ((Ns < NsMin) || (Ns > NsMax)) {
        Ns = NsMax;
    }
    if ((Ntar == 0) || (Ntar > NtarMax)) {
        Ntar = 1;
    }
    if ((mode != 1) && ((Rmax < 1) || (Rmax > RmaxMax))) {
        Rmax = RmaxMax;
    } else if ((mode == 1) && (Rmax > VmaxMax)) {
        Rmax = VmaxMax;
    }
    if (MTI > 1) {
        MTI = 0;
    }
    if ((Mth == 0) || (Mth > 4)) {
        Mth = 4;
    }
    Mth--;

    // Create configuration register
    conf[0] = (mode << 5) + (f0 >> 3);
    conf[1] = (f0 << 5) + (BW >> 3);
    conf[2] = (BW << 5) + (Ns >> 3);
    conf[3] = (Ns << 5) + (Ntar << 2) + (Rmax >> 6);
    conf[4] = (Rmax << 2) + MTI;
    conf[5] = (Mth << 6) + 0b00100000;
    conf[6] = 0b00011000;   // Only interested in I-Q
    conf[7] = compute_crc(conf);
}

void detection(uint16_t bufferI[], uint16_t bufferQ[]) {

    // Variables
    uint8_t rx_ACK;
    const uint16_t max_iterations = 5000;
    uint16_t iterations, temp;

    // This is moved to inicialization: I take full control of the SPI bus (very rude!)
    // SPI configuration
    //SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    //delayMicroseconds(100);
    //SPI.transfer(255);

    //Send configuration to uRAD by SPI
    conf[7] = compute_crc(conf);
    iterations = 0;
    do {
        digitalWrite(pin_SS, LOW);
        SPI.transfer(code_conf);
        delayMicroseconds(20);
        for (auto a : conf) SPI.transfer(a);
        delayMicroseconds(20);
        rx_ACK = SPI.transfer(0);
        digitalWrite(pin_SS, HIGH);
        ++iterations;
    } while ((rx_ACK != ACK) && (iterations < max_iterations));

    #ifdef DEBUG
    Serial.print("conf: ");
    Serial.print(iterations);
    Serial.print(" ");
    #endif

    if (rx_ACK != ACK) goto error;

    delayMicroseconds(20);

    // Are results ready?
    iterations = 0;
    do {
        digitalWrite(pin_SS, LOW);
        SPI.transfer(code_isready);
        delayMicroseconds(iterations==0 ? waitIQ1 : waitIQ2);
        /*delayMicroseconds(100);*/
        rx_ACK = SPI.transfer(0);
        digitalWrite(pin_SS, HIGH);
        delayMicroseconds(10);
        ++iterations;
    }
    while ((rx_ACK != ACK) && (iterations < max_iterations));

    #ifdef DEBUG
    Serial.print("isready: ");
    Serial.print(iterations);
    Serial.print(" ");
    #endif

    if (rx_ACK != ACK) goto error;

    delayMicroseconds(20);

    // Receive results: I,Q
    digitalWrite(pin_SS, LOW);
    SPI.transfer(code_I);
    delayMicroseconds(40);
    for (int i=0; i<2*Ns; i++) {
        ((uint8_t*) bufferI)[i] = SPI.transfer(0);
    }
    digitalWrite(pin_SS, HIGH);

    delayMicroseconds(20);

    digitalWrite(pin_SS, LOW);
    SPI.transfer(code_Q);
    delayMicroseconds(40);
    for (int i=0; i<2*Ns; i++) {
        ((uint8_t*) bufferQ)[i] = SPI.transfer(0);
    }
    digitalWrite(pin_SS, HIGH);

    delayMicroseconds(20);

    conf[5] &= 0b11011111; // The conf is not used anymore!
    /*SPI.endTransaction();*/
    return;

error:
    conf[5] |= 0b00100000; // Use the conf next time again
    /*SPI.endTransaction();*/
    return;
}

/*
Alternative 0 (the optimum one) -> Beware, destructive!

    SPI.transfer(bufferQ, 2*Ns)

Alternative 1

    for (i=0; i<Ns; i++) {
        bufferQ[i] = SPI.transfer(0);
        bufferQ[i] |= (SPI.transfer(0)<<8);
    }

Alternative 2

    for (i=0; i<Ns; i++) {
        temp = SPI.transfer16(0);
        bufferQ[i] = (temp>>8) | (temp<<8);
    }

Alternative 3

    for (i=0; i<2*Ns; i++) {
        ((uint8_t*) bufferQ)[i] = SPI.transfer(0);
    }

*/

class Array2D
{
private:
    const int np, ns;
    uint16_t **rows = NULL;

public:
    // Memory is fragmented on purpose, so that more memory can be allocated
    void init() {
        rows = new uint16_t*[np];
        for (int i=0; i<np; i++)
            rows[i] = new uint16_t[ns];
    }

    Array2D(int Np, int Ns) : np(Np), ns(Ns) {}

    ~Array2D() {
        for (int i=0; i<np; i++)
            delete[] rows[i];
        delete[] rows;
    }

    // Allocation is postponed so that memory is in heap, not stack
    uint16_t* operator[](int index) {
        if (rows == NULL) { init(); }
        return rows[index];
    }
};

/* -------------------------------------------------------------------------- */

Array2D bufferI(Np, Ns);
Array2D bufferQ(Np, Ns);
uint32_t *timesmed;
uint32_t kk=1;
int loopCount = 0;
uint8_t i;
void setup() {

    pinMode(pin_ON_OFF, OUTPUT);  	 // ON/OFF pin
    digitalWrite(pin_ON_OFF, LOW);	 // switch OFF uRAD
    pinMode(pin_SS, OUTPUT);  		 // Slave Select pin SPI
    digitalWrite(pin_SS, HIGH);      // HIGH: slave not selected

    Serial.begin(921600);   // serial port baud rate
    SPI.begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    delayMicroseconds(100);
    SPI.transfer(255);

    // load the configuration
    loadConfiguration(mode, f0, BW, Ns, Ntar, Rmax, MTI, Mth);

    digitalWrite(pin_ON_OFF, HIGH);  // switch ON uRAD
    delay(100); // recommended delay after powering ON uRAD
    detection(bufferI[0], bufferQ[0]);
    timesmed = (uint32_t*) malloc(Np);
}

void loop() {
  if (loopCount < Np) {
    //while (!Serial.available()) {};
    //char rcv = Serial.read();
    //if (rcv != 'm') return;

    for (int i=0; i<Np; i++) {
        timesmed[i] = micros();
        detection(bufferI[i], bufferQ[i]);
        loopCount++;
    }
   Serial.print(Np);
    Serial.print(" ");
    Serial.println(Ns);

    for (int i=0; i<Np; i++) {
        Serial.print("--> ");
        Serial.print( timesmed[i] );
        Serial.print(" (");
        if (i>0) {
          Serial.print( timesmed[i] - timesmed[i-1] );
        }
        Serial.println(")");
        for (int j=0; j<Ns; j++) {
          Serial.print( (uint16_t) (bufferI[i])[j]);
          Serial.print("-");
          Serial.print( (uint16_t) (bufferQ[i])[j]);
          Serial.print(" ");
        }
        Serial.println();
    

    }
}else {
  // Enviar datos a MATLAB
        enviarDatosAMatlab();

        // Otras acciones de preparación para enviar datos a MATLAB

        while (true) {
            // Mantener el programa en un bucle infinito para que no continúe ejecutándose
        }
}

 delay(1000);

}
void enviarDatosAMatlab() {
    // Enviar el número de muestras a MATLAB
    Serial.println();
    Serial.print("Ns: ");
    Serial.println(Ns);

    // Enviar los datos de bufferI a MATLAB
    Serial.print("bufferI: ");
    for (int row = 0; row < Np; row++) {
        for (int i = 0; i < Ns; i++) {
            Serial.print(bufferI[row][i]);
            Serial.print(" ");
        }
        
    }

    // Salto de línea entre bufferI y bufferQ
    Serial.println();

    // Enviar los datos de bufferQ a MATLAB
    Serial.print("bufferQ: ");
    for (int row = 0; row < Np; row++) {
        for (int i = 0; i < Ns; i++) {
            Serial.print(bufferQ[row][i]);
            Serial.print(" ");
        }
        
    }
    Serial.println();
    Serial.print("Tiempo: ");
for (int temp = 0; temp < Np; temp++) {
    if (temp == 0) {
        Serial.print(timesmed[temp]);
        Serial.print(" ");
    } else {
        Serial.print(timesmed[temp] - timesmed[temp - 1]);
        Serial.print(" ");
    }
}

    // Salto de línea antes de "END"
        Serial.println();
    Serial.print("END");
}
