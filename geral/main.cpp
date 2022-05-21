#include <iostream>

using namespace std;

// Controle de velocidade
// PWM 0-255
// Obs: unsigned int 2byte 0-65535
struct valoresVelocidade
{
    unsigned int velMotorEsquerdo;
    unsigned int velMotorDireito;
    // Velocidades motor esquerdo
    unsigned int maxE, minE, baseE;
    // Velocidades motor direito
    unsigned int maxD, minD, baseD;
};

// Controle de PID
struct valoresPID
{
    float entrada;
    float saida;
    // PID reta
    float SetpointR, KpR, KiR, KdR;
    // PID Curva
    float SetpointC, KpC, KiC, KdC;
};
struct valoresMarcasLaterais
{
    unsigned int qntEsqueda;
    unsigned int qntDireitra;
};
struct valoresEncoders
{
    unsigned int qntEsqueda;
    unsigned int qntDireitra;
};
struct valoresSensoresLaterias
{
    // São utilizados 1 sensor em cada lateral
    short int valorSensores[2];
    short int valorMin[5];
    short int valorMax[5];
    float mediaPonderada;
};
struct valoresSensoresArray
{
    // São utilizados 5 sensores laterais
    short int valorSensores[5];
    short int valorMin[5];
    short int valorMax[5];
    float mediaPonderada;
};

class Robo{
    public:
        float P = 0, I = 0, D = 0, ERRO = 0, erroAnterior = 0;
        unsigned int estado; // 0: Parado, 1: Linha, 2: Curva
        valoresVelocidade velocidade;
        valoresPID PID;
        valoresMarcasLaterais marcasLaterais;
        valoresEncoders encoders;
        valoresSensoresLaterias senLateral;
        valoresSensoresArray senArray;
        int ultimaAtualizacao = 0;

        // Inicia o robô em modo linha
        Robo(){ // Construtor
            estado = 1;
            ultimaAtualizacao = 0;
        }
        // Prototipos de funcao
        void vTaskCalcularPID(float *input, float *output); 

};

void Robo::vTaskCalcularPID(float *input, float *output){
    // Calculo com base na apostila de treinamento
    ERRO = *input;
    P = ERRO;
    I = I + ERRO;
    D = ERRO - erroAnterior;
    // PID Reta
    if(estado = 1){
        *output = (PID.KpR * P) + (PID.KiR * I) + (PID.KdR * D);
    }
    // PID Curva
    else if(estado = 2){
        *output = (PID.KpC * P) + (PID.KiC * I) + (PID.KdC * D);
    }
    erroAnterior = ERRO;

}

int main(){
    
}