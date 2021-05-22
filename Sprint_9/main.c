/*
 * Author : Pedro Henrique dos Santos Almeida.
 * Matrícula:118111866.
 */ 

#define F_CPU 16000000UL //Frequência de trabalho da CPU
#define __DELAY_BACKWARD_COMPATIBLE__
#define BAUD 9600
#define MYUNRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "nokia5110.h"

void animar_motor(uint8_t FreqRespiracao);
void executa_a_cada_150ms();
void executa_a_cada_200ms();
void alarme();
uint8_t verifica(char *comunicar, char recebido);
void analise_estado(uint8_t est, char *comunic);
void animar_oxig(uint8_t toxig);
void acend_led();

//Variáveis Globais

static uint8_t FreqRespiracao =8;//Representa as resp/min
static uint8_t taxa_oxig = 0;
static uint8_t tempo = 0;
static uint32_t Tempo_ms = 0;
static uint32_t FreqCard = 0;
static uint32_t i_tempo = 0;
static uint32_t f_tempo = 0;
static uint16_t Tempo_refer = 0;
uint8_t flags_150ms = 0, flags_200ms = 0, flag_t = 0;
static char str1[3], str2[4], str3[4], str4[4], str5[3],str6[4], str7[3], str8[3], str9[3], str10[3], str11[3], str12[3];
static float temperat = 0;
static uint8_t rest_temperat = 0;
static uint8_t satur = 0;
uint16_t leitura_ADC = 0;
static char Erro[8] = "ERRO!   ";
static char pressao_arterial[8] = "       ";
static char recebido;
static uint16_t valor_motor = 2000;
static uint8_t volume_ar = 1;
static uint8_t peep = 1;
static uint8_t dpressao = 0;
static uint8_t flag_confpcv = 0, flag_confvcv = 0;
static uint8_t bpm_max = 90;
static uint8_t bpm_min = 50;
static uint8_t sat_min = 60;

//Definindo estados da máquina de estados:
static uint8_t inicial = 0b0000;
static uint8_t H1 = 0b0001;
static uint8_t H2 = 0b0010;
static uint8_t H3 = 0b0011;
static uint8_t X = 0b0100;
static uint8_t L1 = 0b0101;
static uint8_t L2 = 0b0110;
static uint8_t L3 = 0b0111;
static uint8_t ult = 0b1000;
static uint8_t ponto_ponto = 0b1001;
static uint8_t erro = 0b1010;

ISR(INT0_vect){//Interrupção externa INT0; -D2
	if((flag_t == 0) || (flag_t == 1)){
		FreqRespiracao++;
	}else if(flag_t == 2){
		taxa_oxig = taxa_oxig + 10;
		taxa_oxig = (taxa_oxig > 100) ? 100 : taxa_oxig;
		animar_oxig(taxa_oxig);
	}else if(flag_t == 3){
		peep ++;
	}else if((flag_t == 4) && (flag_confvcv == 1)){
		volume_ar++;
		volume_ar = (volume_ar > 8) ? 8 : volume_ar;
	}else if((flag_t == 4) && (flag_confpcv == 1)){
		dpressao++;
		if(volume_ar > 1){
			volume_ar--;
		}else{
			volume_ar = 1;
		}
	}else if(flag_t == 5){
		bpm_max++;
	}else if(flag_t == 6){
		bpm_min++;
	}else if(flag_t == 7){
		sat_min++;
	}
	
}
ISR(INT1_vect){//Interrupção externa INT1; -D3
	if((flag_t == 0) || (flag_t == 1)){
		FreqRespiracao--;
	}else if(flag_t == 2){
		if(taxa_oxig != 0){
			taxa_oxig = taxa_oxig - 10;
			animar_oxig(taxa_oxig);
		}else{
			taxa_oxig = 0;
			animar_oxig(taxa_oxig);
		}
	}else if(flag_t == 3){
		peep --;
	}else if((flag_t == 4) && (flag_confvcv == 1)){
		if(volume_ar > 1){
			volume_ar--;
		}else{
			volume_ar = 1;
		}
	}else if((flag_t == 4) && (flag_confpcv == 1)){
		dpressao--;
		volume_ar++;
		volume_ar = (volume_ar > 8) ? 8 : volume_ar;
	}else if(flag_t == 5){
		bpm_max--;
	}else if(flag_t == 6){
		bpm_min--;
	}else if(flag_t == 7){
		sat_min--;
	}	
}

ISR(TIMER0_COMPA_vect){//interrupção do TC0 a cada 1ms
	Tempo_ms++;
	Tempo_refer = 60000/(FreqRespiracao*16);
	if(Tempo_ms % Tempo_refer == 0){        
		animar_motor(FreqRespiracao);
	}
	if(!(Tempo_ms % 150)){
		flags_150ms = 1;
	}
	if(!(Tempo_ms % 200)){
		flags_200ms = 1;
	}
}

ISR(PCINT2_vect){
	if((PIND & 0b01000000) == 0){
		if(flag_t == 0){
			flag_confpcv = 1;
			flag_confvcv = 0;
		}else if(flag_t == 1){
			flag_confpcv = 0;
			flag_confvcv = 1;
		}
	}else if(tempo == 0){
		if((PIND & 0b00010000) == 0){
			i_tempo = Tempo_ms;
			tempo++;
		}
	}else if((PIND & 0b00010000) == 0){
			f_tempo = Tempo_ms;
			FreqCard = (60000/(f_tempo - i_tempo));//Frequência em bpm
			tempo = 0;
	}
}

ISR(PCINT0_vect){
	
	static uint8_t sel = 0;
	if((PINB & 0b01000000) == 0){
		sel++;
	} 
	if(sel > 8){
		sel = 0;
		flag_t = sel;
		flag_confpcv = sel;
		flag_confvcv = sel;
	}else{
		flag_t = sel;
	}
	
}

ISR(ADC_vect){//Quando uma conversão é finalizada, é disparado uma interrupção.
	leitura_ADC = ADC;//Cópia do registrador para a variável
}

ISR(USART_RX_vect){
	static char comunica[8] = "        ";
	static char eficienciar = ' ';
	recebido = UDR0;
	static uint8_t estado;
	estado = verifica(comunica,recebido);
	analise_estado(estado, comunica);
}

uint8_t verifica(char *comunicar, char recebido){
	static uint8_t estado_atual = 0;
	static uint8_t proximo_estado = 0;
	static uint8_t i;
	char numero[11] = "0123456789";
	estado_atual = proximo_estado;
	
	if(estado_atual == inicial){
		if(recebido == ';'){
			proximo_estado = H1;
		}else{
			proximo_estado = erro;
		}
		i = 0;
	}else if(estado_atual == H1){
		
		proximo_estado = erro;
		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = H2;
			}
		}

		comunicar[i] = recebido;
		i++;
	}else if(estado_atual == H2){
		
		if(recebido == 'x'){
			proximo_estado = L1;
		}else{
			proximo_estado = erro;
		}

		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = H3;
			}
		}

		comunicar[i] = recebido;
		i++;
	}else if(estado_atual == H3){
		
		if(recebido == 'x'){
			proximo_estado = L1;
		}else{
			proximo_estado = erro;
		}
		
		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = X;
			}
		}
		comunicar[i] = recebido;
		i++;
	}else if(estado_atual == X){
		if(recebido == 'x'){
			proximo_estado = L1;
		}else{
			proximo_estado = erro;
		}
		comunicar[i] = recebido;
		i++;
	}else if(estado_atual == L1){
		proximo_estado = erro;
		
		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = L2;
			}
		}
		comunicar[i] = recebido;
		i++;
	}else if(estado_atual == L2){
		if(recebido == ':'){
			proximo_estado = ponto_ponto;
		}else{
			proximo_estado = erro;
		}
		comunicar[i] = ' ';
		
		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = L3;
				comunicar[i] = recebido;
			}
		}
		i++;
	}else if(estado_atual == L3){
		if(recebido == ':'){
			proximo_estado = ponto_ponto;
		}else{
			proximo_estado = erro;
		}
		comunicar[i] = ' ';

		for(int j = 0;j < 10;j++){
			if(recebido == numero[j]){
				proximo_estado = ult;
				comunicar[i] = recebido;
			}
		}
		i++;
	}else if(estado_atual == ult){
		if(recebido == ':'){
			proximo_estado = ponto_ponto;
		}else{
			proximo_estado = erro;
		}
	}else if(estado_atual == ponto_ponto){
		if(recebido == ';'){
			proximo_estado = H1;
		}else{
			proximo_estado = erro;
		}
		i = 0;
	}else if(estado_atual == erro){
		if(recebido == ';'){
			proximo_estado = H1;
		}else{
			proximo_estado = erro;
		}
		i = 0;
	}
	
	return proximo_estado;
}

void analise_estado(uint8_t est, char *comunic){
	if(est == erro){
		for(int k = 0; k < 7;k++){
			pressao_arterial[k] = Erro[k];
			comunic[k] = ' ';
		}
	}else if(est == ponto_ponto){
		for(int k = 0; k < 7;k++){
			pressao_arterial[k] = comunic[k];
			comunic[k] = ' ';
		}
	}
}

void executa_a_cada_150ms(){

	if(flags_150ms){
		float Vin = 5*(leitura_ADC/1024.);
		if(ADMUX == 0b01000000){
			if((Vin>=2) && (Vin<3.5)){
				temperat = 10*Vin + 10;
			}else if(Vin<2){
				temperat = 30;
			}else{
				temperat = 45;
			}
			ADMUX ^= 0b00000001;
			
		}else if(ADMUX == 0b01000001){
			satur = 0.123*leitura_ADC;
			ADMUX ^= 0b00000001;
		}
		
		flags_150ms = 0;
	}

}

void executa_a_cada_200ms(){
	if(flags_200ms){ 
		uint8_t result = 0;
		rest_temperat = temperat;
		result = ((temperat*10)-(rest_temperat*10));
		
		itoa(FreqRespiracao,str1,10);
		itoa(FreqCard,str2,10);
		itoa(satur,str3,10);
		itoa(rest_temperat,str4,10);
		itoa(result,str5,10);
		itoa(taxa_oxig,str6,10);
		itoa(volume_ar,str7,10);
		itoa(peep,str8,10);
		itoa(dpressao,str9,10);
		itoa(bpm_max,str10,10);
		itoa(bpm_min,str11,10);
		itoa(sat_min,str12,10);
		
		//De acordo com as flags, irá mudar de tela:
		
		if((flag_t == 0) && (flag_confpcv == 0) && (flag_confvcv == 0)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Conf: Modo V.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string("PCV(conf)*",1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string("VCV(conf)",1); //Escreve um texto do tamanho 1
	
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		
		if((flag_t == 1) && (flag_confvcv == 0) && (flag_confpcv == 0)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Conf: Modo V.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string("PCV(conf)",1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string("VCV(conf)*",1); //Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		
		
		if((flag_t == 0) && (flag_confpcv == 1) ){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("PCV: com M.A.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("* rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str9,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 2) && (flag_confpcv == 1)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("PCV: com M.A.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("* FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str9,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 3) && (flag_confpcv == 1)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("PCV: com M.A.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("* cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str9,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 4) && (flag_confpcv == 1)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("PCV: com M.A.",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str9,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("* cmH2O",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 1) && (flag_confvcv == 1)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("VCV:",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("* rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str7,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  Vol",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 2) && (flag_confvcv == 1) ){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("VCV: ",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("* FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str7,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  Vol",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 3) && (flag_confvcv == 1) ){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("VCV: ",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("* cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str7,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("  Vol",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if((flag_t == 4) && (flag_confvcv == 1)){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("VCV: ",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("__________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17
			nokia_lcd_write_string(str1,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("  rsp/min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 25); //Muda o cursos para a posição 0,25
			nokia_lcd_write_string(str6,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 25); //Muda o cursos para a posição 30,25
			nokia_lcd_write_string("  FiO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33
			nokia_lcd_write_string(str8,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 33); //Muda o cursos para a posição 30,33
			nokia_lcd_write_string("  cmH2O",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(str7,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 40); //Muda o cursos para a posição 30,40
			nokia_lcd_write_string("* Vol",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
			
		}
		
		if(flag_t == 5){
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Conf. Alerta",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string(str10,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string("*bpm_max",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string(str11,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 26); //Muda o cursos para a posição 30,26
			nokia_lcd_write_string(" bpm_min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 36); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string(str12,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 36); //Muda o cursos para a posição 30,36
			nokia_lcd_write_string(" %SpO2",1);//Escreve um texto do tamanho 1
			
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		if(flag_t == 6){
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Conf. Alerta",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string(str10,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string(" bpm_max",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string(str11,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 26); //Muda o cursos para a posição 30,26
			nokia_lcd_write_string("*bpm_min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 36); //Muda o cursos para a posição 0,36 ou seja, pula uma linha
			nokia_lcd_write_string(str12,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 36); //Muda o cursos para a posição 30,36
			nokia_lcd_write_string(" %SpO2",1);//Escreve um texto do tamanho 1
			
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		if(flag_t == 7){
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Conf. Alerta",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string(str10,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 17); //Muda o cursos para a posição 30,17
			nokia_lcd_write_string(" bpm_max",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string(str11,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 26); //Muda o cursos para a posição 30,26
			nokia_lcd_write_string(" bpm_min",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 36); //Muda o cursos para a posição 0,36 ou seja, pula uma linha
			nokia_lcd_write_string(str12,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(30, 36); //Muda o cursos para a posição 30,36
			nokia_lcd_write_string("*%SpO2",1);//Escreve um texto do tamanho 1
			
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		
		if(flag_t == 8){
			
			nokia_lcd_clear(); //Limpa o LCD
			
			nokia_lcd_set_cursor(0, 0); //Muda o cursos para a posição 0,0
			nokia_lcd_write_string("Sinais Vitais",1);
			nokia_lcd_set_cursor(0, 8); //Muda o cursos para a posição 0,8
			nokia_lcd_write_string("____________",1);
			nokia_lcd_set_cursor(0, 17); //Muda o cursos para a posição 0,17 ou seja, pula uma linha
			nokia_lcd_write_string(str2,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(44, 17); //Muda o cursos para a posição 44,15
			nokia_lcd_write_string("bpm",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 26); //Muda o cursos para a posição 0,26 ou seja, pula uma linha
			nokia_lcd_write_string(str3,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(44, 26); //Muda o cursos para a posição 44,26
			nokia_lcd_write_string("%SpO2",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 33); //Muda o cursos para a posição 0,33 ou seja, pula uma linha
			nokia_lcd_write_string(str4,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(15, 33); //Muda o cursos para a posição 15,33 ou seja, pula uma linha
			nokia_lcd_write_string(",",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(20, 33); //Muda o cursos para a posição 20,33 ou seja, pula uma linha
			nokia_lcd_write_string(str5,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(44, 33); //Muda o cursos para a posição 44,33
			nokia_lcd_write_string("°C",1);//Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(0, 40); //Muda o cursos para a posição 0,40
			nokia_lcd_write_string(pressao_arterial,1); //Escreve um texto do tamanho 1
			nokia_lcd_set_cursor(44, 40); //Muda o cursos para a posição 44,40
			nokia_lcd_write_string("mmHg",1);//Escreve um texto do tamanho 1
			
			nokia_lcd_render(); //Atualiza a tela do display com o conteúdo do buffer
		}
		
		flags_200ms = 0;
	}
}

void USART_init(unsigned int ubrr){
	UBRR0H = (unsigned char)(ubrr>>8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = 0b10011000;  //Habilita o transmissor e o receptor
	UCSR0C = 0b00000110;  //Sem paridade, 1 bit de parada, 8 bits de dado
}

int main(void)
{
    //Registradores de Direção:
	
	DDRB = 0b10111111; //Habilitados quase todos os pinos B como saída, só B6 como entrada
	DDRC = 0b11111100; //Habilita os pinos C0 e C1 como entradas e o resto como saídas
	DDRD = 0b10100011; //Habilita os pinos D3, D2, D4 e D6 como entrada. 
	PORTD = 0b01011100; //Aciona o pull-up interno dos pinos D3, D2 e D4, D6
	PORTC = 0b00000000; //Aciona o pull-up internos dos pinos C0 e C1
	PORTB = 0b01111000; //Habilitando o pull-up do pino B6,B5,B4,B3.
	
	
	//Configuração de interrupção:
	
	TCCR0A = 0b00000010;//TC0 operando em modo CTC
	TCCR0B = 0b00000011;//Liga TC0 com prescaler = 64
	OCR0A  = 249;		//TC0 conta até 249
	TIMSK0 = 0b00000010;//Habilita interrupção por comparação com OCR0A. A interrupção ocorerá a cada 1ms = (64*(249+1))/16MHz
	
	
	EICRA = 0b00001010; //interrupção externa INT0 e INT1 na borda de descida
	EIMSK = 0b00000011; //Habilitada as interrupção externas INT0 e INT1;
	PCICR = 0b00000101;
	PCMSK2 = 0b01010000;
	PCMSK0 = 0b01000000;

	
	//Configuração AD
	ADMUX = 0b01000001; //Tensão Vcc, canal 0;
	//ADCSRA = 0b00000111;
	ADCSRA = 0b11101111;//Habilita o AD, a interrupção, modo de conversão contínua, prescaler - 128
	ADCSRB = 0b00000000;//modo de conversão contínua
	DIDR0 = 0b00111100; //habilita C0 como entrada do ADC
	
	// Configuração PWM
	//TIMER TC1 - modo PWM rápido via ICR1, prescaler = 8
	//TOP = (F_CPU/(Pre*F_PWM)) - 1, com Pre = 8 e F_PWM = 50Hz , TOP = 39999;
	ICR1 = 39999; // Configura o período do PWM (20ms)
	TCCR1A = 0b10100010; // modo PWN rápido via ICR1, ativa o PWN no OC1B n-invert e o OC1A
	TCCR1B = 0b00011010; //prescaler = 8;
	
	OCR1A = 2000; //1ms
	OCR1B = 2000; //1ms
	
	// Configuração USART
	USART_init(MYUNRR);
	sei(); // habilita interrupções globais, ativando o bit I do SREG
	
	nokia_lcd_init(); //Inicia o LCD
	
	
    while (1) 
    {
		FreqRespiracao = (FreqRespiracao <= 4) ? 5 : FreqRespiracao;
		FreqRespiracao = (FreqRespiracao >= 31) ? 30 : FreqRespiracao;
		peep = (peep <= 1) ? 1 : peep;
		peep = (peep >= 5) ? 5 : peep;
		dpressao = (dpressao < 1) ? 0 : dpressao;
		dpressao = (dpressao > 50) ? 50 : dpressao;
		
		executa_a_cada_150ms();
		executa_a_cada_200ms();
		alarme();
		acend_led();
    }
}

void animar_motor(uint8_t FreqRespiracao){
	static uint8_t contador = 0;
	if(contador < volume_ar){
		valor_motor = valor_motor + 250;
		contador++;
	}else{
		valor_motor = valor_motor - 250;
		if(valor_motor == 2000){
			contador = 0;
		}
	}
	OCR1A = valor_motor;
}

void animar_oxig(uint8_t toxig){
	OCR1B = (20*toxig)+2000;
}

void alarme(){ // Função para descrever o que acontecerá se acontecer alguma anormalidade
	if(flag_t > 7){
		if((temperat < 35) || (temperat > 41) ||(satur < sat_min) || (FreqCard < bpm_min) || (FreqCard > bpm_max)){
			PORTD |= 0b10000000;
			PORTB &= 0b11110111; //Não tem tensão no pino
		}else{
			PORTD &= 0b01111111;
			PORTB |= 0b00001000;
		}
	}else{
		PORTB |= 0b00001000;
	}
}

void acend_led(){ // Função Para indicar o modo ventilatório
	if(flag_confpcv == 1){
		PORTB |= 0b00010000; //B4
	}else{
		PORTB &= 0b11101111;
	}
	if(flag_confvcv == 1){
		PORTB |= 0b00100000;//B5
	}else{
		PORTB &= 0b11011111;
	}
}


