
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *krq_iqs = 0.050283838730420805, kxq_iqs = -0.049808411320704515, kxq_we = 1.910422822440357e-5, kxq_ids = 0.0, kzq_ziqs = 5.0233605125295506e-5
  *krd_ids = 5.466122340265619, kxd_iqs = 0.0, kxd_ids = -5.414440836803701, kzd_zids = 4.969202127514199
  *krw_we = 0.0001974932315859564, kxw_iqs = 0.8135612263744455, kxw_we = 0.00019736068330049696, kzw_zwe = 0.00012695993459097197, kzw_ziqs = -2.2265988790233272e-8
  *iqs_ref1 = krw_we*we_ref1+kxw_iqs*iqs0+kxw_we*we0+kzw_ziqs*ziqs0+kzw_zwe*zwe0
  *vqs0 = krq_iqs*iqs_ref1 + kxq_iqs*iqs0 + kxq_ids*ids0 + kxq_we*we0 + kzq_ziqs*ziqs0
  *vds0 = krd_ids*ids_ref1 + kxd_iqs*iqs0 + kxd_ids*ids0 + kzd_zids*zids0
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "gpio.h"
//#include "math.h"

//Declaração de REGISTRADORES
register float r0 asm("r0");
register float r1 asm("r1");
register float r2 asm("r2");
register float r3 asm("r3");
register float r4 asm("r4");
register float r5 asm("r5");

register float s0 asm("s0");
register float s1 asm("s1");
register float s2 asm("s2");
register float s3 asm("s3");
register float s4 asm("s4");
register float s5 asm("s5");
register float s6 asm("s6");
register float s7 asm("s7");
register float s8 asm("s8");
register float s9 asm("s9");
register float s10 asm("s10");
register float s11 asm("s11");
register float s12 asm("s12");
register float s13 asm("s13");
register float s14 asm("s14");
register float s15 asm("s15");
register float s16 asm("s16");
register float s17 asm("s17");
register float s18 asm("s18");
register float s19 asm("s19");
register float s20 asm("s20");
//SENO, COSSENO & DQ
volatile float asm_theta = 0.0;
volatile float asm_a = 0.0;
volatile float asm_b = 0.0;
volatile float asm_c = 0.0;
volatile float asm_d = 0.0;
volatile float asm_q = 0.0;
volatile float asm_operando = 0.0;
volatile float asm_temp = 0.0;

float cos_oe, sin_oe, ialpha_lido, ibeta_lido, Ids_lido, Iqs_lido;
//VELOCIDADE E POSIÇÃO
float wrad=0, wrpm=0;
float wm_calc = 0.0, wm_filt = 0.0, wm_filt0 = 0.0, theta_e0 = 0.0;
float wm_calc_ant = 0.0;
float posicao_minima;
int Timer_Enconder_Value = 0, Timer_Enconder_Value0 = 0;
float alpha_2ord;
//float alpha=0.1, wf=0;
//float alpha=0.08, wf=0;
float alpha=0.025, wf=0;
//float alpha=0.008, wf=0;
//float alpha=0.004, wf=0;
float wrad0 = 0.0, wlim = 15.0, dtheta = 0.0;
double theta=0, theta0=0;//pulsos=0,
float PPR_Encoder = 1024.0,t_rampa,we_ref_rampa, rampa_time, tempo_espera, current_we_ref, flag_parada = 0.0;
int kk = 0, Div_ts = 20;
int grau_e, nvoltas_grau_e;
float theta_a, theta_b, theta_c, freq_ref;
float theta_e = 0.0;
float theta_ant = 0.0, dthetax = 0, theta_e_ant = 0.0;
//CONSTANTES
int polos = 42, polos_div_2 = 21;//, Numero_voltas_encoder_por_volta = 2;
float ts=0.0001; //PWM 10kHz 0.0002 PARA PWM 5kHz
//float ts=0.0002; //PWM 5kHz
//float ts=0.0004; //PWM 2.5kHz
float pi=3.14159265358979323846;
float Imax;
//int TIM1_Period = 1199;
//VARIÁVEIS
long int longin = 0;
long int  flag_tim3=1, flag_tim4=0, temp1,temp2,flag_rampa,flag_rampa_max,temp3 = 0, final_time_flag, flag_final = 0;
int flag_protecao = 1, flag_para = 1, stop_flag = 1;
float  ampA, ampB, ampC;//w = 5,
int dutya=0,dutyb=0,dutyc=0;
float ias_lido=0.0, ibs_lido = 0.0, ics_lido=0.0, Ia_bits=0, Ib_bits=0, Ic_bits=0;
//VARIAVEIS DE CONTROLE
float z_ids = 0, z_we = 0, z_iqs = 0, z_wf = 0.0;
float wm_ref, we_ref, ids_ref = 0.0, iqs_ref = 0.0;
float wm_ref2, we_ref2;
float wm_ref3, we_ref3;
float wm_ref4, we_ref4;
float wm_ref5, we_ref5;
float we_lido, we_real = 0.0, oe_real = 0.0, vqs, vds, valpha, vbeta, vas, vbs, vcs, Vdc;
float vmax, vmin;
float fator_divisao_ma;
//GANHOS CONTROLADOR

float tk = 0.0, vp_ref = 0.0, vp =0.0, dv=0.0, Fluxo_alpha = 0.0, Fluxo_beta = 0.0, Fluxo =0.0,zoe = 0.0;
float erro_fluxo = 0.0, erro_fluxo_a = 0.0;
//float Fluxo_ref = 0.6, Rs = 4.5, kvf = 1.5, v0 = 0.0, kp_fluxo = 3.5, ki_fluxo = 0.005;
float Fluxo_ref = 0.8, Rs = 4.5, kvf = 1.5, v0 = 0.0, kp_fluxo = 10.5, ki_fluxo = 1.5;

float fpm = 0.19, ParPolos = 21, np = 21, lqs = 0.05, lds = 0.05, jm = 0.00115, bm = 0.0575, rs = 4.5, las = 0.0333;
float est_b, est_a, est_c, lsliding, ksliding, wcsliding, ksliding_ref, E0;
float kp_pll, ki_pll, xipll, wcpll, wcpll2;

//variaveis slinding mode e pll
float ialpha_est = 0.0, ibeta_est = 0.0, ealpha_slide = 0.0, ebeta_slide = 0.0, valpha_ref = 0.0, vbeta_ref = 0.0, ialpha_ref =0.0, ibeta_ref = 0.0;
float ealpha_filt = 0.0, ebeta_filt = 0.0, ealpha_est = 0.0, ebeta_est = 0.0, oe_est = 0.0, erro_oe_est_pll = 0.0;
float we_pll = 0.0, we_pll_filt1 = 0.0, we_pll_filt = 0.0, we_est = 0.0;
float oe_est_pll = 0.0, erro_a_oe_est_pll = 0.0, ealphabeta_abs = 0.0;

//variaveis controle vetorial
float oe_lido = 0.0, cos_oe_lido = 0.0, sin_oe_lido = 0.0, iqs_lido = 0.0, ids_lido = 0.0;
float erro_wm = 0.0, wm_ref_rampa = 0.0, wm_lido = 0.0, zwe = 0.0, te_ref = 0.0, erro_we = 0.0;
float erro_iqs = 0.0, ziqs = 0.0, vqs_ref = 0.0, erro_ids = 0.0, zids = 0.0, vds_ref = 0.0, vdc = 0.0;
float kp_wm, ki_wm, kp_iqs, ki_iqs, kp_ids, ki_ids;

//variaveis preditivo e realimentacao de estados
float kr_we,  kx_iqs, kx_we, kz_we, kr_ids, kx_ids, kz_ids;
float b31, a31, a12,a33, a32i, b22, a22, b11,a11, b32i, b12w, b21w, a12w, a21w, a51, a13;
float mu_zids, rho_vds, mu_tune, rho_tune, mu_zwe, rho_vqs;

//variaveis fcs MSIP
float tl=0, const_iq=0.0,const_id=0.0;
int s1_ant, s2_ant, s3_ant ,s1c,s2c,s3c;
float Jmin=100000000000000;
int s1_min=0,s2_min=0,s3_min=0;
float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c3,tl2;
float const_iq2,const_id2;
int s1c2,s2c2,s3c2;
float iq_c2, id_c2, te_c2, wm_c2,zwetemp,pho;

float flag_transicao = 0.0, transicao_time, monitoramento_flag_transicao;
// funçoes
float sw1,sw2,sw3;
float va,vb,vc;
//com integrador e sem compensacao (rotacao: qsi = 0.95, wn = 0.8bm/jm, correntes: ru = 100, rw = 0.001)
float kr_we_we = 0.030084267610605207;
float kx_we_we = 0.029934221699924397;
float kz_we_we = 0.00018446038333317394;
float kr_vds_iqs = 0.0;
float kr_vqs_iqs = 9.762826334321593;
float kr_vqs_ids = 0.0;
float kr_vds_ids = 9.78231417632;
float kx_vqs_iqs = 9.582699900789686;
float kx_vqs_ids = 0.0;
float kx_vqs_we = -0.005470710968279944;
float kx_vds_iqs = 0.0;
float kx_vds_ids = 9.651672596130698;
float kx_vds_we = 0.0;
float kz_vqs_iqs = 0.014664029434968907;
float kz_vqs_ids = 0.0;
float kz_vds_iqs = 0.0;
float kz_vds_ids = 0.014683085639884614;


//Controle com compensação de rotação
float kr_we_comp = 0.06294086130764234;
float kx_we_comp = 0.06262694245223999;
float kz_we_comp = 0.00034874541223928196;
float kr_iqs_comp = 4.975267465833738;
float kx_iqs_comp = 4.9306909538362715;
//float kz_iqs_comp = 0.0024863905376480452;
float kz_iqs_comp = 0.00;
float kr_ids_comp = 4.975267465833738;
float kx_ids_comp = 4.9306909538362715;
float kz_ids_comp = 0.0024863905376480452;

float ry1 = 1;
float rw1 = 0.1;
float ru1 = 50;

float ry2 = 1;
float rw2 = 0.1;
float ru2 = 100;

//float erro_fluxo = 0.0, erro_fluxo_a = 0.0, kp_fluxo = 0.05, ki_fluxo = 1.2;

int cont_we = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int sign(float x){
	int y = 0;
	if (x>0)
		y = 1;
	if (x< 0)
		y = -1;
	if (x == 0)
		y = 0;
	return y;
}
float absx(float x){
	float y;
	if (x>=0)
		y = x;
	if (x< 0)
		y = -x;
	return y;
}
float atan2_approximation1(float y, float x)
{
    //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
    //Volkan SALMA
	float M_PIx = 3.14159265359;
    float ONEQTR_PI = M_PIx / 4.0;
	float THRQTR_PI = 3.0 * M_PIx / 4.0;
	float r, angle;
	float abs_y = absx(y) + 1e-10f;      // kludge to prevent 0/0 condition
	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}
	angle += (0.1963f * r * r - 0.9817f) * r;
	if ( y < 0.0f )
		return( -angle );     // negate if in quad III or IV
	else
		return( angle );


}
void inicia_float(){
	asm("LDR.W R0, =0xE000ED88");
	asm("LDR R1, [R0]");
	asm("ORR R1, R1, #(0xF << 20)");
	asm("STR R1, [R0]");
	asm("DSB");
	asm("ISB");

}
float asm_sqrt(){
	float out;
	s0 = asm_operando;
	asm(" VSQRT.F32 s1,s0");
	out = s1;
	return out;

}
float asm_sin(){

	float out;
	r0 = asm_theta;
	r1 = -1.0/6.0;
	r2 = 1.0/120.0;
	r3 = -1.0/5040.0;
	r4 = 1.0/362880.0;


	asm(" VMOV s15,r0");
	asm(" VMOV s14,r1");
	asm(" VMOV s13,r2");
	asm(" VMOV s12,r3");
	asm(" VMOV s11,r4");

	/*cubo-> salva em s16*/
	asm(" VMUL.F32 s16,s15,s15 ");
	asm(" VMUL.F32 s16,s16,s15 ");

	/*quíntico-> salva em s17*/
	asm(" VMUL.F32 s17,s16,s15 ");
	asm(" VMUL.F32 s17,s17,s15 ");

	/*sétima potência-> salva em s18*/
	asm(" VMUL.F32 s18,s17,s15 ");
	asm(" VMUL.F32 s18,s18,s15 ");

	/*nona potência-> salva em s19*/
	asm(" VMUL.F32 s19,s18,s15 ");
	asm(" VMUL.F32 s19,s19,s15 ");

	/*multiplica por -1/3!*x^3)*/
	asm(" VMUL.F32 s16,s14,s16");
	/*multiplica por 1/5!*x^5)*/
	asm(" VMUL.F32 s17,s13,s17");
	/*multiplica por -1/7!*x^7)*/
	asm(" VMUL.F32 s18,s12,s18");
	/*multiplica por -1/9!*x^9)*/
	asm(" VMUL.F32 s19,s11,s19");

	/*adiciona x-1/3!*x^3*/
	asm(" VADD.F32 s15,s16");
	/*adiciona x-1/3!*x^3+1/5!*x^5*/
	asm(" VADD.F32 s15,s17");
	/*adiciona x-1/3!*x^3+1/5!*x^5-1/7!*x^7*/
	asm(" VADD.F32 s15,s18");
	/*adiciona x-1/3!*x^3+1/5!*x^5-1/7!*x^7+1/9!*x^9*/
	asm(" VADD.F32 s15,s19");



	/*asm(" VMOV r1,s15 ");*/

	out = s15;

	//out = theta1*theta2+0.46;

	return out;
}
float asm_cos(){

	float out;
	r0 = asm_theta;
	r1 = -1.0/2.0;
	r2 = 1.0/24.0;
	r3 = -1.0/720.0;
	r4 = 1.0/40320.0;
	r5 = -1.0/3628800.0;



	asm(" VMOV s15,r0");
	asm(" VMOV s14,r1");
	asm(" VMOV s13,r2");
	asm(" VMOV s12,r3");
	asm(" VMOV s11,r4");
	asm(" VMOV s10,r5");

	/*quadrado-> salva em s16*/
	asm(" VMUL.F32 s16,s15,s15 ");

	/*quártico-> salva em s17*/
	asm(" VMUL.F32 s17,s16,s16 ");

	/*sexta potência-> salva em s18*/
	asm(" VMUL.F32 s18,s17,s16 ");

	/*oitava potência-> salva em s19*/
	asm(" VMUL.F32 s19,s18,s16 ");

	/*decima potência-> salva em s20*/
	asm(" VMUL.F32 s20,s19,s16 ");


	/*multiplica por -1/2!*x^2)*/
	asm(" VMUL.F32 s16,s14,s16");
	/*multiplica por 1/4!*x^4)*/
	asm(" VMUL.F32 s17,s13,s17");
	/*multiplica por -1/6!*x^6)*/
	asm(" VMUL.F32 s18,s12,s18");
	/*multiplica por -1/8!*x^8)*/
	asm(" VMUL.F32 s19,s11,s19");
	/*multiplica por -1/10!*x^10)*/
	asm(" VMUL.F32 s20,s10,s20");

	r1 = 1.0;
	asm(" VMOV s15,r1");
	/*adiciona x-1/2!*x^2*/
	asm(" VADD.F32 s15,s16");
	/*adiciona x-1/2!*x^2+1/4!*x^4*/
	asm(" VADD.F32 s15,s17");
	/*adiciona x-1/2!*x^2+1/4!*x^4-1/6!*x^6*/
	asm(" VADD.F32 s15,s18");
	/*adiciona x-1/2!*x^2+1/4!*x^4-1/6!*x^6+1/8!*x^8*/
	asm(" VADD.F32 s15,s19");
	/*adiciona x-1/2!*x^2+1/4!*x^4-1/6!*x^6+1/8!*x^8*/
	asm(" VADD.F32 s15,s20");



	/*asm(" VMOV r1,s15 ");*/

	out = s15;

	//out = theta1*theta2+0.46;

	return out;
}
void asm_transf_dq()
{
	s1 = asm_cos();
	s2 = asm_sin();

	s10 = asm_a;
	s11 = asm_b;
	s12 = asm_c;

	s20 = 2.0;
	s19 = 3.0;
	s18 = 0.5773502692;

	asm(" VADD.F32 s3,s11,s12");
	asm(" VDIV.F32 s3,s3,s20");
	asm(" VSUB.F32 s3,s10,s3");
	asm(" VMUL.F32 s3,s3,s20");
	asm(" VDIV.F32 s3,s3,s19"); //alpha

	asm(" VSUB.F32 s4,s11,s12");
	asm(" VMUL.F32 s4,s4,s18"); //beta

	asm(" VMUL.F32 s5,s1,s3");
	asm(" VMUL.F32 s6,s2,s4");
	asm(" VADD.F32 s5,s5,s6"); //eixo de quadratura

	asm(" VMUL.F32 s6,s3,s2");
	asm(" VMUL.F32 s7,s1,s4");
	asm(" VSUB.F32 s6,s7,s6");//eixo direto

	asm_q = s5;
	asm_d = s6;
}
void inicia_interrupcao(){
	HAL_TIM_Base_Start_IT(&htim3);
//	HAL_TIM_Base_Start_IT(&htim4);
	return;
}
void para_interrupcao(){
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim4);
	return;
}
void inicia_pwm(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//fase a
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//fase b
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//fase c
	return;
}
void inicia_encoder(){
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	return;
}
void inicia_dac(){
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
	return;
}
void le_corrente(){
	HAL_ADC_Start(&hadc1);
	Ia_bits = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Start(&hadc2);
	Ib_bits = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Start(&hadc3);
	Ic_bits = HAL_ADC_GetValue(&hadc3);

	ias_lido = -3.8752362381*((0.0006930339*(Ia_bits))) + 6.6254204774 + 0.125;
	ibs_lido = -3.8752362381*((0.0006930339*(Ib_bits))) + 6.6254204774 + 0.125;
	ics_lido = -3.8752362381*((0.0006930339*(Ic_bits))) + 6.6254204774 + 0.125;
}
void le_corrente_corrigida(){
	HAL_ADC_Start(&hadc1);
	Ia_bits = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Start(&hadc2);
	Ib_bits = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Start(&hadc3);
	Ic_bits = HAL_ADC_GetValue(&hadc3);


	//ias_lido = -0.0028*Ia_bits+5.9508;
	//ibs_lido = -0.0028*Ib_bits+5.9508;
	//ics_lido = -0.0028*Ic_bits+5.9508;

	ias_lido = -3.8752362381*((0.0006930339*(Ia_bits))) + 6.6254204774 + 0.125;
	ibs_lido = -3.8752362381*((0.0006930339*(Ib_bits))) + 6.6254204774 + 0.125;
	ics_lido = -3.8752362381*((0.0006930339*(Ic_bits))) + 6.6254204774 + 0.125;

//	ias_lido = -3.8752362381*((0.0006930339*(Ia_bits))) + 6.6254204774 + 0.125-1.0;
//	ibs_lido = -3.8752362381*((0.0006930339*(Ib_bits))) + 6.6254204774 + 0.125-1.0;
//	ics_lido = -3.8752362381*((0.0006930339*(Ic_bits))) + 6.6254204774 + 0.125-1.0;

}
void escreve_dac1(int valor_inteiro){
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,valor_inteiro);
	return;
}
void escreve_dac2(int valor_inteiro){
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,valor_inteiro);
	return;
}
void escreve_dutya(int new_duty_cycle){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_duty_cycle);
	return;
}
void escreve_dutyb(int new_duty_cycle){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, new_duty_cycle);
	return;
}
void escreve_dutyc(int new_duty_cycle){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, new_duty_cycle);
	return;
}
void protecao_corrente(){

	if((ias_lido > Imax)&&(kk>10))
		flag_protecao = 0;

	if((ibs_lido > Imax)&&(kk>10))
		flag_protecao = 0;

	if((ics_lido > Imax)&&(kk>10))
		flag_protecao = 0;

	if((ias_lido < -Imax)&&(kk>10))
			flag_protecao = 0;

	if((ibs_lido < -Imax)&&(kk>10))
			flag_protecao = 0;

	if((ics_lido < -Imax)&&(kk>10))
			flag_protecao = 0;
}
void trata_sobrecorrente(){

	  if (flag_protecao == 0){
		  dutya = 1198;
		  dutyb = 1198;
		  dutyc = 1198;

		  escreve_dutya(dutya);
		  escreve_dutyb(dutyb);
		  escreve_dutyc(dutyc);

		  para_interrupcao();

	  }
}
void ajuste_theta0(){
	  dutya = 1180;
	  dutyb = 1198;
	  dutyc = 1198;

	  escreve_dutya(dutya);
	  escreve_dutyb(dutyb);
	  escreve_dutyc(dutyc);

	  for(int i=0;i<8;i++){
	  //protecao_corrente();
	  //le_corrente();
	  le_corrente_corrigida();
	  HAL_Delay(250);
	  }
}
void reinicializa_encoder(){
	  Timer_Enconder_Value = 0;
	  Timer_Enconder_Value0 = 0;
	  __HAL_TIM_SET_COUNTER(&htim5,0);
}
void medicao_velocidade(){
	Timer_Enconder_Value = __HAL_TIM_GET_COUNTER(&htim5);
	theta = Timer_Enconder_Value*pi/(2*PPR_Encoder);
	theta_e = polos_div_2*theta;
	grau_e = (int)(180/pi*theta_e);

	//correção para deixar theta entre -pi e pi
	if (theta > pi)
		theta = theta-2*pi;
	if (theta < -pi)
		theta = theta+2*pi;

	//correção para perda de ângulo
	if (theta > pi)
		theta = theta_ant;
	if (theta < -pi)
		theta = theta_ant;

	//correção para deixar theta_e entre -pi e pi
	if (grau_e >= 360){
	   nvoltas_grau_e = grau_e/360;
	   theta_e = theta_e-2*pi*nvoltas_grau_e;
	   if(theta_e > pi)
		   theta_e -= 2*pi;
	}
	if (grau_e < 0){
	   nvoltas_grau_e = grau_e/360+1;
	   theta_e = theta_e+2*pi*nvoltas_grau_e;
	   if(theta_e < -pi)
	   	   theta_e += 2*pi;
	}

	//correção para perda de ângulo
	if(theta_e > pi)
		theta_e = theta_e_ant;
	if(theta_e < -pi)
		theta_e = theta_e_ant;

	longin++;
	if(longin >= Div_ts){
		dtheta = (theta-theta0);
	   wrad = dtheta/(ts*Div_ts);
	 if (Timer_Enconder_Value < 2*PPR_Encoder/15.0 && Timer_Enconder_Value0 > 2*PPR_Encoder-2*PPR_Encoder/15.0){
		wrad = wrad0;
	  }
	 if (Timer_Enconder_Value > 2*PPR_Encoder-2*PPR_Encoder/15.0 && Timer_Enconder_Value0 < 2*PPR_Encoder/15.0){
		 wrad = wrad0;
	 }
	 if (wrad-wrad0 > 3.0)
	 	wrad = wrad0;
	 if (wrad-wrad0 < -3.0)
	 	wrad = wrad0;

	   wrpm = wrad*30/pi;
	   wrad0 = wrad;
	   wf = alpha*wrad + (1-alpha)*wf;
	   //wf = wrad;




	   theta0 = theta;
	   Timer_Enconder_Value0 = Timer_Enconder_Value;
	   longin=0;
	  }

 theta_ant = theta;
 theta_e_ant = theta_e;
 we_real = polos_div_2*wrad;
 oe_real = theta_e;
 //we_lido = polos_div_2*wf;

}
void filtro_velocidade_1ord(){

	wm_filt = alpha*wm_calc+(1-alpha)*wm_filt;

}
void filtro_velocidade_2ord(){

	wm_filt0 = alpha_2ord*wm_calc+(1-alpha_2ord)*wm_filt0;
	wm_filt = alpha_2ord*wm_filt0+(1-alpha_2ord)*wm_filt;

}
void medicao_velocidade_corrigido(){
	Timer_Enconder_Value = __HAL_TIM_GET_COUNTER(&htim5);
	if ((Timer_Enconder_Value0 > 4*PPR_Encoder-500) && (Timer_Enconder_Value < 500)){
		//Timer_Enconder_Value0 = -1;
		Timer_Enconder_Value0 = Timer_Enconder_Value0-4*PPR_Encoder;
	}

	if ((Timer_Enconder_Value0 < 500) && (Timer_Enconder_Value > 4*PPR_Encoder-500)){
			//Timer_Enconder_Value0 = -1;
			Timer_Enconder_Value0 = 4*PPR_Encoder-Timer_Enconder_Value0;
	}

	wm_calc = posicao_minima*(Timer_Enconder_Value-Timer_Enconder_Value0)/(ts); //o contador vai até o dobro da PPR


	//filtro_velocidade_1ord();
	filtro_velocidade_2ord();
//	if ((current_we_ref > 21*8.0)||(current_we_ref < -21*8.0))
//		filtro_velocidade_1ord();
//	else
//		filtro_velocidade_2ord();




	if (wm_filt - wm_calc_ant > 10)
		wm_filt = wm_calc_ant;
	if (wm_filt - wm_calc_ant < -10)
		wm_filt = wm_calc_ant;

	wm_calc_ant = wm_filt;

	//we_real = polos_div_2*wm_calc;
	we_real = polos_div_2*wm_filt;

	theta = Timer_Enconder_Value*posicao_minima;
	if (theta > pi)
		theta -= 2*pi;
	if (theta <= -pi)
		theta += 2*pi;


	theta_e = polos_div_2*Timer_Enconder_Value*posicao_minima + pi/2;
	grau_e = (int)(180/pi*theta_e);

	if (grau_e >= 360){
	   nvoltas_grau_e = grau_e/360;
	   theta_e = theta_e-2*pi*nvoltas_grau_e;
	}
	else{
		if (grau_e < 0){
		   nvoltas_grau_e = grau_e/360+1;
		   theta_e = theta_e+2*pi*nvoltas_grau_e;
		}
	}

	if (theta_e > pi)
		theta_e = theta_e-2*pi;
	else{
		if (theta < -pi)
			theta_e = theta_e+2*pi;
	}

	if (theta_e > pi)
		theta_e = theta_e0;
	else{
		if (theta < -pi)
			theta_e = theta_e0;;
	}

	theta_e0 = theta_e;
	Timer_Enconder_Value0 = Timer_Enconder_Value;

	oe_real = theta_e;

}
void estimador_sliding_mode(){

			asm_operando = ealpha_est*ealpha_est+ebeta_est*ebeta_est;
		    ealphabeta_abs = asm_sqrt();
		    if (ealphabeta_abs < E0)
		        	ksliding = ealphabeta_abs*ksliding_ref/E0+ksliding_ref/100.0;
		    else
		        	ksliding = ksliding_ref;


			ialpha_est = est_a*ialpha_lido+est_b*(valpha_ref-ealpha_est);
	        ibeta_est = est_a*ibeta_lido+est_b*(vbeta_ref-ebeta_est);


	        ealpha_slide = ksliding*sign(ialpha_est-ialpha_lido);
	        ebeta_slide = ksliding*sign(ibeta_est-ibeta_lido);

	        ealpha_filt = ealpha_filt+ts*wcsliding*(ealpha_slide-ealpha_filt);
	        ebeta_filt = ebeta_filt+ts*wcsliding*(ebeta_slide-ebeta_filt);

	        ealpha_est = lsliding*ealpha_slide+ealpha_filt;
	        ebeta_est = lsliding*ebeta_slide+ebeta_filt;

	        oe_est = atan2_approximation1(-ebeta_est,ealpha_est);
	        //oe_est = atan2_approximation1(-ealpha_est,ebeta_est);
	        if (oe_est > pi)
	            oe_est -= 2*pi;
	        ;
	        if (oe_est < -pi)
	            oe_est += 2*pi;

	        if ((absx(ealpha_est) < 3.5*E0) && (absx(ebeta_est) < 3.5*E0))
	            oe_est = 0;


	        //pll
	        erro_oe_est_pll = oe_est-oe_est_pll;
	        if (absx(erro_oe_est_pll) > 15*pi/180)
	            erro_oe_est_pll = 0;


	        erro_a_oe_est_pll = erro_oe_est_pll*ts+erro_a_oe_est_pll;
	        we_pll = kp_pll*erro_oe_est_pll+ki_pll*erro_a_oe_est_pll;

	        oe_est_pll = oe_est_pll+ts*we_pll;
	        if (oe_est_pll > pi)
	            oe_est_pll -= 2*pi;

	        if (oe_est_pll < -pi)
	            oe_est_pll += 2*pi;

	        //filtro de primeira ordem
	        //we_pll_filt = we_pll_filt+ts*wcpll*(we_pll-we_pll_filt);

	        //filtro de segunda ordem
	        we_pll_filt1 = we_pll_filt1+ts*we_pll_filt;
	        we_pll_filt = we_pll_filt+ts*(wcpll2*we_pll-wcpll2*we_pll_filt1-2*xipll*wcpll*we_pll_filt);
	        we_est = we_pll_filt1;

}
void controle_vetorial_sensorless(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_est/np;
	oe_lido = oe_est_pll;
	we_lido =  we_est;

//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;

    //theta_a = we_ref_rampa*ts+theta_a;
	theta_a = we_lido*ts+theta_a;
    if(theta_a > pi)
    	theta_a -= 2*pi;
    if(theta_a < -pi)
        theta_a += 2*pi;

	//asm_theta = oe_lido;
    asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	asm_theta = oe_lido;
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;

    erro_wm = wm_ref_rampa-wm_lido;
    zwe = zwe+erro_wm*ts;
    te_ref = (kp_wm*erro_wm+ki_wm*zwe);
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);
    if(iqs_ref > 0.8*Imax){
    	iqs_ref = 0.8*Imax;
    	zwe -= erro_wm*ts;
    }


    ids_ref = 0.0;

    erro_iqs = iqs_ref-iqs_lido;
    ziqs = ziqs+erro_iqs*ts;
    vqs_ref = kp_iqs*erro_iqs+ki_iqs*ziqs+we_lido*(fpm+lds*ids_lido)+v0;
    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        ziqs -= erro_iqs*ts;
        //zwe -= erro_wm*ts;
    }
    if (vqs_ref<=vmin){
        vqs_ref = vmin;
        ziqs -= erro_iqs*ts;
        //zwe -= erro_wm*ts;
    }

    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids*ts;
    vds_ref = kp_ids*erro_ids+ki_ids*zids-we_lido*(lqs*iqs_lido);
    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids*ts;
    }
    if (vds_ref<=-vds*0.5){
        vds_ref = -vds*0.5;
        zids -= erro_ids*ts;
    }



    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;
}
void controle_realimentacao_estados(){

//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_est/np;
//	oe_lido = oe_est_pll;
//	we_lido =  we_est;

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;



    theta_a = we_ref_rampa*ts+theta_a;
    if(theta_a > pi)
    	theta_a -= 2*pi;
    if(theta_a < -pi)
        theta_a += 2*pi;

	//asm_theta = oe_lido;
    asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;

    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;
    ids_ref = 0.0;
    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    vqs_ref = kr_we*we_ref_rampa+kx_we*we_lido+kx_iqs*iqs_lido+kz_we*zwe;
    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        zwe -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        zwe -= erro_we;
    }


    vds_ref =  kr_ids*ids_ref+kx_ids*ids_lido+kz_ids*zids;
    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;
}
void controle_alphabeta(){

//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_est/np;
//	oe_lido = oe_est_pll;
//	we_lido =  we_est;

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;
//


    erro_wm = wm_ref_rampa-wm_lido;
    zwe = zwe+erro_wm*ts;
    te_ref = (kp_wm*erro_wm+ki_wm*zwe);
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);
    if(iqs_ref > 0.8*Imax){
    	iqs_ref = 0.8*Imax;
    	zwe -= erro_wm*ts;
    }


    //theta_a = we_ref_rampa*ts+theta_a;
    //theta_a = we_lido*ts+theta_a;
    theta_a = theta_e;
    if(theta_a > pi)
    	theta_a -= 2*pi;
    if(theta_a < -pi)
        theta_a += 2*pi;

    asm_theta = theta_a;
    cos_oe_lido = asm_cos();
    sin_oe_lido = asm_sin();

	ialpha_ref = iqs_ref*cos_oe_lido+ids_ref*sin_oe_lido;
	ibeta_ref = -iqs_ref*sin_oe_lido+ids_ref*cos_oe_lido;

    erro_iqs = ialpha_ref-ialpha_lido;
    ziqs = ziqs+erro_iqs*ts;
    valpha_ref = kp_iqs*erro_iqs+ki_iqs*ziqs;
    if (valpha_ref>=vmax){
        valpha_ref = vmax;
        ziqs -= erro_iqs*ts;
        //zwe -= erro_wm*ts;
    }
    if (valpha_ref<=-vmax){
        valpha_ref = -vmax;
        ziqs -= erro_iqs*ts;
        //zwe -= erro_wm*ts;
    }

    erro_ids = ibeta_ref-ibeta_lido;
    zids = zids+erro_ids*ts;
    vbeta_ref = kp_ids*erro_ids+ki_ids*zids-we_lido*(lqs*iqs_lido);
    if (vbeta_ref>=vmax){
        vbeta_ref = vmax;
        zids -= erro_ids*ts;
    }
    if (vbeta_ref<= -vmax){
        vbeta_ref = -vmax;
        zids -= erro_ids*ts;
    }



    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;
}
void controle_preditivo_linear_malha_direta(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;

	asm_theta = oe_lido;
   // asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;

    erro_we = we_ref_rampa-we_lido;

    zwe = zwe+erro_we;
        if(zwe > 900000)
        	zwe = 900000;
        if(zwe < -900000)
            zwe = -900000;

    ids_ref = 0.0;
    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    vqs_ref = kr_we*we_ref_rampa-kx_we*we_lido-kx_iqs*iqs_lido+kz_we*zwe;
    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        zwe -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        zwe -= erro_we;
    }


    vds_ref =  kr_ids*ids_ref-kx_ids*ids_lido+kz_ids*zids;
    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void controle_preditivo_linear_malha_direta_com_estPBC(){

	wm_ref_rampa = we_ref_rampa/np;
		wm_lido = we_real/np;
		oe_lido = oe_real;
		we_lido =  we_real;


	//    theta_a = we_ref_rampa*ts+theta_a;
	//    if(theta_a > pi)
	//    	theta_a -= 2*pi;
	//    if(theta_a < -pi)
	//        theta_a += 2*pi;

		asm_theta = oe_lido;
	   // asm_theta = theta_a;
		cos_oe_lido = asm_cos();
		sin_oe_lido = asm_sin();


		iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
		ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;

	    erro_we = we_ref_rampa-we_lido;
	    zwe = zwe+erro_we;


	    ids_ref = 0.0;
	    erro_ids = ids_ref-ids_lido;
	    zids = zids+erro_ids;

	    vqs_ref = kr_we*we_ref_rampa-kx_we*we_lido-kx_iqs*iqs_lido+kz_we*zwe;
	    if (vqs_ref>=vmax){
	        vqs_ref = vmax;
	        zwe -= erro_we;
	    }
	    if (vqs_ref<=-vmax){
	        vqs_ref = -vmax;
	        zwe -= erro_we;
	    }


	    vds_ref =  kr_ids*ids_ref-kx_ids*ids_lido+kz_ids*zids;
	    if (vds_ref>=vmax){
	        vds_ref = vmax;
	        zids -= erro_ids;
	    }
	    if (vds_ref<=-vmax){
	        vds_ref = -vmax;
	        zids -= erro_ids;
	    }


	    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
	    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

	    asm_theta = oe_lido;
	    cos_oe = asm_cos();
	    vas = valpha_ref + fpm*cos_oe*we_lido-10*rs*ias_lido;

	    asm_theta = oe_lido -2.0*pi/3.0;
	    if (asm_theta < -pi)
	    	asm_theta +=2*pi;
	    cos_oe = asm_cos();
	    vbs = (-0.5*valpha_ref-0.866*vbeta_ref) + fpm*cos_oe*we_lido-10*rs*ibs_lido;

	    asm_theta = oe_lido +2.0*pi/3.0;
		if (asm_theta > pi)
			asm_theta -=2*pi;
		cos_oe = asm_cos();
	    vcs = (-0.5*valpha_ref+0.866*vbeta_ref) + fpm*cos_oe*we_lido-10*rs*ics_lido;

}
void controle_preditivo_linear_cascata(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;


    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;


    te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);


    //malha interna
    asm_theta = oe_lido;
   // asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;


    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    erro_iqs = iqs_ref-iqs_lido;
    ziqs = ziqs+erro_iqs;


    vqs_ref = kr_vqs_iqs*iqs_ref+kr_vqs_ids*ids_ref-kx_vqs_iqs*iqs_lido-kx_vqs_ids*ids_lido-kx_vqs_we*we_lido+kz_vqs_iqs*ziqs+kz_vqs_ids*zids;
    vds_ref = kr_vds_ids*ids_ref+kr_vds_iqs*iqs_ref-kx_vds_iqs*iqs_lido-kx_vds_ids*ids_lido-kx_vds_we*we_lido+kz_vds_iqs*ziqs+kz_vds_ids*zids;


    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        ziqs -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        ziqs -= erro_we;
    }



    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void controle_preditivo_linear_cascata_com_compensacao_total(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;


    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;


    te_ref = kr_we_comp*we_ref_rampa-kx_we_comp*we_lido+kz_we_comp*zwe;
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);


    //malha interna
    asm_theta = oe_lido;
   // asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;


    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    erro_iqs = iqs_ref-iqs_lido;
    ziqs = ziqs+erro_iqs;


    vqs_ref = kr_iqs_comp*iqs_ref-kx_iqs_comp*iqs_lido+kz_iqs_comp*ziqs+(fpm+lds*ids_lido)*we_lido;
    vds_ref = kr_ids_comp*ids_ref-kx_ids_comp*ids_lido+kz_ids_comp*zids-lqs*iqs_lido*we_lido;


    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        ziqs -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        ziqs -= erro_we;
    }



    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void controle_preditivo_linear_cascata_com_compensacao_parcial(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;


    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;


    te_ref = kr_we_comp*we_ref_rampa-kx_we_comp*we_lido+kz_we_comp*zwe;
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);


    //malha interna
    asm_theta = oe_lido;
   // asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;


    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    erro_iqs = iqs_ref-iqs_lido;
    ziqs = ziqs+erro_iqs;


    vqs_ref = kr_iqs_comp*iqs_ref-kx_iqs_comp*iqs_lido+kz_iqs_comp*ziqs+fpm*we_lido;
    vds_ref = kr_ids_comp*ids_ref-kx_ids_comp*ids_lido+kz_ids_comp*zids;


    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        ziqs -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        ziqs -= erro_we;
    }



    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void controle_preditivo_linear_cascata_malha_internaPBC(){

	float cos_oe_a, cos_oe_b,cos_oe_c;
	float sin_oe_a, sin_oe_b,sin_oe_c;
	float ias_ref,ibs_ref,ics_ref;
	float ias_ref_p,ibs_ref_p,ics_ref_p;
	float uas_ref,ubs_ref,ucs_ref;
	float k1 = 8*rs;

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


	asm_theta = oe_lido;
	cos_oe_a = asm_cos();
	sin_oe_a = asm_sin();


	asm_theta = oe_lido -2.0*pi/3.0;
	if (asm_theta < -pi)
		asm_theta +=2*pi;
	cos_oe_b = asm_cos();
	sin_oe_b = asm_sin();

	asm_theta = oe_lido +2.0*pi/3.0;
	if (asm_theta > pi)
		asm_theta -=2*pi;
	cos_oe_c = asm_cos();
	sin_oe_c = asm_sin();

//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;


    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;


    te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);


    //malha interna
    ias_ref = iqs_ref*cos_oe_a;
    ibs_ref = iqs_ref*cos_oe_b;
    ics_ref = iqs_ref*cos_oe_c;

    ias_ref_p = -iqs_ref*sin_oe_a*we_lido;
    ibs_ref_p = -iqs_ref*sin_oe_b*we_lido;
    ics_ref_p = -iqs_ref*sin_oe_c*we_lido;

    uas_ref = 0.5*las*ias_ref_p+(k1+rs)*ias_ref;
    ubs_ref = 0.5*las*ibs_ref_p+(k1+rs)*ibs_ref;
    ucs_ref = 0.5*las*ics_ref_p+(k1+rs)*ics_ref;

    vas = uas_ref+fpm*cos_oe_a*we_lido-k1*ias_lido;
    vbs = ubs_ref+fpm*cos_oe_b*we_lido-k1*ibs_lido;
    vcs = ucs_ref+fpm*cos_oe_c*we_lido-k1*ics_lido;

}
void controle_preditivo_nao_linear_cascata_euler_1aordN1(){

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


//    theta_a = we_ref_rampa*ts+theta_a;
//    if(theta_a > pi)
//    	theta_a -= 2*pi;
//    if(theta_a < -pi)
//        theta_a += 2*pi;


    erro_we = we_ref_rampa-we_lido;
    zwe = zwe+erro_we;
    if(zwe > 900000)
    	zwe = 900000;
    if(zwe < -900000)
        zwe = -900000;

    ids_ref = 0.0;

    te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
    iqs_ref = 2.0/3.0*te_ref/(np*fpm);


    //malha interna
    asm_theta = oe_lido;
   // asm_theta = theta_a;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();


	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;


    erro_ids = ids_ref-ids_lido;
    zids = zids+erro_ids;

    erro_iqs = iqs_ref-iqs_lido;
    ziqs = ziqs+erro_iqs;


    vds_ref = (bm*iqs_lido*lds*rw2*ts*ts*we_lido + bm*iqs_lido*lds*ry2*ts*ts*we_lido - 1.5*fpm*iqs_lido*iqs_lido*lds*np*np*rw2*ts*ts - 1.5*fpm*iqs_lido*iqs_lido*lds*np*np*ry2*ts*ts - 1.5*ids_lido*iqs_lido*iqs_lido*lds*lds*np*np*rw2*ts*ts - 1.5*ids_lido*iqs_lido*iqs_lido*lds*lds*np*np*ry2*ts*ts + 1.5*ids_lido*iqs_lido*iqs_lido*lds*lqs*np*np*rw2*ts*ts + 1.5*ids_lido*iqs_lido*iqs_lido*lds*lqs*np*np*ry2*ts*ts - ids_lido*jm*lds*rw2 - ids_lido*jm*lds*ry2 + ids_lido*jm*rs*rw2*ts + ids_lido*jm*rs*ry2*ts + ids_ref*jm*lds*rw2 + ids_ref*jm*lds*ry2 - iqs_lido*jm*lds*rw2*ts*we_lido - iqs_lido*jm*lds*ry2*ts*we_lido + jm*lds*rw2*zids)/(jm*ts*(ru2 + rw2 + ry2));
    vqs_ref = (-bm*ids_lido*lqs*rw1*ts*ts*we_lido - bm*ids_lido*lqs*ry1*ts*ts*we_lido + 1.5*fpm*ids_lido*iqs_lido*lqs*np*np*rw1*ts*ts + 1.5*fpm*ids_lido*iqs_lido*lqs*np*np*ry1*ts*ts + fpm*jm*rw1*ts*we_lido + fpm*jm*ry1*ts*we_lido + 1.5*ids_lido*ids_lido*iqs_lido*lds*lqs*np*np*rw1*ts*ts + 1.5*ids_lido*ids_lido*iqs_lido*lds*lqs*np*np*ry1*ts*ts - 1.5*ids_lido*ids_lido*iqs_lido*lqs*lqs*np*np*rw1*ts*ts - 1.5*ids_lido*ids_lido*iqs_lido*lqs*lqs*np*np*ry1*ts*ts + ids_lido*jm*lqs*rw1*ts*we_lido + ids_lido*jm*lqs*ry1*ts*we_lido - iqs_lido*jm*lqs*rw1 - iqs_lido*jm*lqs*ry1 + iqs_lido*jm*rs*rw1*ts + iqs_lido*jm*rs*ry1*ts + iqs_ref*jm*lqs*rw1 + iqs_ref*jm*lqs*ry1 + jm*lqs*rw1*ziqs)/(jm*ts*(ru1 + rw1 + ry1));


    if (vqs_ref>=vmax){
        vqs_ref = vmax;
        ziqs -= erro_we;
    }
    if (vqs_ref<=-vmax){
        vqs_ref = -vmax;
        ziqs -= erro_we;
    }



    if (vds_ref>=vmax){
        vds_ref = vmax;
        zids -= erro_ids;
    }
    if (vds_ref<=-vmax){
        vds_ref = -vmax;
        zids -= erro_ids;
    }


    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

    vas = valpha_ref;
    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
//#CÁLCULO DAS TENSÕES DO INVERSOR (IGUAL AO CÓDIGO DA SABRINA)
//float  inversor_ideal(){
//	int s1=sw1, s2=sw2, s3=sw3;
//    asm_a = (2.0/3.0*s1-1.0/3.0*s2-1.0/3.0*s3)*vdc;
//    asm_b = (2.0/3.0*s2-1.0/3.0*s1-1.0/3.0*s3)*vdc;
//    asm_c = (2.0/3.0*s3-1.0/3.0*s2-1.0/3.0*s1)*vdc;
//    return 0;
//	}
//#ROTINA PARA O CHAVEAMENTO (IGUAL AO CÓDIGO DA SABRINA)
//int  chaveamento(int indice_min){
//    switch(indice_min){
//		case 1:
//			sw1=1;
//			sw2=0;
//			sw3=0;
//			return 0;
//		case 2:
//			sw1=0;
//			sw2=1;
//			sw3=0;
//			return 0;
//		case 3:
//			sw1=1;
//			sw2=1;
//			sw3=0;
//			return 0;
//		case 4:
//			sw1=0;
//			sw2=0;
//			sw3=1;
//			return 0;
//		case 5:
//			sw1=1;
//			sw2=0;
//			sw3=1;
//			return 0;
//		case 6:
//			sw1=0;
//			sw2=1;
//			sw3=1;
//			return 0;
//		default:
//			sw1=0;
//			sw2=0;
//			sw3=0;
//			return 0;
//					}
//	}
//
//void  inversor_ideal(){
//	asm_temp=sw1;
//	s1=asm_temp;
//	asm_temp=sw2;
//	s2=asm_temp;
//	asm_temp=sw3;
//	s3=asm_temp;
//	asm_temp=vdc;
//	s4=asm_temp;
//	s5=3.0;
//	asm(" VADD.F32 s0,s1,s1");
//	asm(" VSUB.F32 s0,s0,s2");
//	asm(" VSUB.F32 s0,s0,s3");
//	asm(" VDIV.F32 s0,s0,s5");
//	asm(" VMUL.F32 s0,s0,s4");
//	asm_a=s0;
//
//	asm(" VADD.F32 s0,s2,s2");
//	asm(" VSUB.F32 s0,s0,s1");
//	asm(" VSUB.F32 s0,s0,s3");
//	asm(" VDIV.F32 s0,s0,s5");
//	asm(" VMUL.F32 s0,s0,s4");
//	asm_b=s0;
//
//	asm(" VADD.F32 s0,s3,s3");
//	asm(" VSUB.F32 s0,s0,s2");
//	asm(" VSUB.F32 s0,s0,s1");
//	asm(" VDIV.F32 s0,s0,s5");
//	asm(" VMUL.F32 s0,s0,s4");
//	asm_c=s0;
//	// va = (2.0/3.0*s1-1.0/3.0*s2-1.0/3.0*s3)*vdc;
//    // vb = (2.0/3.0*s2-1.0/3.0*s1-1.0/3.0*s3)*vdc;
//    // vc = (2.0/3.0*s3-1.0/3.0*s2-1.0/3.0*s1)*vdc;
//	return;
//	}

//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int s1_min,s2_min,s3_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c2,wm_c2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl=tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//    for(int s1c = 0; s1c <= 1; ++s1c){
//        for(int s2c = 0; s2c <= 1; ++s2c){
//            for(int s3c = 0; s3c <= 1; ++s3c){
//                asm_temp = s1c;
//                s1=asm_temp;
//                asm_temp = s2c;
//				s2=asm_temp;
//				asm_temp = s3c;
//				s3=asm_temp;
//				asm_temp=vdc;
//				s4=asm_temp;
//				s5=3.0;
//				asm(" VADD.F32 s0,s1,s1");
//				asm(" VSUB.F32 s0,s0,s2");
//				asm(" VSUB.F32 s0,s0,s3");
//				asm(" VDIV.F32 s0,s0,s5");
//				asm(" VMUL.F32 s0,s0,s4");
//                asm_a = s0;
//
//                asm(" VADD.F32 s0,s2,s2");
//				asm(" VSUB.F32 s0,s0,s1");
//				asm(" VSUB.F32 s0,s0,s3");
//				asm(" VDIV.F32 s0,s0,s5");
//				asm(" VMUL.F32 s0,s0,s4");
//                asm_b = s0;
//
//                asm(" VADD.F32 s0,s3,s3");
//				asm(" VSUB.F32 s0,s0,s1");
//				asm(" VSUB.F32 s0,s0,s2");
//				asm(" VDIV.F32 s0,s0,s5");
//				asm(" VMUL.F32 s0,s0,s4");
//				asm_c = s0;
//
//
//
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//
////                asm_temp = rs;
////                s1 = asm_temp;
////                asm_temp = ts;
////                s2 = asm_temp;
////                asm_temp = lqs;
////                s3 = asm_temp;
////                asm_temp = lds;
////                s4 = asm_temp;
////                asm_temp = fpm;
////                s5 = asm_temp;
////                asm_temp = bm;
////                s6 = asm_temp;
////                asm_temp = jm;
////                s7 = asm_temp;
////
////                //np
////                asm_temp = np;
////                s20 = asm_temp;
////                //wm_ref_rampa
////                asm_temp = wm_ref_rampa;
////            	s8 = asm_temp;
////            	//wm_lido
////                asm_temp = wm_lido;
////                s9 = asm_temp;
////                //tl
////                asm_temp = tl;
////            	s10 = asm_temp;
////            	//iqs_lido
////                asm_temp = iqs_lido;
////                s11 = asm_temp;
////            	//ids_lido
////                asm_temp = ids_lido;
////                s12 = asm_temp;
////
////                // Iq(k+1):
////                //rs*ts
////                asm(" VMUL.F32 s0,s1,s2");
////                //rs*ts/lqs
////                asm(" VDIV.F32 s0,s0,s3");
////                //(1-rs*ts/lqs)
////                s13 = 1.0;
////                asm(" VSUB.F32 s0,s13,s0");
////                //iq_c1 = (1-rs*ts/lqs)*iqs_lido
////                asm(" VMUL.F32 s13,s0,s11");
////                //lds*ts
////                asm(" VMUL.F32 s0,s2,s4");
////                //lds/lqs*ts
////                asm(" VDIV.F32 s0,s0,s3");
////                //lds/lqs*ts*wm_lido
////                asm(" VMUL.F32 s0,s0,s9");
////                //lds/lqs*ts*wm_lido*ids_lido
////                asm(" VMUL.F32 s0,s0,s12");
////                //iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido
////                asm(" VSUB.F32 s13,s13,s0");
////                //ts/lqs
////                asm(" VDIV.F32 s0,s2,s3");
////                //ts/lqs*fpm
////                asm(" VMUL.F32 s0,s0,s5");
////                //ts/lqs*wm_lido*fpm;
////                asm(" VMUL.F32 s0,s0,s9");
////                //iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido - ts/lqs*wm_lido*fpm;
////                asm(" VSUB.F32 s13,s13,s0");
////                //ts/lqs
////                asm(" VDIV.F32 s0,s2,s3");
////                s19=asm_q;
////                //ts/lqs*asm_q
////                asm(" VMUL.F32 s0,s0,s19");
////                //iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido - ts/lqs*wm_lido*fpm + ts/lqs*asm_q;
////                asm(" VADD.F32 s13,s13,s0");
////                asm_temp = s13;
////                iq_c1 = asm_temp;
//                iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido + ts/lqs*asm_q - ts/lqs*wm_lido*fpm;
//                id_c1 = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido + ts/lds*asm_d;
//                te_c1 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//                wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c1-tl);
//
//                te_c2 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//                wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c2-tl);
//
//                Jc=(wm_ref_rampa-wm_c2)*(wm_ref_rampa-wm_c2)+0.00001*id_c1*id_c1+0.00001*iq_c1*iq_c1;
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//            }
//        }
//    }
//
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}

//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int s1_min,s2_min,s3_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl= tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido;
//	te_c0 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//	wm_c0 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
//	tl2= tl+0.2*ts*(wm_ref_rampa-wm_c0);
//    for(int s1c = 0; s1c <= 1; ++s1c){
//        for(int s2c = 0; s2c <= 1; ++s2c){
//            for(int s3c = 0; s3c <= 1; ++s3c){
//
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//                te_c1 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//                wm_c1 = (1-ts*bm/jm)*wm_c0 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl2);
//                tl2=0;
////                id_c1=id_c1*id_c1;
////                iq_c1=iq_c1*iq_c1;
////                if(id_c1>4||iq_c1>4)
////                	tl2=1;
//                Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(tl2+0.000001)*(id_c1+iq_c1);
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//            }
//        }
//    }
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}

//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int s1_min,s2_min,s3_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c2,wm_c2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl=tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//    for(int s1c = 0; s1c <= 1; ++s1c){
//        for(int s2c = 0; s2c <= 1; ++s2c){
//            for(int s3c = 0; s3c <= 1; ++s3c){
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm+ ts/lqs*asm_q;
//                id_c1 = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido + ts/lds*asm_d;
//                te_c1 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//                wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c1-tl);
//
//                te_c2 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//                wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c2-tl);
//
//                Jc=(wm_ref_rampa-wm_c2)*(wm_ref_rampa-wm_c2)+0.00001*(id_c1*id_c1+iq_c1*iq_c1);
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//            }
//        }
//    }
//
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//
//
//
//
//
//
//
//}

//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int indice_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c2,wm_c2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl=tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//    for(int i = 0; i < 7; ++i){
//        chaveamento(i);
//        inversor_ideal(vdc);
//        asm_a=va;
//        asm_b=vb;
//        asm_c=vc;
//        asm_theta = oe_lido;
//        asm_transf_dq();
////vd_c1,vq_c1 = dq_transf(va,  vb,  vc,oe);
//
//
//        //fpm = 0.19, ParPolos = 21, np = 21, lqs = 0.05, lds = 0.05, jm = 0.00115, bm = 0.0575, rs = 4.5, las = 0.0333;
//
//        iq_c1 = (1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido + ts/lqs*asm_q - ts/lqs*wm_lido*fpm;
//        id_c1 = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido + ts/lds*asm_d;
//        te_c1 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//        wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c1-tl);
//
//        te_c2 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//        wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c2-tl);
//
//        Jc=(wm_ref_rampa-wm_c2)*(wm_ref_rampa-wm_c2)+0.00001*id_c1*id_c1+0.00001*iq_c1*iq_c1;
//        if(Jc<=Jmin){
//            Jmin=Jc;
//            indice_min=i;
//        }
//    }
//
//    chaveamento(indice_min);
//    inversor_ideal(vdc);
//    vas = va;
//    vbs = vb;
//    vcs = vc;
//}

//void controle_preditivo_fcs(){
//	Jmin=100000000000000;
////	int s1_min,s2_min,s3_min;
////	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//	kr_vqs_iqs = 9.762826334321593;
//	kr_vqs_ids = 0;
//	kx_vqs_we = 0.005470710968279944;
//	kx_vqs_iqs = 9.582699900789686;
//	kx_vds_ids = 9.651672596130698;
//	kz_vqs_iqs = 0.014664029434968907;
//	kz_vds_ids = 0.014683085639884614;
//	kz_we_we = 0.00018446038333317394;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//	//DO CODIGO DO ARTHUR
//	erro_we = we_ref_rampa-we_lido;
//	zwe = zwe+erro_we;
////	te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
////	iqs_ref = 2.0/3.0*te_ref/(np*fpm);
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//	tl= tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	te_c0 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//	wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
////	tl2= tl+0.2*ts*(wm_ref_rampa-wm_c1);
//	erro_we = we_ref_rampa-np*wm_c1;
//	zwetemp = zwe+erro_we;
//	te_ref = kr_we_we*we_ref_rampa-kx_we_we*wm_c1+kz_we_we*zwetemp;
//	iqs_ref = 2.0/3.0*te_ref/(np*fpm);
//
//	erro_ids = ids_ref-ids_lido;
//	zids = zids+erro_ids;
//	erro_iqs = iqs_ref-iqs_lido;
//	ziqs = ziqs+erro_iqs;
//	//////
//
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido;
//	s1_ant = s1_min;
//	s2_ant = s2_min;
//	s3_ant = s3_min;
//		for(int k = 0; k <= 2; ++k){
//	        switch(k){
//	            case 0:
//	                s1c=abs(s1_ant-1);
//	                s2c=s2_ant;
//	                s3c=s3_ant;
//	            break;
//	            case 1:
//	                s1c=s1_ant;
//	                s2c=abs(s2_ant-1);
//	                s3c=s3_ant;
//	            break;
//	            case 2:
//	                s1c=s1_ant;
//	                s2c=s2_ant;
//	                s3c=abs(s3_ant-1);
//	            break;
//	            default:
//	                s1c=s1_ant;
//	                s2c=s2_ant;
//	                s3c=s3_ant;
//	            break;
//	        }
//
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//				te_c1 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//				wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl);
////                kr_vqs_iqs*iqs_ref+kr_vqs_ids*ids_ref-kx_vqs_iqs*iqs_lido-kx_vqs_ids*ids_lido-kx_vqs_we*we_lido+kz_vqs_iqs*ziqs+kz_vqs_ids*zids;
//
//
////				te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
//                Jc=kr_vqs_iqs*abs(iqs_ref-iq_c1);//+kx_vds_ids*abs(id_c1);//+kx_vqs_we*abs(wm_c2);//*(wm_ref_rampa-wm_c2) ;//+kz_vds_ids*abs((ids_ref-ids_lido)+zids)+kz_vqs_iqs*abs((iqs_ref-iqs_lido)+ziqs);
////				Jc=kr_vqs_iqs*abs(iqs_ref)-kx_vqs_iqs*abs(iq_c1)-kx_vds_ids*abs(id_c1)+kx_vqs_we*abs(wm_c2)+kz_vds_ids*abs((ids_ref-id_c1)+zids)+kz_vqs_iqs*abs((iqs_ref-iq_c1)+ziqs);
////				Jc=kr_vqs_iqs*abs(iqs_ref-iq_c1)+kx_vds_ids*abs(id_c1)+kx_vqs_we*np*abs(wm_ref_rampa-wm_c2)+kz_vds_ids*abs((ids_ref-id_c1)+zids)+kz_vqs_iqs*abs((iqs_ref-iq_c1)+ziqs);
////				Jc=kx_vqs_we*abs(we_ref_rampa-np*wm_c2);//+kz_we_we*abs(zwetemp+we_ref_rampa-np*wm_c2);
//				//				Jc=kr_vqs_iqs*(iqs_ref)-kx_vqs_iqs*(iq_c1)-kx_vds_ids*(id_c1)+kx_vqs_we*(wm_c2)+kz_vds_ids*((ids_ref-id_c1)+zids)+kz_vqs_iqs*((iqs_ref-iq_c1)+ziqs);
////				Jc=abs(Jc);
//				//                vqs_ref = +kz_vqs_iqs*ziqs+kz_vqs_ids*zids;
////                kr_vqs_iqs*iqs_ref+kr_vqs_ids*ids_ref-kx_vqs_iqs*iqs_lido-kx_vqs_ids*ids_lido-kx_vqs_we*we_lido
//                //                    vds_ref = kr_vds_ids*ids_ref+kr_vds_iqs*iqs_ref-kx_vds_iqs*iqs_lido-kx_vds_ids*ids_lido-kx_vds_we*we_lido+kz_vds_iqs*ziqs+kz_vds_ids*zids;
//
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//
//
//    }
//    vbs = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vas = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}

//void controle_preditivo_fcs(){
//	Jmin=100000000000000;
////	int s1_min,s2_min,s3_min;
////	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//    asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido;
//	tl= tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	te_c0 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//	wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
////	tl2= tl+0.2*ts*(wm_ref_rampa-wm_c0);
//
//	s1_ant = s1_min;
//	s2_ant = s2_min;
//	s3_ant = s3_min;
//		for(int k = 0; k <= 2; ++k){
//	        switch(k){
//	            case 0:
//	                s1c=abs(s1_ant-1);
//	                s2c=s2_ant;
//	                s3c=s3_ant;
//	            break;
//	            case 1:
//	                s1c=s1_ant;
//	                s2c=abs(s2_ant-1);
//	                s3c=s3_ant;
//	            break;
//	            case 2:
//	                s1c=s1_ant;
//	                s2c=s2_ant;
//	                s3c=abs(s3_ant-1);
//	            break;
//	            default:
//	                s1c=s1_ant;
//	                s2c=s2_ant;
//	                s3c=s3_ant;
//	            break;
//	        }
//
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//                te_c1 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//                wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl);
//
//                const_iq2 =(1-rs*ts/lqs)*iq_c1 - lds/lqs*ts*wm_c1*id_c1  - ts/lqs*wm_c1*fpm;
//            	const_id2 = (1-rs*ts/lds)*id_c1 + lqs/lds*ts*wm_c1*iq_c1;
//        		for(int l = 0; l <= 2; ++l){
//        	        switch(l){
//        	            case 0:
//        	                s1c2=abs(s1c-1);
//        	                s2c2=s2c;
//        	                s3c2=s3c;
//        	            break;
//        	            case 1:
//        	                s1c2=s1c;
//        	                s2c2=abs(s2c-1);
//        	                s3c2=s3c;
//        	            break;
//        	            case 2:
//        	                s1c2=s1c;
//        	                s2c2=s2c;
//        	                s3c2=abs(s3c-1);
//        	            break;
//        	            default:
//        	                s1c2=s1c;
//        	                s2c2=s2c;
//        	                s3c2=s3c;
//        	            break;
//        	        }
//                    asm_a = (2*s1c2-s2c2-s3c2)*vdc/3.0;
//                    asm_b = (2*s2c2-s1c2-s3c2)*vdc/3.0;
//                    asm_c = (2*s3c2-s2c2-s1c2)*vdc/3.0;
//
//                    asm_theta = oe_lido;
//                    asm_transf_dq();
//                    iq_c2 = const_iq2 + ts/lqs*asm_q;
//                    id_c2 = const_id2 + ts/lds*asm_d;
//                    te_c2 = 3/2*((lds-lqs)*iq_c2*id_c2+fpm*iq_c2);
//                    wm_c3 = (1-ts*bm/jm)*wm_c2 + polos_div_2*polos_div_2*ts/jm*(te_c2-tl);
//
//                tl2=0;
//                id_c1=id_c1*id_c1;
//                iq_c1=iq_c1*iq_c1;
//                id_c2=id_c2*id_c2;
//                iq_c2=iq_c2*iq_c2;
//                if(id_c1>4||iq_c1>4||id_c2>4||iq_c2>4)
//                	tl2=1;
//                Jc=(wm_ref_rampa-wm_c3)*(wm_ref_rampa-wm_c3)+(wm_ref_rampa-wm_c2)*(wm_ref_rampa-wm_c2)+(0.00001+tl2)*(id_c1+iq_c1+id_c2+iq_c2);
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//        		}
//
//    }
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}

//void controle_preditivo_fcs(){
//	Jmin=100000000000000;
////	int s1_min,s2_min,s3_min;
////	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//	kr_vqs_iqs = 9.762826334321593;
//	kr_vqs_ids = 0;
//	kx_vqs_we = 0.005470710968279944;
//	kx_vqs_iqs = 9.582699900789686;
//	kx_vds_ids = 9.651672596130698;
//	kz_vqs_iqs = 0.014664029434968907;
//	kz_vds_ids = 0.014683085639884614;
//	kz_we_we = 0.00018446038333317394;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//	//DO CODIGO DO ARTHUR
//	erro_we = we_ref_rampa-we_lido;
//	zwe = zwe+erro_we;
////	te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
////	iqs_ref = 2.0/3.0*te_ref/(np*fpm);
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//	tl= tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	te_c0 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//	wm_c1 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
////	tl2= tl+0.2*ts*(wm_ref_rampa-wm_c1);
//	erro_we = we_ref_rampa-np*wm_c1;
//	zwetemp = zwe+erro_we;
//	te_ref = kr_we_we*we_ref_rampa-kx_we_we*wm_c1+kz_we_we*zwetemp;
//	iqs_ref = 2.0/3.0*te_ref/(np*fpm);
//
//	erro_ids = ids_ref-ids_lido;
//	zids = zids+erro_ids;
//	erro_iqs = iqs_ref-iqs_lido;
//	ziqs = ziqs+erro_iqs;
//	//////
//
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido;
//	s1_ant = s1_min;
//	s2_ant = s2_min;
//	s3_ant = s3_min;
//	    for(int s1c = 0; s1c <= 1; ++s1c){
//	        for(int s2c = 0; s2c <= 1; ++s2c){
//	            for(int s3c = 0; s3c <= 1; ++s3c){
//
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//				te_c1 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//				wm_c2 = (1-ts*bm/jm)*wm_c1 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl);
////                kr_vqs_iqs*iqs_ref+kr_vqs_ids*ids_ref-kx_vqs_iqs*iqs_lido-kx_vqs_ids*ids_lido-kx_vqs_we*we_lido+kz_vqs_iqs*ziqs+kz_vqs_ids*zids;
//
//
////				te_ref = kr_we_we*we_ref_rampa-kx_we_we*we_lido+kz_we_we*zwe;
////                Jc=kr_vqs_iqs*abs(iqs_ref)-kx_vqs_iqs*abs(iq_c1)-kx_vds_ids*abs(id_c1)+kx_vqs_we*abs(wm_c2);//*(wm_ref_rampa-wm_c2) ;//+kz_vds_ids*abs((ids_ref-ids_lido)+zids)+kz_vqs_iqs*abs((iqs_ref-iqs_lido)+ziqs);
//				Jc=kr_vqs_iqs*abs(iqs_ref)-kx_vqs_iqs*abs(iq_c1)-kx_vds_ids*abs(id_c1)+kx_vqs_we*abs(wm_c2)+kz_vds_ids*abs((ids_ref-id_c1)+zids)+kz_vqs_iqs*abs((iqs_ref-iq_c1)+ziqs);
////				Jc=kr_vqs_iqs*abs(iqs_ref-iq_c1)+kx_vds_ids*abs(id_c1)+kx_vqs_we*np*abs(wm_ref_rampa-wm_c2)+kz_vds_ids*abs((ids_ref-id_c1)+zids)+kz_vqs_iqs*abs((iqs_ref-iq_c1)+ziqs);
////				Jc=kx_vqs_we*abs(we_ref_rampa-np*wm_c2);//+kz_we_we*abs(zwetemp+we_ref_rampa-np*wm_c2);
//				//				Jc=kr_vqs_iqs*(iqs_ref)-kx_vqs_iqs*(iq_c1)-kx_vds_ids*(id_c1)+kx_vqs_we*(wm_c2)+kz_vds_ids*((ids_ref-id_c1)+zids)+kz_vqs_iqs*((iqs_ref-iq_c1)+ziqs);
////				Jc=abs(Jc);
//				//                vqs_ref = +kz_vqs_iqs*ziqs+kz_vqs_ids*zids;
////                kr_vqs_iqs*iqs_ref+kr_vqs_ids*ids_ref-kx_vqs_iqs*iqs_lido-kx_vqs_ids*ids_lido-kx_vqs_we*we_lido
//                //                    vds_ref = kr_vds_ids*ids_ref+kr_vds_iqs*iqs_ref-kx_vds_iqs*iqs_lido-kx_vds_ids*ids_lido-kx_vds_we*we_lido+kz_vds_iqs*ziqs+kz_vds_ids*zids;
//
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//	            }
//	        }
//    }
//    vbs = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vas = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}

//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int s1_min,s2_min,s3_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl= tl+0.2*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - lds/lqs*ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + lqs/lds*ts*wm_lido*iqs_lido;
//	te_c0 = 3/2*((-lqs+lds)*iqs_lido*ids_lido+fpm*iqs_lido);
//	wm_c0 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
//	tl2= tl+0.2*ts*(wm_ref_rampa-wm_c0);
//	s1_ant = s1_min;
//	s2_ant = s2_min;
//	s3_ant = s3_min;
//		for(int k = 0; k <= 3; ++k){
//			switch(k){
//				case 0:
//					s1c=abs(s1_ant-1);
//					s2c=s2_ant;
//					s3c=s3_ant;
//				break;
//				case 1:
//					s1c=s1_ant;
//					s2c=abs(s2_ant-1);
//					s3c=s3_ant;
//				break;
//				case 2:
//					s1c=s1_ant;
//					s2c=s2_ant;
//					s3c=abs(s3_ant-1);
//				break;
////				case 3:
////					s1c=s1_ant;
////					s2c=abs(s2_ant-1);
////					s3c=abs(s3_ant-1);
////				break;
//				default:
//					s1c=s1_ant;
//					s2c=s2_ant;
//					s3c=s3_ant;
//				break;
//			}
//
//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//
//                asm_theta = oe_lido;
//                asm_transf_dq();
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//                te_c1 = 3/2*((lds-lqs)*iq_c1*id_c1+fpm*iq_c1);
//                wm_c1 = (1-ts*bm/jm)*wm_c0 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl2);
////                pho=0;
////                id_c1=id_c1*id_c1;
////                iq_c1=iq_c1*iq_c1;
////                if(id_c1>16||iq_c1>16)
////                	pho=1;
//                //0.00000001
//                //0.0000000001
////                Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.00000001)*abs(iq_c1);
//				Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.0000001)*abs(iq_c1);
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//    }
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}


void controle_preditivo_fcs(){
	//deu boa
	float Jmin=100000000000000;
	int s1_min,s2_min,s3_min;
	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;

	wm_ref_rampa = we_ref_rampa/np;
	wm_lido = we_real/np;
	oe_lido = oe_real;
	we_lido =  we_real;


	asm_theta = oe_lido;
	cos_oe_lido = asm_cos();
	sin_oe_lido = asm_sin();

	tl= tl+0.25*ts*(wm_ref_rampa-wm_lido);
	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;

	const_iq =(1-rs*ts/lqs)*iqs_lido - ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
	const_id = (1-rs*ts/lds)*ids_lido + ts*wm_lido*iqs_lido;
	te_c0 = 1.5*fpm*iqs_lido;
	wm_c0 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
	tl2= tl+0.25*ts*(wm_ref_rampa-wm_c0);
	s1_ant = s1_min;
	s2_ant = s2_min;
	s3_ant = s3_min;
		for(int k = 0; k <= 3; ++k){
			switch(k){
				case 0:
					s1c=abs(s1_ant-1);
					s2c=s2_ant;
					s3c=s3_ant;
				break;
				case 1:
					s1c=s1_ant;
					s2c=abs(s2_ant-1);
					s3c=s3_ant;
				break;
				case 2:
					s1c=s1_ant;
					s2c=s2_ant;
					s3c=abs(s3_ant-1);
				break;
				default:
					s1c=s1_ant;
					s2c=s2_ant;
					s3c=s3_ant;
				break;
			}

                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;

                asm_theta = oe_lido;
                asm_transf_dq();
                iq_c1 = const_iq + ts/lqs*asm_q;
                id_c1 = const_id + ts/lds*asm_d;
                te_c1 = 1.5*fpm*iq_c1;
                wm_c1 = (1-ts*bm/jm)*wm_c0 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl2);
//                pho=0;
//                id_c1=id_c1*id_c1;
//                iq_c1=iq_c1*iq_c1;
//                if(id_c1>16||iq_c1>16)
//                	pho=1;
                //0.00000001
                //0.0000000001
                Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.00000001)*abs(iq_c1);
//				Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.0000001)*abs(iq_c1);
//				Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.000000001)*abs(id_c1)+(0.0000001)*abs(iq_c1);
                if(Jc<=Jmin){
                    Jmin=Jc;
                    s1_min=s1c;
                    s2_min=s2c;
                    s3_min=s3c;
                }
    }
    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
}


//void controle_preditivo_fcs(){
//	float Jmin=100000000000000;
//	int s1_min,s2_min,s3_min;
//	float Jc,iq_c1,id_c1,te_c1,wm_c1,te_c0,wm_c0,tl2;
//
//	wm_ref_rampa = we_ref_rampa/np;
//	wm_lido = we_real/np;
//	oe_lido = oe_real;
//	we_lido =  we_real;
//
//
//	asm_theta = oe_lido;
//	cos_oe_lido = asm_cos();
//	sin_oe_lido = asm_sin();
//
//	tl= tl+0.25*ts*(wm_ref_rampa-wm_lido);
//	iqs_lido = ialpha_lido*cos_oe_lido-ibeta_lido*sin_oe_lido;
//	ids_lido = ialpha_lido*sin_oe_lido+ibeta_lido*cos_oe_lido;
//
//	const_iq =(1-rs*ts/lqs)*iqs_lido - ts*wm_lido*ids_lido  - ts/lqs*wm_lido*fpm;
//	const_id = (1-rs*ts/lds)*ids_lido + ts*wm_lido*iqs_lido;
//	te_c0 = 1.5*fpm*iqs_lido;
//	wm_c0 = (1-ts*bm/jm)*wm_lido+polos_div_2*polos_div_2* ts/jm*(te_c0-tl);
//	tl2= tl+0.25*ts*(wm_ref_rampa-wm_c0);
//	s1_ant = s1_min;
//	s2_ant = s2_min;
//	s3_ant = s3_min;
//		for(int k = 0; k <= 3; ++k){
//			switch(k){
//				case 0:
//					s1c=abs(s1_ant-1);
//					s2c=s2_ant;
//					s3c=s3_ant;
//				break;
//				case 1:
//					s1c=s1_ant;
//					s2c=abs(s2_ant-1);
//					s3c=s3_ant;
//				break;
//				case 2:
//					s1c=s1_ant;
//					s2c=s2_ant;
//					s3c=abs(s3_ant-1);
//				break;
//				default:
//					s1c=s1_ant;
//					s2c=s2_ant;
//					s3c=s3_ant;
//				break;
//			}
//				asm_temp = vdc;
//                asm_a = s1c;
//                asm_b = s2c;
//                asm_c = s3c;
//
//                s0=asm_temp;
//            	s10 = asm_a;
//            	s11 = asm_b;
//            	s12 = asm_c;
//
//            	s20 = 2.0;
//            	s19 = 3.0;
//            	//                asm_a = (2*s1c-s2c-s3c)*vdc/3.0;
//            	//                asm_b = (2*s2c-s1c-s3c)*vdc/3.0;
//            	//                asm_c = (2*s3c-s2c-s1c)*vdc/3.0;
//            	asm(" VADD.F32 s1,s10,s10");
//            	asm(" VSUB.F32 s1,s1,s11");
//            	asm(" VSUB.F32 s1,s1,s12");
//            	asm(" VMUL.F32 s1,s1,s0");
//            	asm(" VDIV.F32 s1,s1,s19");
//
//            	asm(" VADD.F32 s2,s11,s11");
//				asm(" VSUB.F32 s2,s2,s10");
//				asm(" VSUB.F32 s2,s2,s12");
//				asm(" VMUL.F32 s2,s2,s0");
//				asm(" VDIV.F32 s2,s2,s19");
//
//            	asm(" VADD.F32 s3,s12,s12");
//				asm(" VSUB.F32 s3,s3,s11");
//				asm(" VSUB.F32 s3,s3,s10");
//				asm(" VMUL.F32 s3,s3,s0");
//				asm(" VDIV.F32 s3,s3,s19");
//
//				asm(" VMOV s10,s1");
//				asm(" VMOV s11,s2");
//				asm(" VMOV s12,s3");
////				s10 = s1;
////				s11 = s2;
////				s12 = s3;
//            	s18 = 0.5773502692;
//
//            	asm(" VADD.F32 s3,s11,s12");
//            	asm(" VDIV.F32 s3,s3,s20");
//            	asm(" VSUB.F32 s3,s10,s3");
//            	asm(" VMUL.F32 s3,s3,s20");
//            	asm(" VDIV.F32 s3,s3,s19"); //alpha
//
//            	asm(" VSUB.F32 s4,s11,s12");
//            	asm(" VMUL.F32 s4,s4,s18"); //beta
//
//                asm_theta = oe_lido;
//            	s1 = asm_cos();
//            	s2 = asm_sin();
//
//            	asm(" VMUL.F32 s5,s1,s3");
//            	asm(" VMUL.F32 s6,s2,s4");
//            	asm(" VADD.F32 s5,s5,s6"); //eixo de quadratura
//
//            	asm(" VMUL.F32 s6,s3,s2");
//            	asm(" VMUL.F32 s7,s1,s4");
//            	asm(" VSUB.F32 s6,s7,s6");//eixo direto
//
//            	asm_q = s5;
//            	asm_d = s6;
//
//                iq_c1 = const_iq + ts/lqs*asm_q;
//                id_c1 = const_id + ts/lds*asm_d;
//                te_c1 = 1.5*fpm*iq_c1;
//                wm_c1 = (1-ts*bm/jm)*wm_c0 + polos_div_2*polos_div_2*ts/jm*(te_c1-tl2);
////                pho=0;
////                id_c1=id_c1*id_c1;
////                iq_c1=iq_c1*iq_c1;
////                if(id_c1>16||iq_c1>16)
////                	pho=1;
//                //0.00000001
//                //0.0000000001
//                Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.00000001)*abs(iq_c1);
////				Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.0000000001)*abs(id_c1)+(0.0000001)*abs(iq_c1);
////				Jc=(wm_ref_rampa-wm_c1)*(wm_ref_rampa-wm_c1)+(0.000000001)*abs(id_c1)+(0.0000001)*abs(iq_c1);
//                if(Jc<=Jmin){
//                    Jmin=Jc;
//                    s1_min=s1c;
//                    s2_min=s2c;
//                    s3_min=s3c;
//                }
//    }
//    vas = (2*s1_min-s2_min-s3_min)*vdc/3.0;
//    vbs = (2*s2_min-s1_min-s3_min)*vdc/3.0;
//    vcs = (2*s3_min-s2_min-s1_min)*vdc/3.0;
//}



void malha_aberta(){
//		wm_ref_rampa = we_ref_rampa/np;
//		wm_lido = we_est/np;
//		oe_lido = oe_est_pll;
//		we_lido =  we_est;

//		wm_ref_rampa = we_ref_rampa/np;
//		wm_lido = we_real/np;
//		oe_lido = oe_real;
//		we_lido =  we_real;



	    theta_a = we_ref_rampa*ts+theta_a;
	    if(theta_a > pi)
	    	theta_a -= 2*pi;
	    if(theta_a < -pi)
	        theta_a += 2*pi;


	    asm_theta = theta_a;
	    vqs_ref = we_ref_rampa/4.0;
	    vds_ref = 0.0;


	    //asm_theta = theta_a;
	    cos_oe_lido = asm_cos();
	    sin_oe_lido = asm_sin();

	    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
	    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

	    vas = valpha_ref;
	    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
	    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void malha_aberta2(){
//		wm_ref_rampa = we_ref_rampa/np;
//		wm_lido = we_est/np;
//		oe_lido = oe_est_pll;
//		we_lido =  we_est;

//		wm_ref_rampa = we_ref_rampa/np;
//		wm_lido = we_real/np;
//		oe_lido = oe_real;
//		we_lido =  we_real;



	    theta_a = we_ref_rampa*ts+theta_a;
	    if(theta_a > pi)
	    	theta_a -= 2*pi;
	    if(theta_a < -pi)
	        theta_a += 2*pi;


	    asm_theta = theta_a;
	    vqs_ref = we_ref_rampa/fator_divisao_ma;
	    vds_ref = 0.0;


	    //asm_theta = theta_a;
	    cos_oe_lido = asm_cos();
	    sin_oe_lido = asm_sin();

	    valpha_ref = cos_oe_lido*vqs_ref+sin_oe_lido*vds_ref;
	    vbeta_ref = -sin_oe_lido*vqs_ref+cos_oe_lido*vds_ref;

	    vas = valpha_ref;
	    vbs = -0.5*valpha_ref-0.866*vbeta_ref;
	    vcs = -0.5*valpha_ref+0.866*vbeta_ref;

}
void teste_mono(){

		theta_a = we_ref_rampa*ts+theta_a;
		if(theta_a > pi)
			theta_a -= 2*pi;
		if(theta_a < -pi)
			theta_a += 2*pi;


		asm_theta = theta_a;
		cos_oe_lido = asm_cos();
		sin_oe_lido = asm_sin();


//	    vas = we_ref_rampa/5.0*cos_oe_lido;
//	    vbs = we_ref_rampa/5.0*cos_oe_lido;
//	    vcs = 0.0;

//	    vbs =  we_ref_rampa/5.0*cos_oe_lido;
//	    vas = 0.0;
//	    vcs = 0.0;

	    vcs =  we_ref_rampa/5.0*cos_oe_lido;
	    vas = 0.0;
	    vbs = 0.0;

}
void teste_bi(){

		theta_a = we_ref_rampa*ts+theta_a;
		if(theta_a > pi)
			theta_a -= 2*pi;
		if(theta_a < -pi)
			theta_a += 2*pi;


		asm_theta = theta_a;
		cos_oe_lido = asm_cos();
		sin_oe_lido = asm_sin();


	    vas = we_ref_rampa/5.0*cos_oe_lido;
	    vbs = we_ref_rampa/5.0*sin_oe_lido;;
	    vcs = 0.0;

//	    vbs =  we_ref_rampa/5.0*cos_oe_lido;
//	    vas = 0.0;
//	    vcs = we_ref_rampa/5.0*sin_oe_lido;

//	    vcs =  we_ref_rampa/5.0*cos_oe_lido;
//	    vas = we_ref_rampa/5.0*sin_oe_lido;
//	    vbs = 0.0;

}
void rotina_parada(){
	flag_parada++;
     //if(flag_rampa < flag_rampa_max+rampa_time){
	 if(flag_parada < rampa_time){
        //we_ref_rampa = -(current_we_ref/rampa_time)*(flag_rampa-flag_rampa_max)+current_we_ref;
    	we_ref_rampa = -(current_we_ref/rampa_time)*flag_parada+current_we_ref;
     }
     else{
        we_ref_rampa = 0;
        flag_protecao = 0;
     }
}
void rampa_unica(){


   if(flag_rampa < rampa_time){
	  we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
   }
   else{
	  we_ref_rampa = we_ref;
	  current_we_ref = we_ref;
   }

}
void senoide_rampa(){

   if(flag_rampa < rampa_time){
	  we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
   }
   else{
	  theta_a += 1;
	  if (theta_a > 1.0/(2*freq_ref*ts))
		  theta_a -= 2.0/(2*freq_ref*ts);
	  asm_theta = 2*pi*theta_a*freq_ref*ts;
	  sin_oe = asm_sin();
	  we_ref_rampa = we_ref+15*sin_oe;
	  current_we_ref = we_ref;
   }

}
void inversao_rampa(){

    float tempo1, tempo2, tempo3;
    float tempo_rampa2, tempo_rampa3;

    float we_ref_inv = -we_ref;

    tempo1 = rampa_time+tempo_espera;
    tempo_rampa2 = 2*rampa_time+tempo1;
    tempo2 = tempo_rampa2+tempo_espera;
    tempo_rampa3 = rampa_time+tempo2;

    if (flag_rampa < rampa_time){
        we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
    }
    if ((flag_rampa>rampa_time)&&(flag_rampa < tempo1)){
        we_ref_rampa = we_ref;
        current_we_ref = we_ref;
    }
    if ((flag_rampa>tempo1)&&(flag_rampa < tempo_rampa2)){
        we_ref_rampa = (we_ref_inv-we_ref)/(2*rampa_time)*(flag_rampa-(tempo1))+we_ref;
    }
    if ((flag_rampa>tempo_rampa2)&&(flag_rampa < tempo2)){
        we_ref_rampa = we_ref_inv;
        current_we_ref = we_ref_inv;
    }
    if ((flag_rampa>tempo2)&&(flag_rampa < tempo_rampa3)){
        we_ref_rampa = (we_ref)/(rampa_time)*(flag_rampa-(tempo2))+we_ref_inv;
    }
    if (flag_rampa>tempo_rampa3){
        we_ref_rampa = 0.0;
        current_we_ref = 0.0;
    }

}
void multiplas_rampas(){

	float tempo1, tempo2, tempo3,tempo4;
	float tempo_rampa2, tempo_rampa3,tempo_rampa4,tempo_rampa5;


    tempo1 = rampa_time+tempo_espera;
    tempo_rampa2 = rampa_time+tempo1;
    tempo2 = tempo_rampa2+tempo_espera;
    tempo_rampa3 = rampa_time+tempo2;
    tempo3 = tempo_rampa3+tempo_espera;
    tempo_rampa4 = rampa_time+tempo3;
    tempo4 = tempo_rampa4+tempo_espera;
    tempo_rampa5 = rampa_time+tempo4;

    if (flag_rampa < rampa_time){
        we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
    }
    if ((flag_rampa>rampa_time)&&(flag_rampa < tempo1)){
        we_ref_rampa = we_ref;
        current_we_ref = we_ref;
    }
    if ((flag_rampa>tempo1)&&(flag_rampa < tempo_rampa2)){
        we_ref_rampa = (we_ref2-we_ref)/(rampa_time)*(flag_rampa-(tempo1))+we_ref;
    }
    if ((flag_rampa>tempo_rampa2)&&(flag_rampa < tempo2)){
        we_ref_rampa = we_ref2;
        current_we_ref = we_ref2;
    }
    if ((flag_rampa>tempo2)&&(flag_rampa < tempo_rampa3)){
        we_ref_rampa = (we_ref3-we_ref2)/(rampa_time)*(flag_rampa-(tempo2))+we_ref2;
    }
    if (flag_rampa>tempo_rampa3){
        we_ref_rampa = we_ref3;
        current_we_ref = we_ref3;
    }
    if ((flag_rampa>tempo3)&&(flag_rampa < tempo_rampa4)){
        we_ref_rampa = (we_ref4-we_ref3)/(rampa_time)*(flag_rampa-(tempo3))+we_ref3;
    }
    if (flag_rampa>tempo_rampa4){
        we_ref_rampa = we_ref4;
        current_we_ref = we_ref4;
    }
    if ((flag_rampa>tempo4)&&(flag_rampa < tempo_rampa5)){
		we_ref_rampa = (we_ref5-we_ref4)/(rampa_time)*(flag_rampa-(tempo4))+we_ref4;
	}
	if (flag_rampa>tempo_rampa5){
		we_ref_rampa = we_ref5;
		current_we_ref = we_ref5;
	}


    if (current_we_ref > 5.0*polos_div_2)
    	alpha_2ord = 0.05;
    else
    	alpha_2ord = 0.025;
}
void mini_degraus(){

	float tempo1, tempo2, tempo3,tempo4, tempo5;
	float degrau1,degrau2,degrau3,degrau4,degrau5;

//	alpha_2ord = 0.008;

    tempo1 = 2*tempo_espera;
    tempo2 = 4*tempo_espera;
    tempo3 = 6*tempo_espera;
    tempo4 = 8*tempo_espera;
    tempo5 = 10*tempo_espera;

    degrau1 = 0.1*polos_div_2;
    degrau2 = polos_div_2*0.3;
    degrau3 = polos_div_2*0.2;
    degrau4 = polos_div_2*0.4;
    degrau5 = -polos_div_2*0.2;


    if ((flag_rampa>0)&&(flag_rampa < tempo1)){
    	we_ref_rampa = degrau1;
    	current_we_ref = degrau1;
    }
    if ((flag_rampa>tempo1)&&(flag_rampa < tempo2)){
        we_ref_rampa = degrau2;
        current_we_ref = degrau2;
    }
    if ((flag_rampa>tempo2)&&(flag_rampa < tempo3)){
		we_ref_rampa = degrau3;
		current_we_ref = degrau3;
	}
    if ((flag_rampa>tempo3)&&(flag_rampa < tempo4)){
		we_ref_rampa = degrau4;
		current_we_ref = degrau4;
	}
    if ((flag_rampa>tempo4)&&(flag_rampa < tempo5)){
		we_ref_rampa = degrau5;
		current_we_ref = degrau5;
	}
    if (flag_rampa>tempo5){
    	we_ref_rampa = 0.0;
    	current_we_ref = 0.0;
    }

}
void gerenciamento_referencia_e_tempo(){

    if(stop_flag == 1){
        rampa_unica();
        //multiplas_rampas();
        //inversao_rampa();
        //senoide_rampa();
        //mini_degraus();
    }
    else{
        rotina_parada();

        if (flag_rampa > flag_rampa_max+rampa_time)
            flag_rampa = flag_rampa_max+rampa_time;

    }
    if (flag_final >= final_time_flag)
        flag_protecao = 0;

}

int main(void)
  {


  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_ADC3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  inicia_dac();
  escreve_dac1(0);
  escreve_dac2(0);

  inicia_pwm();
  inicia_encoder();
  inicia_interrupcao();
  inicia_float();

  Vdc = 128.0/1.73;

  ampA = 0.3;
  ampB = 0.3;
  ampC = 0.3;

  flag_protecao = 1;
  Imax = 3.0;
  ias_lido = 0.0;
  ibs_lido = 0.0;
  ics_lido = 0.0;
  wm_lido = 0.0;
  wm_filt = 0.0;
  we_real = 0.0;
  we_lido = 0.0;
  zwe = 0.0;
  zids = 0.0;
  ziqs = 0.0;


  posicao_minima = pi/(2*PPR_Encoder);

  rampa_time = 60000.0;
//  rampa_time = 10000;
  tempo_espera = 30000;
  flag_rampa_max = 500000;
  final_time_flag = 10*flag_rampa_max;

  fator_divisao_ma = 4.0;

  flag_final = 0;
  flag_rampa = 0;

//  wm_ref = 2.5;
//  we_ref = polos_div_2*wm_ref;

  wm_ref = 50.0/21.0;
  we_ref = polos_div_2*wm_ref;

  wm_ref2 = 4.0;
  we_ref2 = polos_div_2*wm_ref2;

  wm_ref3 = 6.0;
  we_ref3 = polos_div_2*wm_ref3;

  wm_ref4 = 8.0;
  we_ref4 = polos_div_2*wm_ref4;

  wm_ref5 = 0.0;
  we_ref5 = polos_div_2*wm_ref5;

  current_we_ref = we_ref;

  transicao_time = 50000;
  freq_ref = 1;


  //parametros slinding mode
  est_b = ts/lqs;
  est_a = 1.0-ts/lqs*rs;
  est_c = fpm/lqs;

  //ganhos sliding mode
//  lsliding = 0.001;
//  ksliding_ref = 1.8*fpm*wm_ref*ParPolos;
//  ksliding = ksliding_ref/100.0;
//  //filtro sliding mode
//  wcsliding = 2*pi*50;
//  E0 = ksliding_ref/50.0;

//  lsliding = 75.0;
//  ksliding_ref = 75;
//  E0 = 0.02;
//  ksliding = ksliding_ref/100.0;
//  //filtro sliding mode
//  wcsliding = 2*pi*30;


//  //pi pll
//  kp_pll = 70;
//  ki_pll = 70000;
//  //filtro pll
//  wcpll = 2*pi*15.0;
//  wcpll2 = wcpll*wcpll;
//  xipll = 2.0;

  lsliding = 0.001;
  ksliding_ref = 1.8*fpm*wm_ref*ParPolos;
  ksliding = ksliding_ref/100.0;
  //filtro sliding mode
  wcsliding = 2*pi*30.0;
  E0 = ksliding_ref/50.0;

  //pi pll
  kp_pll = 70;
  ki_pll = 70000;
  //filtro pll
  wcpll = 2*pi*5.0;
  wcpll2 = wcpll*wcpll;
  xipll = 2.0;

  vdc = Vdc;
  v0 = 2.0;
  vmax = 0.9*vdc;
  vmin = -0.9*vdc;
  //zwe = 274.6;
  ids_ref = 0.0;

  //asm_operando = alpha;
  //alpha_2ord = asm_sqrt();
  alpha_2ord = alpha;

  //ganhos controle vetorial
//  kp_wm = 0.3;
//  ki_wm = 200.0;
//  kp_ids = 5.0;
//  ki_ids = 0.0;
//  kp_iqs = 5.0;
//  ki_iqs = 0.0;
//  kp_wm = 100;
//  ki_wm = 50;
//  kp_ids = 100;
//  ki_ids = 50;
//  kp_iqs = 100;
//  ki_iqs = 50;

//  kp_wm = 0.09;
//  ki_wm = 0.02;
//  kp_ids = 10;
//  ki_ids = 3000.0;
//  kp_iqs = 10;
//  ki_iqs = 3000.0;


//  //ganhos controle alphabeta
//10 kHz
  //  kp_wm = 0.5;
//  ki_wm = 50;
//  kp_ids = 50;
//  ki_ids = 5.0;
//  kp_iqs = 50;
//  ki_iqs = 5.0;

  //5kHz
  kp_wm = 0.4;
  ki_wm = 40;
  kp_ids = 40;
  ki_ids = 4;
  kp_iqs = 40;
  ki_iqs = 4;

//  kp_wm = 0.25;
//  ki_wm = 25;
//  kp_ids = 25;
//  ki_ids = 2.5;
//  kp_iqs = 25;
//  ki_iqs = 2.5;

//  kp_wm = 0.25;
//  ki_wm = 30;
//  kp_ids = 20;
//  ki_ids = 2;
//  kp_iqs = 20;
//  ki_iqs = 2;


  //realimentacao de estados/preditivo
//  b31 = (0.75*fpm*np*np*ts*ts)/(jm*lqs);
//  a13 = (0.5*fpm*fpm*ts*ts)/(lqs*lqs)-(fpm*ts)/lqs;
//  a33 = (0.5*bm*bm*ts*ts)/(jm*jm)-(bm*ts)/jm+1;
//  a32i = (3*(lds-lqs)*np*np*ts)/(2*jm);
//  a31 = (1.125*fpm*fpm*np*np*np*np*ts*ts)/(jm*jm)+(3*fpm*np*np*ts)/(2*jm);
//  b22 = ts/lds-(0.5*rs*ts*ts)/(lds*lds);
//  a22 = (0.5*rs*rs*ts*ts)/(lds*lds)-(rs*ts)/lds+1;
//  b11 = ts/lqs-(0.5*rs*ts*ts)/(lqs*lqs);
//  a11 = 1-rs*ts/lqs+0.5*rs*rs*ts*ts/(lqs*lqs);
//  b32i = (0.75*(lds-lqs)*np*np*ts*ts)/(jm*lds);
//  b12w = 0;
//  b21w = 0;
//  a12w = -lds*ts/lqs;
//  a21w = (lqs*ts)/lds;
//  a51 = (lds-lqs)/fpm;

//  kr_we = a33/b31*0.0041;
//  kr_ids = a22/b22*0.0051;
//  kx_iqs = -a11/b11*0.02;
//  kx_we = -a33/b31*0.004;
//  kx_ids = -a22/b22*0.005;
//  kz_we = a33/b31*0.00001;
//  //kz_we = 0;
//  kz_ids = 1.0/b22*0.001;

//  kr_we = a33/b31*0.041;
//  kr_ids = a22/b22*0.051;
//  kx_iqs = -a11/b11*0.2;
//  kx_we = -a33/b31*0.04;
//  kx_ids = -a22/b22*0.05;
//  //kz_we = a33/b31*0.001;
//  kz_we = 0;
//  kz_ids = 1.0/b22*0.005;

//  mu_zids = 0.0005;
//  rho_vds = 0.0005;

//  mu_tune = b31*b31/10.0;
//  rho_tune = -a31*a13/5.0;

//  mu_tune = b31*b31/10.0;
//  rho_tune = -a31*a13/5.0;
//
//  mu_zwe = mu_tune;
//  rho_vqs = rho_tune;
//
//  kr_we = (rho_vqs+mu_zwe)/(b31*rho_vqs+b31*mu_zwe+b31);
//  kr_ids = (rho_vds+mu_zids)/(b22*rho_vds+b22*mu_zids+b22);
//  kx_iqs = -((a31)*mu_zwe)/(b31*rho_vqs+b31*mu_zwe+b31);
//  kx_we = -(a33*rho_vqs+a33*mu_zwe)/(b31*rho_vqs+b31*mu_zwe+b31);
//  kx_ids = -(a22*rho_vds+a22*mu_zids)/(b22*rho_vds+b22*mu_zids+b22);
//  kz_we = mu_zwe/(b31*rho_vqs+b31*mu_zwe+b31);
//  kz_ids = mu_zids/(b22*rho_vds+b22*mu_zids+b22);

  //ganhos malha direta
  //ru = 5000, rw = 0.01, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
//  kr_we = 0.07463444522195675;
//  kx_iqs = 1.3990570180155595;
//  kx_we = 0.07349423327609794;
//  kz_we = 0.0009134589677089783;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.058582833659468425;

  //ru = 7500, rw = 0.01, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
  kr_we = 0.049792031746068624;
  kx_iqs = 0.9333811885703879;
  kx_we = 0.04903133759889404;
  kz_we = 0.000609403893382213;
  kr_ids = 3.1395581584915924;
  kx_ids = 3.1114289065710046;
  kz_ids = 0.058582833659468425;

  //ru = 75000, rw = 0.01, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
//  kr_we = 0.004985648604177283;
//  kx_iqs = 0.0934601394531096;
//  kx_we = 0.004909479473132608;
//  kz_we = 6.101817259856771e-5;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.058582833659468425;

  //ru = 2500, rw = 0.01, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
//  kr_we = 0.1489482108125724;
//  kx_iqs = 2.792043144601335;
//  kx_we = 0.14667274810728764;
//  kz_we = 0.001823047980392428;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.058582833659468425;

  //ru = 1000, rw = 0.01, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
//  kr_we = 0.3699862643062683;
//  kx_iqs = 6.934970634650744;
//  kx_we = 0.3643344940784033;
//  kz_we = 0.004528846687484774;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.058582833659468425;

  //ru = 10000, rw = 0.001, ru_ids = 161.99864140883597, rw_ids = 0.01901437937113647
//  kr_we = 0.03661771843561937;
//  kx_iqs = 0.6887703852227522;
//  kx_we = 0.036055847136544236;
//  kz_we = 4.5722205670271856e-5;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.058582833659468425;

//   kr_we = 0.07463444522195675;
//   kx_iqs = 1.3990570180155595;
//   kx_we = 0.07349423327609794;
//   kz_we = 0.0000;
//   kr_ids = 3.1395581584915924;
//   kx_ids = 3.1114289065710046;
//   kz_ids = 0.00;

//  kr_we = 0.07463444522195675;
//  kx_iqs = 1.3990570180155595;
//  kx_we = 0.07349423327609794;
//  kz_we = 0.00001;
//  kr_ids = 3.1395581584915924;
//  kx_ids = 3.1114289065710046;
//  kz_ids = 0.00001;

  dutya = 1198;
  dutyb = 1198;
  dutyc = 1198;

  HAL_Delay(3000); //Delay para poder Debug no STMStudio

  ajuste_theta0();
  //reinicializa_encoder();
  Timer_Enconder_Value = 0;
  Timer_Enconder_Value0 = 0;
  __HAL_TIM_SET_COUNTER(&htim5,0);
  //HAL_Delay(200);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {


	   if(flag_tim3==1){
		  flag_tim3 = 0;
		  kk++;
		   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		   protecao_corrente();
	 	   //le_corrente();
		   le_corrente_corrigida();

	 	   flag_rampa++;
	 	   flag_final++;

	 	  gerenciamento_referencia_e_tempo();
//	 	   if(stop_flag == 1){
//	 	   //rampa com varias variacoes
//	 	   if(flag_rampa < rampa_time){
//	 		  we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
//	 	   }
//		   else{
//			  if (flag_rampa < rampa_time+tempo_espera){
//				  we_ref_rampa = we_ref;
//	 		   	  current_we_ref = we_ref;
//			  }
//			  else{
//				  if (flag_rampa < 2*rampa_time+tempo_espera){
//					  we_ref_rampa = (we_ref2-we_ref)/(rampa_time)*(flag_rampa-(rampa_time+tempo_espera))+we_ref;
//				  }
//				  else{
//					  if (flag_rampa < 2*rampa_time+2*tempo_espera){
//						  we_ref_rampa = we_ref2;
//			 		   	  current_we_ref = we_ref2;
//					  }
//					  else{
//						  if (flag_rampa < 3*rampa_time+2*tempo_espera){
//							  we_ref_rampa = (we_ref3-we_ref2)/(rampa_time)*(flag_rampa-(2*rampa_time+2*tempo_espera))+we_ref2;
//						  }
//						  else{
//							  we_ref_rampa = we_ref3;
//			 		   	  	  current_we_ref = we_ref3;
//						  }
//
//					  }
//				  }
//			  }
//		   }
//
//	 	   //rampa unica
////		   if(flag_rampa < rampa_time){
////			  we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
////		   }
////		   else{
////			  we_ref_rampa = we_ref;
////			  current_we_ref = we_ref;
////		   }
//
//		   //senoide de rotacao apos rampa
////		   if(flag_rampa < rampa_time){
////			  we_ref_rampa = (we_ref/rampa_time)*flag_rampa;
////		   }
////		   else{
////			  theta_a += 1;
////			  if (theta_a > 1.0/(2*freq_ref*ts))
////				  theta_a -= 2.0/(2*freq_ref*ts);
////			  asm_theta = 2*pi*theta_a*freq_ref*ts;
////			  sin_oe = asm_sin();
////			  we_ref_rampa = we_ref+15*sin_oe;
////		   }
//
//		   	   if (flag_rampa > flag_rampa_max)
//		   	 		  flag_rampa = flag_rampa_max;
//	 	   }
//	 	   else{
//	 		  if(flag_rampa < flag_rampa_max+rampa_time){
//				  we_ref_rampa = -(current_we_ref/rampa_time)*(flag_rampa-flag_rampa_max)+current_we_ref;
//			   }
//			   else{
//				  we_ref_rampa = 0;
//				  flag_protecao = 0;
//			   }
//	 		 if (flag_rampa > flag_rampa_max+rampa_time)
//	 		 	 flag_rampa = flag_rampa_max+rampa_time;
//	 	   }
//
//	 	   if (flag_final >= final_time_flag)
//	 		   flag_protecao = 0;

	 	   if (flag_protecao == 1){

	 		  //medicao_velocidade();
	 		 	medicao_velocidade_corrigido();

				ialpha_lido = 2.0/3.0*(ias_lido-0.5*ibs_lido-0.5*ics_lido);
				ibeta_lido = 2.0/3.0*(0.866*ics_lido-0.866*ibs_lido);

				valpha = 2.0/3.0*(vas-0.5*vbs-0.5*vcs);
				vbeta = 2.0/3.0*(0.866*vcs-0.866*vbs);

				valpha_ref = valpha;
				vbeta_ref = vbeta;

				//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
				//if (flag_rampa > rampa_time)
				//wcpll2 = wcpll*wcpll;
				//estimador_sliding_mode();
				//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

				//controle_vetorial_sensorless();
//				if(flag_rampa < rampa_time+25000){
//					asm_theta = theta_a;
//					controle_realimentacao_estados();
//				}
//				else{
//					//controle_vetorial_sensorless();
//					flag_transicao++;
//					if(flag_transicao < transicao_time)
//						asm_theta = (1-flag_transicao/transicao_time)*theta_a+flag_transicao/transicao_time*theta_e;
//					else
//						asm_theta = theta_e;
//					controle_realimentacao_estados();
//				}
//				monitoramento_flag_transicao = flag_transicao/1000.0;



				//controle_preditivo_linear_malha_direta();
				//controle_preditivo_linear_cascata();
				//controle_preditivo_linear_cascata_com_compensacao_parcial();
				//controle_preditivo_linear_cascata_com_compensacao_total();
				//controle_preditivo_linear_malha_direta_com_estPBC();
				//controle_preditivo_linear_cascata_malha_internaPBC();
				controle_preditivo_fcs();
				//controle_preditivo_nao_linear_cascata_euler_1aordN1();

				//malha_aberta();
				//malha_aberta2();
				//teste_mono();
				//teste_bi();
				//controle_realimentacao_estados();
//				controle_vetorial_sensorless();
//				controle_alphabeta();
//				if(flag_rampa > rampa_time+100000){
//					vmax = 40;
//					vmin = 20;
//				}





//				Fluxo_alpha = (valpha-Rs*ialpha_lido)*ts+Fluxo_alpha;
//				if (Fluxo_alpha > 10.0)
//					Fluxo_alpha = 10.0;
//				if (Fluxo_alpha < -10.0)
//					Fluxo_alpha = -10.0;
//
//				Fluxo_beta = (vbeta-Rs*ibeta_lido)*ts+Fluxo_beta;
//				if (Fluxo_beta > 10.0)
//					Fluxo_beta = 10.0;
//				if (Fluxo_beta < -10.0)
//					Fluxo_beta = -10.0;
//
//				asm_operando = Fluxo_alpha*Fluxo_alpha+Fluxo_beta*Fluxo_beta;
//
//				Fluxo = asm_sqrt();
//
//				erro_fluxo = Fluxo_ref-Fluxo;
//				erro_fluxo_a = erro_fluxo_a+erro_fluxo*ts;
//				dv = kp_fluxo*erro_fluxo+ki_fluxo*erro_fluxo_a;
//				//dv = kp_fluxo*erro_fluxo;
//				//vp = kvf*we_ref_rampa/(2*pi)+v0;
//				vp = 25.0;
//
//				//vp_ref = vp+dv;
//				vp_ref = vp;
//				if (vp_ref >= 0.9*Vdc)
//					vp_ref = 0.9*Vdc;
//				if (vp_ref <= -0.9*Vdc)
//					vp_ref = -0.9*Vdc;
////
//				vp_ref = 8.0;
//				tk +=ts;
//				if (tk >= 42*pi/21)
//					tk = 0;
//
//				theta_a = 21*tk;
//				if(theta_a >= 2*pi)
//					tk = 0.0;
//				if(theta_a <= -2*pi)
//					tk = 0.0;
////
//				if(theta_a>pi)
//					theta_a-=2*pi;
//				if(theta_a<-pi)
//					theta_a+=2*pi;
//
//				theta_b = theta_a-2*pi/3;
//				if(theta_b>pi)
//					theta_b-=2*pi;
//				if(theta_b<-pi)
//					theta_b+=2*pi;
//
//				theta_c = theta_a+2*pi/3;
//				if(theta_c>pi)
//					theta_c-=2*pi;
//				if(theta_c<-pi)
//					theta_c+=2*pi;
////
//				asm_theta = theta_a;
//				vas = vp_ref*asm_cos();
//
//				asm_theta = theta_b;
//				vbs = vp_ref*asm_cos();
//
//				asm_theta = theta_c;
//				vcs = vp_ref*asm_cos();
//
//
				ampA = (0.5-0.5*vas/Vdc);
				ampB = (0.5-0.5*vbs/Vdc);
				ampC = (0.5-0.5*vcs/Vdc);

				if(ampA >= 0.95) ampA = 0.95;
				if(ampB >= 0.95) ampB = 0.95;
				if(ampC >= 0.95) ampC = 0.95;

				if(ampA <= 0.05) ampA = 0.05;
				if(ampB <= 0.05) ampB = 0.05;
				if(ampC <= 0.05) ampC = 0.05;

				dutya = (int)(1199*ampA);
				dutyb = (int)(1199*ampB);
				dutyc = (int)(1199*ampC);

				if(flag_para == 0){
					dutya = 1198;
					dutyb = 1198;
					dutyc = 1198;
				}

//				dutya = 1170;
//				dutyb = 1198;
//				dutyc = 1198;

				escreve_dutya(dutya);
				escreve_dutyb(dutyb);
				escreve_dutyc(dutyc);

//				temp2 = (int)200*(theta_e+3*pi);
//				temp1 = (int)200*(-oe_est_pll+3*pi);

//				temp2 = (int)10*(valpha_ref+2*vmin);
//				temp1 = (int)10*(vbeta_ref+2*vmin);

//				temp2 = (int)10*(ealpha_est+2*vmin);
//				temp1 = (int)10*(ebeta_est+2*vmin);

//				temp1 = (int)(620*ialpha_lido+1024);
//				temp2 = (int)(620*ibeta_lido+1024);



				temp1 = (int)15*(we_real)+540;
				temp2 = (int)15*(we_ref_rampa)+540;

				//temp2 = (int)(theta_e*180/pi+540);
				//temp2 = (int)(iqs_lido*1000+540);

//				temp1 = (int)(oe_real*180/pi+540);
//				temp2 = (int)(theta*180/pi+540);


//				temp2 = (int)200*(ialpha_lido+2.0);
//				temp1 = (int)200*(ialpha_ref+2.0);

//				if (temp2 < 0)
//					temp2 = 0;
				escreve_dac1(temp1);
				escreve_dac2(temp2);

				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	 	   }
	 	   else{
	 			trata_sobrecorrente();
	 	   }
	  }
}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
