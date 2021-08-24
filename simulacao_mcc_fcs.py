# -*- coding: utf-8 -*-
"""
Created on Tue May 25 08:59:37 2021

@author: 10695421999
"""

#####################################
#SIMULAÇÃO DO MOTOR DE INDUÇÃO CONTROLADO PELA REDE NEURAL
#O CÓDIGO É DIVIDIDO EM 3 PARTES: INICIALIZAÇÃO, SIMULAÇÃO E PLOTS
#####################################

#####################################
#IMPORTA OS PACOTES
#
#####################################

import numpy as np
import time
from math import sqrt
from math import pi
from math import sin
from math import cos
import matplotlib.pyplot as plt
np.random.seed(1)

print("começou\n")
tempo_inicial=time.time()
#####################################
#PRIMEIRA PARTE DO CÓGIGO:
#INICIALIZAÇÃO
#####################################
#PARAMETROS DA SIMULAÇÃO
dt = 1e-5
ts=0.0002
razao_ts_dt=round(ts/dt)
tmax = 30.0-dt
npontos = round(tmax/dt)
i = 0
# # #PARAMETROS DO MOTOR QUE A VANESSA TRABALHA
ra = 13.94
la = 0.08017
ka = 0.3195
jm = 0.012
bm = 0.00881
vdc = 64

#VARIÁVEIS
va = 0
ia = 0

we = 0
wr = 0
wm = 0
te = 0
tl = 0
oe = 0
om = 0

#INICIALIZAÇÃO DOS VETORES PARA O ARMEZENAMENTO DA INFORMAÇÃO
Va = np.zeros(npontos)
Ia = np.zeros(npontos)

We = np.zeros(npontos)
Wr = np.zeros(npontos)
Wm = np.zeros(npontos)
Te = np.zeros(npontos)
Tl = np.zeros(npontos)
Oe = np.zeros(npontos)
Om = np.zeros(npontos)

T = np.zeros(npontos)
Sa= np.zeros(npontos)
Sb= np.zeros(npontos)
Sc= np.zeros(npontos)
Vrms = np.zeros(npontos)
Wb = np.zeros(npontos)
Wref = np.zeros(npontos)
Tlest = np.zeros(npontos)

#MAIS ALGUMAS VARIAVEIS ALTERADAS COM MAIS FREQUÊNCIA
wrefr = 40 
wref = 0

for j in range(1, npontos):

    t = j*dt
    wref = wrefr*t/(6)
    if (t >= 6):
        wref = wrefr
    Wref[j] = wref

j=0
tl = 0
sa=0
sb=0
sc=0

flag=5
Jmin=1e20
indice_min=0
flag2=0
n=1
tl_est = 0
s3=0
s1_min=0
s2_min=0
vas=0
vbs=0
print("inicialização concluída\n")
##############################################################################
#####################################
#SEGUNDA PARTE DO CÓGIGO:
#SIMULAÇÃO
#####################################
for t_int in range(npontos):
    t=t_int*dt
    ##########################################################
    ############################
    if t >= 0.6*tmax:
        tl = 0.1
    ##
    #ALIMENTAÇÃO DO SISTEMA
    if flag>=razao_ts_dt:#TESTE DO TEMPO DE CONTROLE
        #LOOPS DE CONTROLE UM PRA K+1E OUTRO PRA K+2
        #LOOPS DE CONTROLE UM PRA K+1E OUTRO PRA K+2
        ia_c0=ia + 0.02*(-0.5+np.random.random())
        wm_c0=wm + 1*(-0.5+np.random.random())
        Jmin=100000000000000
        tl_est=tl_est+8*ts*(Wref[i]-wm_c0)
        wm_c1=(1-ts*bm/jm)*wm_c0+(ka*ia_c0-tl_est)*ts/jm
        for s1 in range(0,2):
            for s2 in range(0,2):
                vas_c = (2*s1-s2)*vdc/3.0
                vbs_c = (2*s2-s1)*vdc/3.0
                ia_c1=(1-ra*ts/la)*ia_c0-ts*ka/la*wm_c0+(vas_c-vbs_c)*ts/la
                
                wm_c2=(1-ts*bm/jm)*wm_c1+(ka*ia_c1-tl_est)*ts/jm
                Jc=(wm_c2-Wref[i])*(wm_c2-Wref[i])+0.0075*ia_c1*ia_c1
    
                if Jc<=Jmin:
                    Jmin=Jc
                    s1_min=s1
                    s2_min=s2

        vas = (2*s1_min-s2_min)*vdc/3.0
        vbs = (2*s2_min-s1_min)*vdc/3.0
        j+=1
        flag=0
        #EM CÓDIGOS QUE DEMORAM MUITO FICA DIFICIL SABER
        #SE TRAVOU OU SE TÁ RODANDO AINDA
        #ESSA LÓGICA MOSTRA QUANTOS % DO CÓDIGO PASSOU
        if flag2>=npontos*0.05*n/razao_ts_dt:
            print(n*5,"% concluído\n")
            n+=1
        flag2+=1
    #ENTRADA DA ALIMENTAÇÃO DO SISTEMA, QUE VEM DO CONTROLE

    ia_t=ia
    ia=(1-ra*dt/la)*ia-dt*ka/la*wm+(vas-vbs)*dt/la
    wm=(1-dt*bm/jm)*wm-dt/jm*tl+ka*ia_t*dt/jm
    
    va=0.0001*(vas-vbs)+0.9999*va
    te=ka*ia
    oe = oe+dt*wm
    if oe > 2*pi:
        oe = oe-2*pi

    if oe < 0:
        oe = oe+2*pi
        
    ##########################################################################
    #LEITURA DAS VARIÁVEIS
    Sa[i]=sa
    Sb[i]=sb
    Sc[i]=sc

    Va[i] = va
    Ia[i] = ia

    We[i] = we
    Wr[i] = wr
    Wm[i] = wm
    Te[i] = te
    Tl[i] = tl
    Oe[i] = oe
    Tlest[i] = tl_est
    T[i] = t

    i = i+1
    flag+=1


print('demorou ',time.time()-tempo_inicial,' s\n\n\n')
#####################################
#TERCEIRA PARTE DO CÓGIGO:
#PLOT E/OU SALVAR OS DADOS
#####################################

plt.rc('text', usetex=False)
plt.close("all")


plt.figure(3)
plt.plot(T,Ia,color='C0',label=r'$i_{as}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i[A]$")
plt.title(r"Corrente")
plt.grid()
plt.legend()
plt.savefig("sim_mcc_corrente.pdf", format='pdf')

plt.figure(5)
plt.plot(T, Wref, color='k', label=r"$\omega_m^{*}$")
plt.plot(T, Wm, color='C0', label=r"$\omega_m$")
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$\omega_m (t)$ [rad/s]")
plt.title(r"Velocidade Angular")
plt.grid()
plt.legend()
plt.savefig("sim_mcc_velocidade.pdf", format='pdf')

plt.show()


# plt.figure(1)
# plt.plot(T,Sa,color='C0',label=r'Sa')
# plt.plot(T,Sb,color='C1',label=r'Sb')
# plt.plot(T,Sc,color='C2',label=r'Sc')
# plt.xlabel(r"$t$[s]")
# plt.ylabel(r"$Estado das chaves$")
# plt.title(r"Ação de Controle")
# plt.grid()
# plt.legend()


# plt.figure(2)
# plt.plot(T,Va,color='C0',label=r'Va')
# plt.xlabel(r"$t$[s]")
# plt.ylabel(r"$Tensão Aplicada$")
# plt.title(r"Ação de Controle")
# plt.grid()
# plt.legend()

# plt.figure(4)
# plt.plot(T,Tl,color='C0',label=r'Tl')
# plt.plot(T,Tlest,color='C1',label=r'Tlest')
# plt.xlabel(r"$t$[s]")
# plt.ylabel(r"$Corrente$")
# plt.title(r"Corrente")
# plt.grid()
# plt.legend()

# plt.figure(4)
# plt.plot(Wm,Te,color='C0')
# plt.xlabel(r"$Speed$[rpm]")
# plt.ylabel(r"Torque")
# plt.title(r"Angular Speed")
# plt.grid()
# plt.legend()
