# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 14:12:27 2021

@author: 10695421999
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 14:03:49 2021

@author: 10695421999
"""

import numpy as np
import time
from math import sqrt
from math import pi
from math import sin
from math import cos
import matplotlib.pyplot as plt
# from mpmath import *
from numpy import random
# mp.dps = 25; mp.pretty = True
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 10 16:05:14 2021

@author: 10695421999
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Jun 10 15:12:28 2021

@author: 10695421999
"""

# -*- coding: utf-8 -*-
"""
Created on Wed Jun  9 14:29:01 2021

@author: 10695421999
"""

# -*- coding: utf-8 -*-
"""
Created on Fri May 21 08:11:39 2021

@author: 10695421999
"""

# -*- coding: utf-8 -*-
"""
Created on Thu May 20 10:18:10 2021

@author: 10695421999
"""

np.random.seed(1)

print("começou\n")
tempo_inicial = time.time()
#####################################
# PRIMEIRA PARTE DO CÓGIGO:
# INICIALIZAÇÃO
#####################################
# PARAMETROS DA SIMULAÇÃO
dt = 1e-5
ts = 0.0001
razao_ts_dt = round(ts/dt)
tmax = 12-dt
npontos = round(tmax/dt)
i = 0

# # #PARAMETROS DA TESE DO CAVALCA

# Rs = 2.50
# Rr = 2.24
# Ls = 0.288
# Lr = 0.288
# Lm = 0.27
# jm = 0.0135
# bm = 0.0027
# vrms = 310

# parametros vanessa
Rs = 20.6
Rr = 20.6
Ls = 0.82425
Lr = 0.82425
Lm = 0.78
jm = 0.012
bm = 0.00881
vrms = 192

P = 4 # POLOS OU PAR DE POLOS?
Pd2 = P/2
# Ls = Lm+lls
# Lr = Lm+llr
sig = Lr*Ls-Lm**2

# VARIÁVEIS
vas = 0
vbs = 0
vcs = 0
valphas = 0
vbetas = 0

var = 0
vbr = 0
vcr = 0
valphar = 0
vbetar = 0

fas = 0
fbs = 0
fcs = 0
falphas = 0
fbetas = 0

far = 0
fbr = 0
fcr = 0
falphar = 0
fbetar = 0

ias = 0
ibs = 0
ics = 0
ialphas = 0
ibetas = 0
ids = 0
iqs = 0
izs = 0

iar = 0
ibr = 0
icr = 0
ialphar = 0
ibetar = 0
idr = 0
iqr = 0

we = 0
wr = 0
wm = 0
te = 0
tl = 0
# tl = tb
oe = 0
om = 0
#####################
# CÁLCULO DAS TENSÕES DO INVERSOR (IGUAL AO CÓDIGO DA SABRINA)
def  inversor_ideal(s1, s2, s3, vdc):
    vas = (2.0/3.0*s1-1.0/3.0*s2-1.0/3.0*s3)*vdc
    vbs = (2.0/3.0*s2-1.0/3.0*s1-1.0/3.0*s3)*vdc
    vcs = (2.0/3.0*s3-1.0/3.0*s2-1.0/3.0*s1)*vdc
    return vas, vbs, vcs

# ROTINA PARA O CHAVEAMENTO (IGUAL AO CÓDIGO DA SABRINA)
def  chaveamento(indice_min):
    if indice_min == 1:
        sw1 = 1
        sw2 = 0
        sw3 = 0
    elif indice_min == 2:
        sw1 = 0
        sw2 = 1
        sw3 = 0
    elif indice_min == 3:
        sw1 = 1
        sw2 = 1
        sw3 = 0
    elif indice_min == 4:
        sw1 = 0
        sw2 = 0
        sw3 = 1
    elif indice_min == 5:
        sw1 = 1
        sw2 = 0
        sw3 = 1
    elif indice_min == 6:
        sw1 = 0
        sw2 = 1
        sw3 = 1
    else:
        sw1 = 0
        sw2 = 0
        sw3 = 0
    return sw1, sw2, sw3


def abg_transform(a, b, c):
    alpha = (2*a-b-c)/3
    beta = sqrt(3)*(b-c)/3
    return alpha, beta


def inv_ab_transf(alpha, beta):
    a = alpha
    b = -alpha/2+beta*sqrt(3)/2
    c = -alpha/2-beta*sqrt(3)/2
    return a, b, c


def dq_transf(a, b, c, oe):
    d = cos(oe)*a+cos(oe-2*pi/3)*b+cos(oe+2*pi/3)*c
    q = sin(oe)*a+sin(oe-2*pi/3)*b+sin(oe+2*pi/3)*c
    # z=(a+b+c)/3
    return 2/3*d, 2/3*q


def invdq_transf(q, d, oe):
    a = sin(oe)*d + cos(oe)*q
    b = sin(oe-2*pi/3)*d + cos(oe-2*pi/3)*q
    c = sin(oe+2*pi/3)*d + cos(oe+2*pi/3)*q
    return a, b, c

# INICIALIZAÇÃO DOS VETORES PARA O ARMEZENAMENTO DA INFORMAÇÃO


Vas = np.zeros(npontos)
Vbs = np.zeros(npontos)
Vcs = np.zeros(npontos)
Valphas = np.zeros(npontos)
Vbetas = np.zeros(npontos)

Var = np.zeros(npontos)
Vbr = np.zeros(npontos)
Vcr = np.zeros(npontos)
Valphar = np.zeros(npontos)
Vbetar = np.zeros(npontos)

Fas = np.zeros(npontos)
Fbs = np.zeros(npontos)
Fcs = np.zeros(npontos)
Falphas = np.zeros(npontos)
Fbetas = np.zeros(npontos)

Far = np.zeros(npontos)
Fbr = np.zeros(npontos)
Fcr = np.zeros(npontos)
Falphar = np.zeros(npontos)
Fbetar = np.zeros(npontos)

Ias = np.zeros(npontos)
Ibs = np.zeros(npontos)
Ics = np.zeros(npontos)
Ialphas = np.zeros(npontos)
Ibetas = np.zeros(npontos)
Ids = np.zeros(npontos)
Iqs = np.zeros(npontos)
Izs = np.zeros(npontos)

Iar = np.zeros(npontos)
Ibr = np.zeros(npontos)
Icr = np.zeros(npontos)
Ialphar = np.zeros(npontos)
Ibetar = np.zeros(npontos)

We = np.zeros(npontos)
Wr = np.zeros(npontos)
Wm = np.zeros(npontos)
Te = np.zeros(npontos)
Tl = np.zeros(npontos)
Oe = np.zeros(npontos)
Om = np.zeros(npontos)
Om_ref = np.zeros(npontos)

T = np.zeros(npontos)
Sa = np.zeros(npontos)
Sb = np.zeros(npontos)
Sc = np.zeros(npontos)
Vrms = np.zeros(npontos)
Wb = np.zeros(npontos)
Wref = np.zeros(npontos)

Ialphas_ref = np.zeros(npontos)
Ibetas_ref = np.zeros(npontos)

# MAIS ALGUMAS VARIAVEIS ALTERADAS COM MAIS FREQUÊNCIA
wrefr = 90  # 1500*pi/30
wref = 0

for j in range(1, npontos):

    t = j*dt
    wref = wrefr*t/(6)
    # if t>=3.0:
    #     wref=0.333*wrefr
    # elif t>=2.8:
    #     wref=0
    # elif t>=1.6:
    #     wref=2*wrefr
    # elif t>=1.4:
    #     wref=0
    # elif t>=1.0:
    #     wref=wrefr
    # elif t>=0.9:
    #     wref=0
    # elif t>=0.6:
    #     wref=wrefr-wrefr*(t-0.6)/(0.3)
    # el
    if (t >= 6):
        wref = wrefr
    Wref[j] = wref

j = 0
tl = 0
sa = 0
sb = 0
sc = 0

vdc = 160  # 220*sqrt(2)*sqrt(3)
flag = 5
Jmin = 1e20
indice_min = (0,0,0)
flag2 = 0
n = 1
tl_est=0
ialphar_est = 0
ibetar_est = 0 
eas_a = 0
ebs_a = 0
# ki=0.0001
# kp=0.005
ki=0.05
kp=0.4
falphas_c0 = 0
fbetas_c0  = 0
falpha_ref = 0.75 
ea = 0

# Kp_vel=0.0436
# Ki_vel=0.19
Kp_vel=0.05
Ki_vel=0.4
Flux_r_ref=0.5
Te_max=8
Int_er_vel = 0
tetar = 0
ias_ref = 0
ibs_ref = 0
print("inicialização concluída\n")
##############################################################################
#####################################
# SEGUNDA PARTE DO CÓGIGO:
# SIMULAÇÃO
#####################################
for t_int in range(npontos):
    t = t_int*dt
    ##########################################################
    ############################
    if t >= 0.7*tmax:
        tl = 0.5 # 0.5
    ##
    # ALIMENTAÇÃO DO SISTEMA
    if flag >= razao_ts_dt:  # TESTE DO TEMPO DE CONTROLE
        Jmin = 10000000000000000000000
        wm_medido = wm + 2*(-0.5+random.random())
        ialphas_medido = ialphas + 0.2*(random.random()-0.5)
        ibetas_medido  = ibetas  + 0.2*(random.random()-0.5)
        ialphar_medido = ialphar_est #+ 0.05*(random.random()-0.5)
        ibetar_medido  = ibetar_est  #+ 0.05*(random.random()-0.5)
        # tl_est=tl_est+1*ts*(Wref[i]-wm_medido)

        er_vel = Wref[i]-wm_medido
        Int_er_vel=Int_er_vel + ts*er_vel
        Te_ref=Kp_vel*er_vel + Ki_vel*Int_er_vel
        abs_Te_ref = abs(Te_ref)
        if (abs_Te_ref > Te_max):
            Te_ref=Te_max*Te_ref/abs_Te_ref
            Int_er_vel=Int_er_vel - ts*er_vel
        Isd_ref = (1.0/Lm)*Flux_r_ref
        Isq_ref = Te_ref/((3.0/2.0)*(P/2.0)*(Lm/Lr)*Flux_r_ref)
        tetar = tetar + ts*(wm_medido + (Isq_ref/Isd_ref)*(Rr/Lr))
        while (tetar > pi):
            tetar = tetar-2*pi
        while (tetar < -pi):
            tetar = tetar+2*pi

        ias_ref = Isd_ref*cos(tetar) - Isq_ref*sin(tetar)
        ibs_ref = Isd_ref*sin(tetar) + Isq_ref*cos(tetar)

        falphas_c0 = Ls*ialphas_medido + Lm*ialphar_medido
        fbetas_c0  = Ls*ibetas_medido  + Lm*ibetar_medido
        falphar_c0 = Lr*ialphar_medido + Lm*ialphas_medido
        fbetar_c0  = Lr*ibetar_medido  + Lm*ibetas_medido
        falphar_c1 = falphar_c0 - Rr*ts*ialphar_medido - ts*wm_medido*fbetar_c0
        fbetar_c1  = fbetar_c0  - Rr*ts*ibetar_medido  + ts*wm_medido*falphar_c0

        Dict = {1: (sa, sb, sc), 2: (abs(sa-1), sb, sc), 3: (sa, abs(sb-1), sc), 4: (sa, sb, abs(sc-1))}
        for p in Dict:
            sa_c1,sb_c1,sc_c1 = Dict[p]
            valphas_c = 1/3*(-sc_c1-sb_c1+2*sa_c1)*vdc
            vbetas_c  = (sb_c1-sc_c1)*vdc/sqrt(3)
                
            falphas_c1 = falphas_c0 - Rs*ts*ialphas_medido + ts*valphas_c
            fbetas_c1  = fbetas_c0  - Rs*ts*ibetas_medido  + ts*vbetas_c
            ialphas_c1 = (Lr*falphas_c1- Lm*falphar_c1)/sig
            ibetas_c1  = (Lr*fbetas_c1 - Lm*fbetar_c1 )/sig
            
            Jc = (ias_ref-ialphas_c1)**2 + (ibs_ref-ibetas_c1)**2
            if Jc <= Jmin:
                Jmin = Jc
                indice_min = sa_c1, sb_c1, sc_c1
                ialphar_est = (Ls*falphar_c1- Lm*falphas_c1)/sig
                ibetar_est  = (Ls*fbetar_c1 - Lm*fbetas_c1 )/sig

        
        # ENTRADA DA ALIMENTAÇÃO DO SISTEMA, QUE VEM DO CONTROLE
        sa, sb, sc = indice_min
        vas,  vbs,  vcs = inversor_ideal(sa, sb, sc, vdc)
        # vas,  vbs,  vcs = 311*cos(377*t),311*cos(377*t-2*pi/3),311*cos(377*t+2*pi/3)
        valphas = 2/3*(vas-1/2*vbs-1/2*vcs)
        vbetas = 2/3*(sqrt(3)/2*vbs-sqrt(3)/2*vcs)
        valphar = 0
        vbetar = 0

        flag = 0
        if flag2 >= npontos*0.05*n/razao_ts_dt:
            print(n*5, "% concluído\n")
            n += 1
        flag2 += 1
    
    falphas = falphas -Rs*dt*ialphas + dt*valphas
    fbetas  = fbetas  -Rs*dt*ibetas  + dt*vbetas
    temp = falphar
    falphar = falphar -Rr*dt*ialphar - dt*wm*fbetar
    fbetar  = fbetar  -Rr*dt*ibetar  + dt*wm*temp

    ialphas = (Lr*falphas- Lm*falphar)/sig
    ibetas  = (Lr*fbetas - Lm*fbetar )/sig
    ialphar = (Ls*falphar- Lm*falphas)/sig
    ibetar  = (Ls*fbetar - Lm*fbetas )/sig

    
    wm = (1-dt*bm/jm)*wm + (dt/jm)*(te-tl)
    
    te = 3/2*P*Lm*(ialphar*ibetas-ibetar*ialphas)
    
    fas,fbs,fcs = inv_ab_transf(falphas, fbetas)
    far,fbr,fcr = inv_ab_transf(falphar, fbetar)

    ias, ibs, ics = inv_ab_transf(ialphas, ibetas)
    iar, ibr, icr = inv_ab_transf(ialphar, ibetar)
    # FIM DA SIMULAÇÃO
    ##############################################
    # AS EXPRESSÕES A SEGUIR APENAS TRATAM AS VARIAVEIS PARA POSTERIOR LEITURA
    
    # fas = falphas
    # fbs = -1/2*falphas+sqrt(3)/2*fbetas
    # fcs = -1/2*falphas-sqrt(3)/2*fbetas

    # far = falphar*sin(-oe)+fbetar*cos(-oe)
    # fbr = falphar*sin(-oe-2*pi/3)+fbetar*cos(-oe-2*pi/3)
    # fcr = falphar*sin(-oe-4*pi/3)+fbetar*cos(-oe-4*pi/3)

    om = om+dt*wm
    if om > pi:
        om = om-2*pi
    if om < -pi:
        om = om+2*pi
        
    # if(wm>10000):
    #     break
    ##########################################################################
    # LEITURA DAS VARIÁVEIS
    Ialphas_ref[i] = ias_ref
    Ibetas_ref [i]  = ibs_ref
    Sa[i] = sa
    Sb[i] = sb
    Sc[i] = sc

    Vas[i] = vas
    Vbs[i] = vbs
    Vcs[i] = vcs
    Valphas[i] = valphas
    Vbetas[i] = vbetas

    Var[i] = var
    Vbr[i] = vbr
    Vcr[i] = vcr
    Valphar[i] = valphar
    Vbetar[i] = vbetar

    Fas[i] = fas
    Fbs[i] = fbs
    Fcs[i] = fcs
    Falphas[i] = falphas
    Fbetas[i] = fbetas

    Far[i] = far
    Fbr[i] = fbr
    Fcr[i] = fcr
    Falphar[i] = falphar
    Fbetar[i] = fbetar

    Ias[i] = ias
    Ibs[i] = ibs
    Ics[i] = ics
    Ialphas[i] = ialphas
    Ibetas[i] = ibetas
    Ids[i] = ids
    Iqs[i] = iqs
    Izs[i] = izs

    Iar[i] = iar
    Ibr[i] = ibr
    Icr[i] = icr
    Ialphar[i] = ialphar
    Ibetar[i] = ibetar

    We[i] = we
    Wr[i] = wr
    Wm[i] = wm
    Te[i] = te
    Tl[i] = tl
    Om[i] = om
    Om_ref[i] = tetar

    T[i] = t

    i = i+1
    flag += 1

print('demorou ', time.time()-tempo_inicial, ' s\n\n\n')
#####################################
# TERCEIRA PARTE DO CÓGIGO:
# PLOT E/OU SALVAR OS DADOS
#####################################
# np.savetxt('input4.dat',inputs)
# np.savetxt('output4.dat',output)


plt.rc('text', usetex=False)
# plt.rc('font', family='serif')
plt.close('all')

plt.figure(1)
plt.plot(T, Wm, color='C0', label=r"$\omega_m$")
plt.plot(T, Wref, color='k', label=r"$\omega_m^{*}$")
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$\omega_m (t)$ [rad/s]")
plt.title(r"Velocidade Angular")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_velocidade.pdf", format='pdf')

plt.figure(2)
plt.plot(T, Ias, color='C0', label=r'$i_{as}$')
plt.plot(T, Ibs, color='C1', label=r'$i_{bs}$')
plt.plot(T, Ics, color='C2', label=r'$i_{cs}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i(t)$[A]")
plt.title(r"Corrente Estator")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_corrente.pdf", format='pdf')
plt.figure(3)
plt.plot(T, Ialphas    , color='C0', label=r'$i_{\alpha s}$')
plt.plot(T, Ialphas_ref, color='k',linestyle='--', label=r'$i_{\alpha s}^{*}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i(t)$[A]")
plt.title(r"Correntes Estator")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_alphascorrente.pdf", format='pdf')
plt.figure(4)
plt.plot(T, Ibetas     , color='C0', label=r'$i_{\beta s}$')
plt.plot(T, Ibetas_ref , color='k' ,linestyle='--', label=r'$i_{\beta s}^{*}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i(t)$[A]")
plt.title(r"Correntes Estator")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_betascorrente.pdf", format='pdf')
plt.figure(5)
plt.plot(T, Ialphar, color='C0', label=r'$i_{\alpha r}$')
plt.plot(T, Ibetar,  color='C1', label=r'$i_{\beta r}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i(t)$[A]")
plt.title(r"Correntes rotor")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_alpharcorrente.pdf", format='pdf')
plt.figure(6)
plt.plot(T, Om, color='C0', label=r'$\theta_{m}$')
plt.plot(T, Om_ref,  color='k',linestyle='--', label=r'$\theta_{m}^{*}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$\theta_{m} (t)$[rad]")
plt.title(r"Posição Angular")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_posicao.pdf", format='pdf')

plt.figure(7)
plt.plot(T, Vas, color='C0', label=r'$v_{as}$')
plt.plot(T, Vbs, color='C1', label=r'$v_{bs}$')
plt.plot(T, Vcs, color='C2', label=r'$v_{cs}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$v(t)$[V]")
plt.title(r"Ação de Controle")
plt.grid()
plt.legend()
# plt.savefig("sim_mit_tensao.pdf", format='pdf')

plt.show()