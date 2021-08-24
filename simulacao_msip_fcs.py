import numpy as np
from numpy import random
import time
from math import sqrt
from math import pi
from math import sin
from math import cos
import matplotlib.pyplot as plt
# from mpmath import *
# mp.dps = 25; mp.pretty = True

np.random.seed(1)
print("começou\n")
tempo_inicial = time.time()
#####################################
# PRIMEIRA PARTE DO CÓGIGO:
# INICIALIZAÇÃO
#####################################
# PARAMETROS DA SIMULAÇÃO
dt = 1e-5
ts = 1e-4
razao_ts_dt = round(ts/dt)
tmax = 20-dt
npontos = round(tmax/dt)
i = 0

# # #PARAMETROS arthur
# Rs = 20.6
# Lqs = 0.82425
# Lds = 0.82425
# Lam = 0.78
# jm = 0.012
# bm = 0.00881
# vrms = 192

Rs = 4.5
Lqs = 0.05
Lds = 0.05
Lam = 0.19
jm = 0.00115
bm = 0.0575
vrms = 128

# float fpm = 0.19, ParPolos = 21, np = 21, lqs = 0.05, lds = 0.05, jm = 0.00115, bm = 0.0575, rs = 4.5, las = 0.0333;
P = 42 # POLOS OU PAR DE POLOS?
Pd2 = P/2

# VARIÁVEIS
vas = 0
vbs = 0
vcs = 0
valphas = 0
vbetas = 0

vds = 0
vqs = 0

fas = 0
fbs = 0
fcs = 0
falphas = 0
fbetas = 0

ias = 0
ibs = 0
ics = 0
ialphas = 0
ibetas = 0
ids = 0
iqs = 0
izs = 0

we = 0
wr = 0
wm = 0
te = 0
tl = 0
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

Vds = np.zeros(npontos)
Vqs = np.zeros(npontos)

Fas = np.zeros(npontos)
Fbs = np.zeros(npontos)
Fcs = np.zeros(npontos)
Falphas = np.zeros(npontos)
Fbetas = np.zeros(npontos)

Ias = np.zeros(npontos)
Ibs = np.zeros(npontos)
Ics = np.zeros(npontos)
Ialphas = np.zeros(npontos)
Ibetas = np.zeros(npontos)
Ids = np.zeros(npontos)
Iqs = np.zeros(npontos)
Izs = np.zeros(npontos)

We = np.zeros(npontos)
Wr = np.zeros(npontos)
Wm = np.zeros(npontos)
Te = np.zeros(npontos)
Tl = np.zeros(npontos)
Tlest = np.zeros(npontos)
Oe = np.zeros(npontos)
Om = np.zeros(npontos)

T = np.zeros(npontos)
Sa = np.zeros(npontos)
Sb = np.zeros(npontos)
Sc = np.zeros(npontos)
Vrms = np.zeros(npontos)
Wb = np.zeros(npontos)
Wref = np.zeros(npontos)

# MAIS ALGUMAS VARIAVEIS ALTERADAS COM MAIS FREQUÊNCIA
wrefr = 50
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

vdc = 128  # 220*sqrt(2)*sqrt(3)
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
ki=0.001
kp=0.1
falphas_c0 = 0
fbetas_c0  = 0
falpha_ref = 0.75 
ea = 0

Kp_vel=0.0436
Ki_vel=0.19
# Kp_vel=0.2
# Ki_vel=0.9
Flux_r_ref=0.5
Te_max=1.4
Int_er_vel = 0
tetar = 0
kr_we_we = 0.030084267610605207
kx_we_we = 0.029934221699924397
kz_we_we = 0.00018446038333317394
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
    if t >= 15:
        tl =  0.1
    ##
    # ALIMENTAÇÃO DO SISTEMA
    if flag >= razao_ts_dt:  # TESTE DO TEMPO DE CONTROLE
        Jmin = 10000000000000000000000
        wm_medido = wm + 2*(-0.5+random.random())
        iqs_medido = iqs + 0.02*(random.random()-0.5)
        ids_medido  = ids + 0.02*(random.random()-0.5)
        e=(Wref[i]-wm_medido)
        tl_est=tl_est+2.5*ts*e
        te_c0 = 3/2*((Lds-Lqs)*ids_medido+Lam)*iqs_medido
        wm_c0 = (1-ts*bm/jm)*wm_medido + Pd2**2*(ts/jm)*(te_c0-tl_est)
        tl_est2=tl_est+2.5*ts*e
        Dict = {1: (sa, sb, sc), 2: (abs(sa-1), sb, sc), 3: (sa, abs(sb-1), sc), 4: (sa, sb, abs(sc-1))}
        for p in Dict:
            sa_c1,sb_c1,sc_c1 = Dict[p]
            vas_c = (2.0/3.0*sa_c1-1.0/3.0*sb_c1-1.0/3.0*sc_c1)*vdc
            vbs_c = (2.0/3.0*sb_c1-1.0/3.0*sa_c1-1.0/3.0*sc_c1)*vdc
            vcs_c = (2.0/3.0*sc_c1-1.0/3.0*sb_c1-1.0/3.0*sa_c1)*vdc
            vds_c, vqs_c = dq_transf(vas_c, vbs_c, vcs_c, oe)
            iqs_c = (1-Rs/Lqs*ts)*iqs_medido - Lds/Lqs*ts*wm_medido*ids_medido - Lam/Lqs*ts*wm_medido + ts/Lqs*vqs_c
            ids_c = (1-Rs/Lds*ts)*ids_medido + Lqs/Lds*ts*wm_medido*iqs_medido + ts/Lds*vds_c
            te_c = 3/2*((Lds-Lqs)*ids_c+Lam)*iqs_c
            wm_c = (1-ts*bm/jm)*wm_c0 + Pd2**2*(ts/jm)*(te_c-tl_est2)

            Jc = (Wref[i+2]-wm_c)**2 +(0.00000001)*abs(iqs_c) +(0.0000000001)*abs(ids_c)
            if Jc <= Jmin:
                Jmin = Jc
                indice_min = sa_c1, sb_c1, sc_c1


        
        # ENTRADA DA ALIMENTAÇÃO DO SISTEMA, QUE VEM DO CONTROLE
        sa, sb, sc = indice_min
        vas,  vbs,  vcs = inversor_ideal(sa, sb, sc, vdc)
        # wtemp=Wref[t_int]/10
        # vas,  vbs,  vcs = 311*cos(wtemp*t),311*cos(wtemp*t-2*pi/3),311*cos(wtemp*t+2*pi/3)
        vds,vqs=dq_transf(vas, vbs, vcs, oe)
        valphas = 2/3*(vas-1/2*vbs-1/2*vcs)
        vbetas = 2/3*(sqrt(3)/2*vbs-sqrt(3)/2*vcs)
        valphar = 0
        vbetar = 0

        flag = 0
        if flag2 >= npontos*0.05*n/razao_ts_dt:
            print(n*5, "% concluído\n")
            n += 1
        flag2 += 1
    
    temp = iqs
    iqs = (1-Rs/Lqs*dt)*iqs - Lds/Lqs*dt*wm*ids - Lam/Lqs*dt*wm + dt/Lqs*vqs
    ids = (1-Rs/Lds*dt)*ids + Lqs/Lds*dt*wm*temp + dt/Lds*vds
    wm = (1-dt*bm/jm)*wm + Pd2**2*(dt/jm)*(te-tl)
    te = 3/2*((Lds-Lqs)*ids+Lam)*iqs
   
    wr = (P/2)*wm  # WR É USADO PARA O CÁLCULO DO FLUXO NO ROTOR
    oe = oe+dt*wr
    if oe > (P/2)*2*pi:
        oe = oe-(P/2)*2*pi
    if oe < 0:
        oe = oe+(P/2)*2*pi     

    fas,fbs,fcs = inv_ab_transf(falphas, fbetas)
    ias, ibs, ics = invdq_transf(iqs,ids,oe)

   # FIM DA SIMULAÇÃO
    ##############################################
    # AS EXPRESSÕES A SEGUIR APENAS TRATAM AS VARIAVEIS PARA POSTERIOR LEITURA
    ##########################################################################
    # LEITURA DAS VARIÁVEIS
    Sa[i] = sa
    Sb[i] = sb
    Sc[i] = sc

    Vas[i] = vas
    Vbs[i] = vbs
    Vcs[i] = vcs
    Valphas[i] = valphas
    Vbetas[i] = vbetas


    Fas[i] = fas
    Fbs[i] = fbs
    Fcs[i] = fcs
    Falphas[i] = falphas
    Fbetas[i] = fbetas


    Ias[i] = ias
    Ibs[i] = ibs
    Ics[i] = ics
    Ialphas[i] = ialphas
    Ibetas[i] = ibetas
    Ids[i] = ids
    Iqs[i] = iqs
    Izs[i] = izs


    We[i] = we
    Wr[i] = wr
    Wm[i] = wm
    Te[i] = te
    Tl[i] = tl
    Tlest[i] = tl_est
    Oe[i] = oe

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
plt.plot(T, Vas, color='C0', label=r'$v_{as}$')
plt.plot(T, Vbs, color='C1', label=r'$v_{bs}$')
plt.plot(T, Vcs, color='C2', label=r'$v_{cs}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$V[V]$")
plt.title(r"Tensões no estator")
plt.grid()
plt.legend()
plt.savefig("sim_msip_tensao.pdf", format='pdf')

plt.figure(2)
plt.plot(T, Ids, color='C0', label=r'$i_{ds}$')
plt.plot(T, Iqs, color='C1', label=r'$i_{qs}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i[A]$")
plt.title(r"Correntes Estator")
plt.grid()
plt.legend()
plt.savefig("sim_msip_dqcorrente.pdf", format='pdf')

plt.figure(3)
plt.plot(T, Ias, color='C0', label=r'$i_{as}$')
plt.plot(T, Ibs, color='C1', label=r'$i_{bs}$')
plt.plot(T, Ics, color='C2', label=r'$i_{cs}$')
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$i[A]$")
plt.title(r"Correntes Estator")
plt.grid()
plt.legend()
plt.savefig("sim_msip_corrente.pdf", format='pdf')
# plt.figure(4)
# plt.plot(Wm, Te, color='C0')
# plt.xlabel(r"$Speed$[rpm]")
# plt.ylabel(r"Torque[Nm]")
# plt.title(r"Angular Speed")
# plt.grid()
# plt.legend()

plt.figure(5)
plt.plot(T, Te, color='C0', label=r"$T_e (t)$")
plt.plot(T, Tlest, color='C1', label=r"$T_{l,est} (t)$")
plt.plot(T, Tl, color='C2', label=r"$T_{l} (t)$")
plt.xlabel(r"$t$[s]")
plt.ylabel(r"Torque[Nm]")
plt.title(r"Torque")
plt.grid()
plt.legend()
plt.savefig("sim_msip_torque.pdf", format='pdf')

plt.figure(6)
plt.plot(T, Wm, color='C0', label=r"$\omega_m$")
plt.plot(T, Wref, color='k', label=r"$\omega_m^{*}$")
plt.xlabel(r"$t$[s]")
plt.ylabel(r"$\omega_m (t)$ [rad/s]")
plt.title(r"Velocidade Angular")
plt.grid()
plt.legend()
plt.savefig("sim_msip_velocidade.pdf", format='pdf')

plt.show()