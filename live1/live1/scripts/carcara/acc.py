import numpy as np
import skfuzzy as fuzz

class controllerFuzzy():

    def __init__(self):
        self.erro  = np.arange(-100, 100, 1)
        self.derivada_erro = np.arange(-100, 100, 1)
        self.deltaVel  = np.arange(-100, 100, 1)

        # Generate fuzzy membership functions
        self.erro_NG = fuzz.trapmf(self.erro, [-100, -100, -80, -40])
        self.erro_NM = fuzz.trimf(self.erro, [-60, -40, -20])
        self.erro_NP = fuzz.trimf(self.erro, [-40, -20, 0])
        self.erro_Z = fuzz.trimf(self.erro, [-5, 0, 5])
        self.erro_PP = fuzz.trimf(self.erro, [0, 20, 40])
        self.erro_PM = fuzz.trimf(self.erro, [20, 40, 60])
        self.erro_PG = fuzz.trapmf(self.erro, [40, 80, 100, 100])

        self.derivada_erro_NG = fuzz.trapmf(self.derivada_erro, [-100, -100, -80, -40])
        self.derivada_erro_NM = fuzz.trimf(self.derivada_erro, [-60, -40, -20])
        self.derivada_erro_NP = fuzz.trimf(self.derivada_erro, [-40, -20, 0])
        self.derivada_erro_Z = fuzz.trimf(self.derivada_erro, [-5, 0, 5])
        self.derivada_erro_PP = fuzz.trimf(self.derivada_erro, [0, 20, 40])
        self.derivada_erro_PM = fuzz.trimf(self.derivada_erro, [20, 40, 60])
        self.derivada_erro_PG = fuzz.trapmf(self.derivada_erro, [40, 80, 100, 100])

        self.deltaVel_NG = fuzz.trapmf(self.deltaVel, [-100, -100, -80, -40])
        self.deltaVel_NM = fuzz.trimf(self.deltaVel, [-60, -40, -20])
        self.deltaVel_NP = fuzz.trimf(self.deltaVel, [-40, -20, 0])
        self.deltaVel_Z = fuzz.trimf(self.deltaVel, [-5, 0, 5])
        self.deltaVel_PP = fuzz.trimf(self.deltaVel, [0, 20, 40])
        self.deltaVel_PM = fuzz.trimf(self.deltaVel, [20, 40, 60])
        self.deltaVel_PG = fuzz.trapmf(self.deltaVel, [40, 80, 100, 100])

    def fuzzyficacao(self, erro, dErro):

        #Erro
        erro_levelNG = fuzz.interp_membership(self.erro, self.erro_NG, erro)
        erro_levelNM = fuzz.interp_membership(self.erro, self.erro_NM, erro)
        erro_levelNP = fuzz.interp_membership(self.erro, self.erro_NP, erro)
        erro_levelZ  = fuzz.interp_membership(self.erro, self.erro_Z,  erro)
        erro_levelPP = fuzz.interp_membership(self.erro, self.erro_PP, erro)
        erro_levelPM = fuzz.interp_membership(self.erro, self.erro_PM, erro)
        erro_levelPG = fuzz.interp_membership(self.erro, self.erro_PG, erro)

        derivada_erro_levelNG = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_NG, dErro)
        derivada_erro_levelNM = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_NM, dErro)
        derivada_erro_levelNP = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_NP, dErro)
        derivada_erro_levelZ  = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_Z , dErro)
        derivada_erro_levelPP = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_PP, dErro)
        derivada_erro_levelPM = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_PM, dErro)
        derivada_erro_levelPG = fuzz.interp_membership(self.derivada_erro, self.derivada_erro_PG, dErro)

        return [erro_levelNG, erro_levelNM,  erro_levelNP, erro_levelZ, erro_levelPP, erro_levelPM, erro_levelPG, derivada_erro_levelNG, derivada_erro_levelNM, derivada_erro_levelNP, derivada_erro_levelZ, derivada_erro_levelPP, derivada_erro_levelPM, derivada_erro_levelPG]


    def inferencia(self, level):

        #Regras
        activeted_1 = np.fmin(np.fmin(level[0], level[7]), self.deltaVel_NG)
        activeted_2 = np.fmin(np.fmin(level[0], level[8]), self.deltaVel_NG)
        activeted_3 = np.fmin(np.fmin(level[0], level[9]), self.deltaVel_NG)
        activeted_4 = np.fmin(np.fmin(level[0], level[10]), self.deltaVel_NG)
        activeted_5 = np.fmin(np.fmin(level[0], level[11]), self.deltaVel_NG)
        activeted_6 = np.fmin(np.fmin(level[0], level[12]), self.deltaVel_NG)
        activeted_7 = np.fmin(np.fmin(level[0], level[13]), self.deltaVel_NG)

        activeted_8 = np.fmin(np.fmin(level[1], level[7]), self.deltaVel_NG)
        activeted_9 = np.fmin(np.fmin(level[1], level[8]), self.deltaVel_NM)
        activeted_10 = np.fmin(np.fmin(level[1], level[9]), self.deltaVel_NM)
        activeted_11 = np.fmin(np.fmin(level[1], level[10]), self.deltaVel_NM)
        activeted_12 = np.fmin(np.fmin(level[1], level[11]), self.deltaVel_NM)
        activeted_13 = np.fmin(np.fmin(level[1], level[12]), self.deltaVel_NM)
        activeted_14 = np.fmin(np.fmin(level[1], level[13]), self.deltaVel_NG)

        activeted_15 = np.fmin(np.fmin(level[2], level[7]), self.deltaVel_NG)
        activeted_16 = np.fmin(np.fmin(level[2], level[8]), self.deltaVel_NM)
        activeted_17 = np.fmin(np.fmin(level[2], level[9]), self.deltaVel_NP)
        activeted_18 = np.fmin(np.fmin(level[2], level[10]), self.deltaVel_NP)
        activeted_19 = np.fmin(np.fmin(level[2], level[11]), self.deltaVel_NP)
        activeted_20 = np.fmin(np.fmin(level[2], level[12]), self.deltaVel_NM)
        activeted_21 = np.fmin(np.fmin(level[2], level[13]), self.deltaVel_NG)

        activeted_22 = np.fmin(np.fmin(level[3], level[7]), self.deltaVel_NG)
        activeted_23 = np.fmin(np.fmin(level[3], level[8]), self.deltaVel_NM)
        activeted_24 = np.fmin(np.fmin(level[3], level[9]), self.deltaVel_NP)
        activeted_25 = np.fmin(np.fmin(level[3], level[10]), self.deltaVel_Z)
        activeted_26 = np.fmin(np.fmin(level[3], level[11]), self.deltaVel_PP)
        activeted_27 = np.fmin(np.fmin(level[3], level[12]), self.deltaVel_PM)
        activeted_28 = np.fmin(np.fmin(level[3], level[13]), self.deltaVel_PG)

        activeted_29 = np.fmin(np.fmin(level[4], level[7]), self.deltaVel_PG)
        activeted_30 = np.fmin(np.fmin(level[4], level[8]), self.deltaVel_PM)
        activeted_31 = np.fmin(np.fmin(level[4], level[9]), self.deltaVel_PP)
        activeted_32 = np.fmin(np.fmin(level[4], level[10]), self.deltaVel_PP)
        activeted_33 = np.fmin(np.fmin(level[4], level[11]), self.deltaVel_PP)
        activeted_34 = np.fmin(np.fmin(level[4], level[12]), self.deltaVel_PM)
        activeted_35 = np.fmin(np.fmin(level[4], level[13]), self.deltaVel_PG)

        activeted_36 = np.fmin(np.fmin(level[5], level[7]), self.deltaVel_PG)
        activeted_37 = np.fmin(np.fmin(level[5], level[8]), self.deltaVel_PM)
        activeted_38 = np.fmin(np.fmin(level[5], level[9]), self.deltaVel_PM)
        activeted_39 = np.fmin(np.fmin(level[5], level[10]), self.deltaVel_PM)
        activeted_40 = np.fmin(np.fmin(level[5], level[11]), self.deltaVel_PM)
        activeted_41 = np.fmin(np.fmin(level[5], level[12]), self.deltaVel_PM)
        activeted_42 = np.fmin(np.fmin(level[5], level[13]), self.deltaVel_PG)

        activeted_43 = np.fmin(np.fmin(level[6], level[7]), self.deltaVel_PG)
        activeted_44 = np.fmin(np.fmin(level[6], level[8]), self.deltaVel_PG)
        activeted_45 = np.fmin(np.fmin(level[6], level[9]), self.deltaVel_PG)
        activeted_46 = np.fmin(np.fmin(level[6], level[10]), self.deltaVel_PG)
        activeted_47 = np.fmin(np.fmin(level[6], level[11]), self.deltaVel_PG)
        activeted_48 = np.fmin(np.fmin(level[6], level[12]), self.deltaVel_PG)
        activeted_49 = np.fmin(np.fmin(level[6], level[13]), self.deltaVel_PG)

        aggregated1 = np.fmax(activeted_1, np.fmax(activeted_2, np.fmax(activeted_3, np.fmax(activeted_4, np.fmax(activeted_5, np.fmax(activeted_6, activeted_7))))))
        aggregated2 = np.fmax(activeted_8, np.fmax(activeted_9, np.fmax(activeted_10, np.fmax(activeted_11, np.fmax(activeted_12, np.fmax(activeted_13, activeted_14))))))
        aggregated3 = np.fmax(activeted_15, np.fmax(activeted_16, np.fmax(activeted_17, np.fmax(activeted_18, np.fmax(activeted_19, np.fmax(activeted_20, activeted_21))))))
        aggregated4 = np.fmax(activeted_22, np.fmax(activeted_23, np.fmax(activeted_24, np.fmax(activeted_25, np.fmax(activeted_26, np.fmax(activeted_27, activeted_28))))))
        aggregated5 = np.fmax(activeted_29, np.fmax(activeted_30, np.fmax(activeted_31, np.fmax(activeted_32, np.fmax(activeted_33, np.fmax(activeted_34, activeted_35))))))
        aggregated6 = np.fmax(activeted_36, np.fmax(activeted_37, np.fmax(activeted_38, np.fmax(activeted_39, np.fmax(activeted_40, np.fmax(activeted_41, activeted_42))))))
        aggregated7 = np.fmax(activeted_43, np.fmax(activeted_44, np.fmax(activeted_45, np.fmax(activeted_46, np.fmax(activeted_47, np.fmax(activeted_48, activeted_49))))))

        aggregated = np.fmax(aggregated1, np.fmax(aggregated2, np.fmax(aggregated3, np.fmax(aggregated4, np.fmax(aggregated5, np.fmax(aggregated6, aggregated7))))))

        return aggregated

    def defuzzyficacao(self, aggregated):

        saida = fuzz.defuzz(self.deltaVel, aggregated, 'centroid')

        return saida

    def controller(self, erro, dErro):

        if erro < self.erro[0]:
            erro = self.erro[0]
        elif erro > self.erro[-1]:
            erro = self.erro[-1]

        if dErro < self.derivada_erro[0]:
            dErro = self.derivada_erro[0]
        elif dErro > self.derivada_erro[-1]:
            dErro = self.derivada_erro[-1]

        erro = int(erro)
        dErro = int(dErro)
        
        saida_ms = self.defuzzyficacao(self.inferencia(self.fuzzyficacao(erro, dErro)))/100
        print(saida_ms)
        saida_rpm = (saida_ms/0.15) * 2 * 3.14 / 60
        
        return saida_rpm