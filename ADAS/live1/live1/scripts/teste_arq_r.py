import pickle
try:
    arquivo = open("log_platoon_action.bin", "rb")
    l = pickle.load(arquivo)
    print(l)
    arquivo.close()
except:
    print("Problemas com o arquivo.")