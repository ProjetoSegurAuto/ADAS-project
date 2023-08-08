import pickle

try:
    arquivo = open("log_platoon_action.bin", "wb")
    lista = [1, 2, 3]
    pickle.dump(lista, arquivo)
    arquivo.close()
except:
    print("Problemas com o arquivo.")