

import serial
import PySimpleGUI as sg

from time import sleep #Importa a biblioteca para criar delays no código

import serial as srl #Importa a biblioteca pyserial
import serial.tools.list_ports #comando para listar seriais

from threading import Thread

from datetime import datetime

import time

from numpy import mean


#thread q le os dados vindos da serial
def verificaSerial(portaCOM, layout, tempoECUsCarro1, tempoECUsCarro2, stop_thread):

	import time #evita erro qndo se usa o time.time()

	idCarLista = [] #lista com os MAC dos carros encontrados (escuta o MAC de quem envia via broadcast)

	tempoListaMACs = 0

	GRAPH_STEP_SIZE = 10 #espaço entre as amostas

	fatorAjusteDirecao = 3.5 #divisor para melhor visualização no grafico
	fatorAjusteRPM = 1.4 #divisor para melhor visualização no grafico
	numAmostras = 16 #numero de amostras para fazer a media das leituras

	graphRPMEsqCarro1 = [0,0]
	graphsPRPMEsqCarro1 = [0,0]
	xRPMEsqCarro1 = 0
	lastxRPMEsqCarro1 = 0

	medGraphRPMEsqCarro1 = [0] * numAmostras
	maxGraphRPMEsqCarro1 = 0 

	textoGraphRPMEsqCarro1 = 0 # usado para mostrar os valores no grafico de RPM
	textoGraphsPRPMEsqCarro1 = 0 # usado para mostrar os valores no grafico de RPM

	textomedGraphRPMEsqCarro1 = 0
	textomaxGraphRPMEsqCarro1 = 0 

	contLinhaVerticalRPMEsqCarro1 = 0


	graphRPMDirCarro1 = [0,0]
	graphsPRPMDirCarro1 = [0,0]
	xRPMDirCarro1 = 0
	lastxRPMDirCarro1 = 0

	medGraphRPMDirCarro1 = [0] * numAmostras
	maxGraphRPMDirCarro1 = 0

	textoGraphRPMDirCarro1 = 0 # usado para mostrar os valores no grafico de RPM
	textoGraphsPRPMDirCarro1 = 0 # usado para mostrar os valores no grafico de RPM

	textomedGraphRPMDirCarro1 = 0
	textomaxGraphRPMDirCarro1 = 0 


	graphDirecCarro1 = [0,0]
	graphsPDirecCarro1 = [0,0]
	xgraphDirecCarro1 = 0
	lastxDirecCarro1 = 0

	medGraphDirecCarro1 = [0] * numAmostras
	maxGraphsPDirecCarro1 = 0

	textomedGraphDirecCarro1 = 0
	textomaxGraphsPDirecCarro1 = 0

	textoGraphDirecCarro1 = 0 # usado para mostrar os valores no grafico de direcao
	textoGraphsPDirecCarro1 = 0 # usado para mostrar os valores no grafico de direcao


	graphDistancecCarro1 = [0,0]
	graphsPDistanceCarro1 = [0,0]
	xgraphDistanceCarro1 = 0
	lastxgraphDistanceCarro1 = 0


	graphRPMEsqCarro2 = [0,0]
	graphsPRPMEsqCarro2 = [0,0]
	xRPMEsqCarro2 = 0
	lastxRPMEsqCarro2 = 0

	medGraphRPMEsqCarro2 = [0] * numAmostras
	maxGraphRPMEsqCarro2 = 0

	textoGraphRPMEsqCarro2 = 0 # usado para mostrar os valores no grafico de RPM
	textoGraphsPRPMEsqCarro2 = 0 # usado para mostrar os valores no grafico de RPM

	textomedGraphRPMEsqCarro2 = 0
	textomaxGraphRPMEsqCarro2 = 0 


	graphRPMDirCarro2 = [0,0]
	graphsPRPMDirCarro2 = [0,0]
	xRPMDirCarro2 = 0
	lastxRPMDirCarro2 = 0

	medGraphRPMDirCarro2 = [0] * numAmostras
	maxGraphRPMDirCarro2 = 0

	textoGraphRPMDirCarro2 = 0 # usado para mostrar os valores no grafico de RPM
	textoGraphsPRPMDirCarro2 = 0 # usado para mostrar os valores no grafico de RPM

	textomedGraphRPMDirCarro2 = 0
	textomaxGraphRPMDirCarro2 = 0 

	
	graphDirecCarro2 = [0,0]
	graphsPDirecCarro2 = [0,0]
	xgraphDirecCarro2 = 0
	lastxDirecCarro2 = 0

	medGraphDirecCarro2 = [0] * numAmostras
	maxGraphsPDirecCarro2 = 0

	textomedGraphDirecCarro2 = 0
	textomaxGraphsPDirecCarro2 = 0

	textoGraphDirecCarro2 = 0 #usado para mostrar os valores no grafico de direcao
	textoGraphsPDirecCarro2 = 0 #usado para mostrar os valores no grafico de direcao


	tempoinicio = time.time()

	while True: #Loop principal	    


	    #msg = str(portaCOM.readline()) #Lê os dados em formato de string    
	    #msg = portaCOM.read(19).hex() # 20 bytes + 0x0d + 0x0a
	    msg = portaCOM.readline().hex() # ate ler 0x0d + 0x0a

	    #print(msg)
	    
	    timeoutECUs = 2 #2 segundos
	    #muda a cor do texto se alguma ecu não enviar msg por um pediodo de tempo
	    
	    if((time.time() - tempoECUsCarro1[0]) > timeoutECUs):
	    	layout['idCarro1label'].Update(text_color = 'red')
	    else:
	    	layout['idCarro1label'].Update(text_color = 'white')
	   	
	    if((time.time() - tempoECUsCarro1[1]) > timeoutECUs):
	    	layout['idCANCarro11'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro11'].Update('0')
	    else:
	    	layout['idCANCarro11'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro11'].Update("{:.3f}".format(time.time() - tempoECUsCarro1[1]))

	    if((time.time() - tempoECUsCarro1[2]) > timeoutECUs):
	    	layout['idCANCarro12'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro12'].Update('0')
	    else:
	    	layout['idCANCarro12'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro12'].Update("{:.3f}".format(time.time() - tempoECUsCarro1[2]))

	    if((time.time() - tempoECUsCarro1[3]) > timeoutECUs):
	    	layout['idCANCarro13'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro13'].Update('0')
	    else:
	    	layout['idCANCarro13'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro13'].Update("{:.3f}".format(time.time() - tempoECUsCarro1[3]))


	    #muda a cor do texto se alguma ecu não enviar msg
	    
	    if((time.time() - tempoECUsCarro2[0]) > timeoutECUs):
	    	layout['idCarro2label'].Update(text_color = 'red')
	    else:
	    	layout['idCarro2label'].Update(text_color = 'white')
		
	    if((time.time() - tempoECUsCarro2[1]) > timeoutECUs):
	    	layout['idCANCarro21'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro21'].Update('0')
	    else:
	    	layout['idCANCarro21'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro21'].Update("{:.3f}".format(time.time() - tempoECUsCarro2[1]))

	    if((time.time() - tempoECUsCarro2[2]) > timeoutECUs):
	    	layout['idCANCarro22'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro22'].Update('0')
	    else:
	    	layout['idCANCarro22'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro22'].Update("{:.3f}".format(time.time() - tempoECUsCarro2[2]))

	    if((time.time() - tempoECUsCarro2[3]) > timeoutECUs):
	    	layout['idCANCarro23'].Update(text_color = 'red')
	    	layout['msgTypeTimeCarro23'].Update('0')
	    else:
	    	layout['idCANCarro23'].Update(text_color = 'white')
	    	layout['msgTypeTimeCarro23'].Update("{:.3f}".format(time.time() - tempoECUsCarro2[3]))
	    


	    if len(msg) > 35:
	        #print(msg) #Imprime a mensagem recebida
	       	msgAux = ""

	       	idCar = msg[0:12]
	       	#idCar = msg[0:2] 
	       	#print(idCar, len(idCar))


	       	idCar = idCar[0:2] + ":" + idCar[2:4] + ":" + idCar[4:6] + ":" + idCar[6:8] + ":" + idCar[8:10] + ":" + idCar[10:12] 
	       	idCar = idCar.upper() #letras maiusculas

	       	#print(idCar)


	       	if not idCar in idCarLista: #se o MAC não está na lista adiciona a lista
	       		idCarLista.append(idCar) #adiciona o MAC a lista
	       		layout['logMAC'].print(idCar)
	       	#print(idCarLista)

	       	#if layout['idCarro1'].get() == str(int('0x'+ idCar, base=16))  and layout['idCarro1'].get() != '':
	       	if layout['idCarro1'].get() == str(idCar)  and layout['idCarro1'].get() != '':
	       		
	       		tempoECUsCarro1[0] = time.time()
	       		#layout['logCar1'].print(str(msg))

	       		for i in range(0,int(len(msg)-4),2): #-4 corta o 0x0d 0x0a
	        		msgAux = str(msgAux) + ' ' + str(msg[i:i+2])


	        	layout['logCarro1'].print(str(msgAux))
	        	"""	
	        	#deixa o fundo de algumas msg coloridas
	        	if int('0x'+ msg[12:14], base=16) == 14 or int('0x'+ msg[12:14], base=16) == 16 or int('0x'+ msg[12:14], base=16) == 18:
	        			layout['logCarro1'].print(msgAux, text_color='red', background_color='blue')

	        	elif int('0x'+ msg[12:14], base=16) == 12:
	        			layout['logCarro1'].print(msgAux, text_color='red', background_color='green')	
	        	
	        	elif int('0x'+ msg[12:14], base=16)%2 == 0 and int('0x'+ msg[12:14], base=16) != 0 :

	        		layout['logCarro1'].print(msgAux, text_color='red', background_color='yellow')	

	        		#layout['logCarro2'].print('resp: ' + msgAux)
	        	else:
	        		layout['logCarro1'].print(msgAux)		
				"""

	        	if layout['idCANCarro11'].get() != "":

		        	#if int(layout['idCANCarro11'].get(), base=16) == int('0x'+ msg[18:20], base=16) and int(layout['msgTypeCarro11'].get()) == int('0x'+ msg[12:14], base=16):
		        	if int('0x'+ layout['idCANCarro11'].get(), base=16) == int('0x'+ msg[18:20], base=16):
		        		#print(msg[6:8])
		        		layout['pidCarro1'].Update(str(msg[24:26]))
		        		layout['rpmEsqCarro1'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['sPrpmEsqCarro1'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['sPrpmEsqHEXCarro1'].Update(str('0x'+ msg[26:28]))
		        		layout['rpmDirCarro1'].Update(str(int('0x'+ msg[30:32], base=16)))
		        		layout['sPrpmDirCarro1'].Update(str(int('0x'+ msg[32:34], base=16)))
		        		layout['sPrpmDirHEXCarro1'].Update(str('0x'+ msg[30:32]))
		        		layout['erro1Carro1'].Update(str(msg[34:36]))
		        		tempoECUsCarro1[1] = time.time()

		        		#if (time.time()-tempoinicio > 0.1):


		        		graphRPMEsqCarro1[0] = int('0x'+ msg[26:28], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPRPMEsqCarro1[0] = int('0x'+ msg[28:30], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphRPMDirCarro1[0] = int('0x'+ msg[30:32], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPRPMDirCarro1[0] = int('0x'+ msg[32:34], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA

		        		#calcula media RPM Esq
		        		medGraphRPMEsqCarro1.insert(0,medGraphRPMEsqCarro1.pop()) # rotaciona para direita >>>>
		        		medGraphRPMEsqCarro1[0] = graphRPMEsqCarro1[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo RPM Esq
		        		if graphRPMEsqCarro1[0] > maxGraphRPMEsqCarro1:
		        			maxGraphRPMEsqCarro1 = graphRPMEsqCarro1[0]

		        		#calcula media RPM Dir
		        		medGraphRPMDirCarro1.insert(0,medGraphRPMDirCarro1.pop()) # rotaciona para direita >>>>
		        		medGraphRPMDirCarro1[0] = graphRPMDirCarro1[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo RPM Dir
		        		if graphRPMDirCarro1[0] > maxGraphRPMDirCarro1:
		        			maxGraphRPMDirCarro1 = graphRPMDirCarro1[0]


		        if layout['idCANCarro12'].get() != "":
	        	
		        	if int('0x'+ layout['idCANCarro12'].get(), base=16) == int('0x'+ msg[18:20], base=16):

		        		#print(msg[6:8])
		        		layout['posCarro1'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['setpointCarro1'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['ohmCarro1'].Update(str(int('0x'+ msg[30:34], base=16)))
		        		layout['erro2Carro1'].Update(str(msg[34:36]))
		        		tempoECUsCarro1[2] = time.time()

		        		graphDirecCarro1[0] = int('0x'+ msg[26:28], base=16) * fatorAjusteDirecao #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPDirecCarro1[0] = int('0x'+ msg[28:30], base=16) * fatorAjusteDirecao #AJUSTE PRA MELHORAR A VISÃO NA TELA

		        		#calcula media 
		        		medGraphDirecCarro1.insert(0,medGraphDirecCarro1.pop()) # rotaciona para direita >>>>
		        		medGraphDirecCarro1[0] = graphDirecCarro1[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo
		        		if graphsPDirecCarro1[0] > maxGraphsPDirecCarro1:
		        			maxGraphsPDirecCarro1 = graphsPDirecCarro1[0]
		        			

		        if layout['idCANCarro13'].get() != "":

		        	#if int(layout['idCANCarro13'].get()) == int('0x'+ msg[6:8], base=16) and int(layout['msgTypeCarro13'].get()) == int('0x'+ msg[12:14], base=16):
		        	if int('0x'+ layout['idCANCarro13'].get(), base=16) == int('0x'+ msg[18:20], base=16):

		        		#print(msg[6:8])		
		        		layout['byte0Carro1'].Update(str(int('0x'+ msg[20:22], base=16)))
		        		layout['byte1Carro1'].Update(str(int('0x'+ msg[22:24], base=16)))
		        		layout['byte2Carro1'].Update(str(int('0x'+ msg[24:26], base=16)))
		        		layout['byte3Carro1'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['byte4Carro1'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['byte5Carro1'].Update(str(int('0x'+ msg[30:32], base=16)))
		        		layout['byte6Carro1'].Update(str(int('0x'+ msg[32:34], base=16)))
		        		layout['byte7Carro1'].Update(str(int('0x'+ msg[34:36], base=16)))

		        		tempoECUsCarro1[3] = time.time()


	        	# Desenha o grafico RPM ESQUERDA CARRO1
	        	layout['graphRPMEsqCarro1'].delete_figure(textoGraphRPMEsqCarro1) #remove o texto do grafico id
	        	textoGraphRPMEsqCarro1 = layout['graphRPMEsqCarro1'].draw_text(text= "--- " + ("{:.1f}".format(graphRPMEsqCarro1[0]/fatorAjusteRPM)), location=(100, 190), color='blue')
	        	

	        	layout['graphRPMEsqCarro1'].delete_figure(textoGraphsPRPMEsqCarro1) #remove o texto do grafico id
	        	textoGraphsPRPMEsqCarro1 = layout['graphRPMEsqCarro1'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPRPMEsqCarro1[0]/fatorAjusteRPM)), location=(50, 190), color='red')

	        	layout['graphRPMEsqCarro1'].delete_figure(textomedGraphRPMEsqCarro1) #remove o texto do grafico id
	        	textomedGraphRPMEsqCarro1 = layout['graphRPMEsqCarro1'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphRPMEsqCarro1)/fatorAjusteRPM)), location=(160, 190), color='black')

	        	layout['graphRPMEsqCarro1'].delete_figure(textomaxGraphRPMEsqCarro1) #remove o texto do grafico id
	        	textomaxGraphRPMEsqCarro1 = layout['graphRPMEsqCarro1'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphRPMEsqCarro1/fatorAjusteRPM)), location=(220, 190), color='black')

	        	
	        	if xRPMEsqCarro1 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphRPMEsqCarro1'].DrawLine((lastxRPMEsqCarro1, graphRPMEsqCarro1[1]), (xRPMEsqCarro1, graphRPMEsqCarro1[0]), width=2, color='blue')
	        		layout['graphRPMEsqCarro1'].DrawLine((lastxRPMEsqCarro1, graphsPRPMEsqCarro1[1]), (xRPMEsqCarro1, graphsPRPMEsqCarro1[0]), width=2, color='red')
	        		
	        		#grid lines
	        		"""
	        		#Desenha linhas horizontais
		        	for i in range(0,GRAPH_SIZE[0], GRAPH_STEP_SIZE*4): # GRAPH_SIZE(600,200) - GRAPH_SIZE(x,y)
		        		layout['graphRPMEsqCarro1'].DrawLine((0, i), (GRAPH_SIZE[0],i), width=1, color='gray') 

	        		#Desenha linhas verticais
		        	for i in range(0,GRAPH_SIZE[0], GRAPH_STEP_SIZE*4): #GRAPH_SIZE(600,200)
		        		layout['graphRPMEsqCarro1'].DrawLine((i, 0), (i,GRAPH_SIZE[1]-25), width=1, color='gray') 
					"""

	        		#/\y
	        		#|	        		
	        		#|
	        		#|
	        		#--------> x 	#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue')

	        		contLinhaVerticalRPMEsqCarro1 = 4
	        			
	        		
	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphRPMEsqCarro1'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphRPMEsqCarro1'].DrawLine((lastxRPMEsqCarro1, graphRPMEsqCarro1[1]), (xRPMEsqCarro1, graphRPMEsqCarro1[0]), width=2, color='blue')
		        	layout['graphRPMEsqCarro1'].DrawLine((lastxRPMEsqCarro1, graphsPRPMEsqCarro1[1]), (xRPMEsqCarro1, graphsPRPMEsqCarro1[0]), width=2, color='red')		        	

		        	#grid lines
		        	"""
	        		#Desenha linhas horizontais
	        		for i in range(0,GRAPH_SIZE[0], GRAPH_STEP_SIZE*4): #GRAPH_SIZE(600,200)
		        		layout['graphRPMEsqCarro1'].DrawLine((0, i), (GRAPH_SIZE[0],i), width=1, color='gray') 


	        		#Desenha linhas verticais
	        		if contLinhaVerticalRPMEsqCarro1 >= 4:
	        			contLinhaVerticalRPMEsqCarro1 = 0
	        			layout['graphRPMEsqCarro1'].DrawLine((GRAPH_SIZE[0], 0), (GRAPH_SIZE[0],200-25), width=1, color='gray')

	        		contLinhaVerticalRPMEsqCarro1 = contLinhaVerticalRPMEsqCarro1 + 1
		        	"""

		        	xRPMEsqCarro1 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxRPMEsqCarro1 = xRPMEsqCarro1
		        graphRPMEsqCarro1[1] = graphRPMEsqCarro1[0]
		        graphsPRPMEsqCarro1[1] = graphsPRPMEsqCarro1[0]
		       
	        	xRPMEsqCarro1 += GRAPH_STEP_SIZE


	        	# Desenha o grafico RPM DIREITA CARRO1
	        	layout['graphRPMDirCarro1'].delete_figure(textoGraphRPMDirCarro1) #remove o texto do grafico pelo id
	        	textoGraphRPMDirCarro1 = layout['graphRPMDirCarro1'].draw_text(text= "--- " + ("{:.1f}".format(graphRPMDirCarro1[0]/fatorAjusteRPM)), location=(100, 190), color='blue')

	        	layout['graphRPMDirCarro1'].delete_figure(textoGraphsPRPMDirCarro1) #remove o texto do grafico id
	        	textoGraphsPRPMDirCarro1 = layout['graphRPMDirCarro1'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPRPMDirCarro1[0]/fatorAjusteRPM)), location=(50, 190), color='red')

	        	layout['graphRPMDirCarro1'].delete_figure(textomedGraphRPMDirCarro1) #remove o texto do grafico id
	        	textomedGraphRPMDirCarro1 = layout['graphRPMDirCarro1'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphRPMDirCarro1)/fatorAjusteRPM)), location=(160, 190), color='black')

	        	layout['graphRPMDirCarro1'].delete_figure(textomaxGraphRPMDirCarro1) #remove o texto do grafico id
	        	textomaxGraphRPMDirCarro1 = layout['graphRPMDirCarro1'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphRPMDirCarro1/fatorAjusteRPM)), location=(220, 190), color='black')


	        	if xRPMDirCarro1 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphRPMDirCarro1'].DrawLine((lastxRPMDirCarro1, graphRPMDirCarro1[1]), (xRPMDirCarro1, graphRPMDirCarro1[0]), width=2, color='blue')
	        		layout['graphRPMDirCarro1'].DrawLine((lastxRPMDirCarro1, graphsPRPMDirCarro1[1]), (xRPMDirCarro1, graphsPRPMDirCarro1[0]), width=2, color='red')
	        		
	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphRPMDirCarro1'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphRPMDirCarro1'].DrawLine((lastxRPMDirCarro1, graphRPMDirCarro1[1]), (xRPMEsqCarro1, graphRPMDirCarro1[0]), width=2, color='blue')
		        	layout['graphRPMDirCarro1'].DrawLine((lastxRPMDirCarro1, graphsPRPMDirCarro1[1]), (xRPMEsqCarro1, graphsPRPMDirCarro1[0]), width=2, color='red')		        	

		        	#layout['graphRPMEsqCarro1'].DrawLine((lastx, lasty2), (x, y2), width=1,color='blue')
		        	#layout['graphRPMEsqCarro1'].draw_text(text='a', location=(x, y1 + 10))

		        	xRPMDirCarro1 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxRPMDirCarro1 = xRPMDirCarro1
		        graphRPMDirCarro1[1] = graphRPMDirCarro1[0]
		        graphsPRPMDirCarro1[1] = graphsPRPMDirCarro1[0]
		       
	        	xRPMDirCarro1 += GRAPH_STEP_SIZE


	        	# Desenha o grafico da DIREÇÃO 
	        	layout['graphDirecCarro1'].delete_figure(textoGraphDirecCarro1) #remove o texto do grafico id
	        	textoGraphDirecCarro1 = layout['graphDirecCarro1'].draw_text(text= "--- " + ("{:.1f}".format(graphDirecCarro1[0]/fatorAjusteDirecao)), location=(100, 190), color='blue')

	        	layout['graphDirecCarro1'].delete_figure(textoGraphsPDirecCarro1) #remove o texto do grafico id
	        	textoGraphsPDirecCarro1 = layout['graphDirecCarro1'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPDirecCarro1[0]/fatorAjusteDirecao)), location=(50, 190), color='red')

	        	layout['graphDirecCarro1'].delete_figure(textomedGraphDirecCarro1) #remove o texto do grafico id
	        	textomedGraphDirecCarro1 = layout['graphDirecCarro1'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphDirecCarro1)/fatorAjusteDirecao)), location=(160, 190), color='black')

	        	layout['graphDirecCarro1'].delete_figure(textomaxGraphsPDirecCarro1) #remove o texto do grafico id
	        	textomaxGraphsPDirecCarro1 = layout['graphDirecCarro1'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphsPDirecCarro1/fatorAjusteDirecao)), location=(220, 190), color='black')


	        	if xgraphDirecCarro1 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphDirecCarro1'].DrawLine((lastxDirecCarro1, graphDirecCarro1[1]), (xgraphDirecCarro1, graphDirecCarro1[0]), width=2, color='blue')
	        		layout['graphDirecCarro1'].DrawLine((lastxDirecCarro1, graphsPDirecCarro1[1]), (xgraphDirecCarro1, graphsPDirecCarro1[0]), width=2, color='red')
	        		
	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphDirecCarro1'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphDirecCarro1'].DrawLine((lastxDirecCarro1, graphDirecCarro1[1]), (xgraphDirecCarro1, graphDirecCarro1[0]), width=2, color='blue')
		        	layout['graphDirecCarro1'].DrawLine((lastxDirecCarro1, graphsPDirecCarro1[1]), (xgraphDirecCarro1, graphsPDirecCarro1[0]), width=2, color='red')		        	

		        	

		        	xgraphDirecCarro1 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxDirecCarro1 = xgraphDirecCarro1
		        graphDirecCarro1[1] = graphDirecCarro1[0]
		        graphsPDirecCarro1[1] = graphsPDirecCarro1[0]
		       
	        	xgraphDirecCarro1 += GRAPH_STEP_SIZE


	        msgAux = ""

	       	if layout['idCarro2'].get() == str(idCar)  and layout['idCarro2'].get() != '':
	       		
	       		tempoECUsCarro2[0] = time.time()


	       		for i in range(0,int(len(msg)-4),2): #-4 corta o 0x0d 0x0a
	        		msgAux = str(msgAux) + ' ' + str(msg[i:i+2])


	        	layout['logCarro2'].print(str(msgAux))

	        	"""
	        	#deixa o fundo de algumas msg coloridas
	        	if int('0x'+ msg[12:14], base=16) == 14 or int('0x'+ msg[12:14], base=16) == 16 or int('0x'+ msg[12:14], base=16) == 18:
	        			layout['logCarro2'].print(msgAux, text_color='red', background_color='blue')

	        	elif int('0x'+ msg[12:14], base=16) == 12:
	        			layout['logCarro2'].print(msgAux, text_color='red', background_color='green')	
	        	
	        	elif int('0x'+ msg[12:14], base=16)%2 == 0 and int('0x'+ msg[12:14], base=16) != 0 :

	        		layout['logCarro2'].print(msgAux, text_color='red', background_color='yellow')	

	        	else:
	        		layout['logCarro2'].print(msgAux)		
				"""

	        	
	        	if layout['idCANCarro21'].get() != "":

		        	if int('0x'+ layout['idCANCarro21'].get(), base=16) == int('0x'+ msg[18:20], base=16):

		        		#print(msg[6:8])
		        		layout['pidCarro2'].Update(str(msg[24:26]))
		        		layout['rpmEsqCarro2'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['sPrpmEsqCarro2'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['sPrpmEsqHEXCarro2'].Update(str('0x'+ msg[26:28]))
		        		layout['rpmDirCarro2'].Update(str(int('0x'+ msg[30:32], base=16)))
		        		layout['sPrpmDirCarro2'].Update(str(int('0x'+ msg[32:34], base=16)))
		        		layout['sPrpmDirHEXCarro2'].Update(str('0x'+ msg[30:32]))
		        		layout['erro1Carro2'].Update(str(msg[34:36]))
		        		tempoECUsCarro2[1] = time.time()

		        		graphRPMEsqCarro2[0] = int('0x'+ msg[26:28], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPRPMEsqCarro2[0] = int('0x'+ msg[28:30], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphRPMDirCarro2[0] = int('0x'+ msg[30:32], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPRPMDirCarro2[0] = int('0x'+ msg[32:34], base=16) * fatorAjusteRPM #AJUSTE PRA MELHORAR A VISÃO NA TELA

		        		#calcula media RPM Esq
		        		medGraphRPMEsqCarro2.insert(0,medGraphRPMEsqCarro2.pop()) # rotaciona para direita >>>>
		        		medGraphRPMEsqCarro2[0] = graphRPMEsqCarro2[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo RPM Esq
		        		if graphRPMEsqCarro2[0] > maxGraphRPMEsqCarro2:
		        			maxGraphRPMEsqCarro2 = graphRPMEsqCarro2[0]

		        		#calcula media RPM Dir
		        		medGraphRPMDirCarro2.insert(0,medGraphRPMDirCarro2.pop()) # rotaciona para direita >>>>
		        		medGraphRPMDirCarro2[0] = graphRPMDirCarro2[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo RPM Dir
		        		if graphRPMDirCarro2[0] > maxGraphRPMDirCarro2:
		        			maxGraphRPMDirCarro2 = graphRPMDirCarro2[0]


		        if layout['idCANCarro22'].get() != "":

		        	#if int(layout['idCANCarro22'].get()) == int('0x'+ msg[6:8], base=16) and int(layout['msgTypeCarro22'].get()) == int('0x'+ msg[12:14], base=16):
		        	if int('0x'+ layout['idCANCarro22'].get(), base=16) == int('0x'+ msg[18:20], base=16):

		        		#print(msg[6:8])
		        		layout['posCarro2'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['setpointCarro2'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['ohmCarro2'].Update(str(int('0x'+ msg[30:34], base=16)))
		        		layout['erro2Carro2'].Update(str(msg[34:36]))
		        		tempoECUsCarro2[2] = time.time()

		        		graphDirecCarro2[0] = int('0x'+ msg[26:28], base=16) * fatorAjusteDirecao #AJUSTE PRA MELHORAR A VISÃO NA TELA
		        		graphsPDirecCarro2[0] = int('0x'+ msg[28:30], base=16) * fatorAjusteDirecao #AJUSTE PRA MELHORAR A VISÃO NA TELA

		        		#calcula media 
		        		medGraphDirecCarro2.insert(0,medGraphDirecCarro2.pop()) # rotaciona para direita >>>>
		        		medGraphDirecCarro2[0] = graphDirecCarro2[0] # adiciona a nova leitura - a media é calcula com o comando mean([])

		        		# vefifica o valor maximo
		        		if graphsPDirecCarro2[0] > maxGraphsPDirecCarro2:
		        			maxGraphsPDirecCarro2 = graphsPDirecCarro2[0]


		        if layout['idCANCarro23'].get() != "":

		        	#if int(layout['idCANCarro23'].get()) == int('0x'+ msg[6:8], base=16) and int(layout['msgTypeCarro23'].get()) == int('0x'+ msg[12:14], base=16):
		        	if int('0x'+ layout['idCANCarro23'].get(), base=16) == int('0x0'+ msg[18:20], base=16):
		        	
		        		#print(msg[6:8])		
		        		layout['byte0Carro2'].Update(str(int('0x'+ msg[20:22], base=16)))
		        		layout['byte1Carro2'].Update(str(int('0x'+ msg[22:24], base=16)))
		        		layout['byte2Carro2'].Update(str(int('0x'+ msg[24:26], base=16)))
		        		layout['byte3Carro2'].Update(str(int('0x'+ msg[26:28], base=16)))
		        		layout['byte4Carro2'].Update(str(int('0x'+ msg[28:30], base=16)))
		        		layout['byte5Carro2'].Update(str(int('0x'+ msg[30:32], base=16)))
		        		layout['byte6Carro2'].Update(str(int('0x'+ msg[32:34], base=16)))
		        		layout['byte7Carro2	'].Update(str(int('0x'+ msg[34:36], base=16)))
		        		tempoECUsCarro2[3] = time.time()


	        	# Desenha o grafico RPM ESQUERDA
	        	layout['graphRPMEsqCarro2'].delete_figure(textoGraphRPMEsqCarro2) #remove o texto do grafico id
	        	textoGraphRPMEsqCarro2 = layout['graphRPMEsqCarro2'].draw_text(text= "--- " + ("{:.1f}".format(graphRPMEsqCarro2[0]/fatorAjusteRPM)), location=(100, 190), color='blue')

	        	layout['graphRPMEsqCarro2'].delete_figure(textoGraphsPRPMEsqCarro2) #remove o texto do grafico id
	        	textoGraphsPRPMEsqCarro2 = layout['graphRPMEsqCarro2'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPRPMEsqCarro2[0]/fatorAjusteRPM)), location=(50, 190), color='red')

	        	layout['graphRPMEsqCarro2'].delete_figure(textomedGraphRPMEsqCarro2) #remove o texto do grafico id
	        	textomedGraphRPMEsqCarro2 = layout['graphRPMEsqCarro2'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphRPMEsqCarro2)/fatorAjusteRPM)), location=(160, 190), color='black')

	        	layout['graphRPMEsqCarro2'].delete_figure(textomaxGraphRPMEsqCarro2) #remove o texto do grafico id
	        	textomaxGraphRPMEsqCarro2 = layout['graphRPMEsqCarro2'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphRPMEsqCarro2/fatorAjusteRPM)), location=(220, 190), color='black')

	        	
	        	if xRPMEsqCarro2 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphRPMEsqCarro2'].DrawLine((lastxRPMEsqCarro2, graphRPMEsqCarro2[1]), (xRPMEsqCarro2, graphRPMEsqCarro2[0]), width=2, color='blue')
	        		layout['graphRPMEsqCarro2'].DrawLine((lastxRPMEsqCarro2, graphsPRPMEsqCarro2[1]), (xRPMEsqCarro2, graphsPRPMEsqCarro2[0]), width=2, color='red')
	        		

	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphRPMEsqCarro2'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphRPMEsqCarro2'].DrawLine((lastxRPMEsqCarro2, graphRPMEsqCarro2[1]), (xRPMEsqCarro2, graphRPMEsqCarro2[0]), width=2, color='blue')
		        	layout['graphRPMEsqCarro2'].DrawLine((lastxRPMEsqCarro2, graphsPRPMEsqCarro2[1]), (xRPMEsqCarro2, graphsPRPMEsqCarro2[0]), width=2, color='red')		        	

		        	#layout['graphRPMEsqCarro2'].DrawLine((lastx, lasty2), (x, y2), width=1,color='blue')
		        	#layout['graphRPMEsqCarro2'].draw_text(text='a', location=(x, y1 + 10))

		        	xRPMEsqCarro2 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxRPMEsqCarro2 = xRPMEsqCarro2
		        graphRPMEsqCarro2[1] = graphRPMEsqCarro2[0]
		        graphsPRPMEsqCarro2[1] = graphsPRPMEsqCarro2[0]
		       
	        	xRPMEsqCarro2 += GRAPH_STEP_SIZE



	        	# Desenha o grafico RPM DIREITA
	        	layout['graphRPMDirCarro2'].delete_figure(textoGraphRPMDirCarro2) #remove o texto do grafico id
	        	textoGraphRPMDirCarro2 = layout['graphRPMDirCarro2'].draw_text(text= "--- " + ("{:.1f}".format(graphRPMDirCarro2[0]/fatorAjusteRPM)), location=(100, 190), color='blue')

	        	layout['graphRPMDirCarro2'].delete_figure(textoGraphsPRPMDirCarro2) #remove o texto do grafico id
	        	textoGraphsPRPMDirCarro2 = layout['graphRPMDirCarro2'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPRPMDirCarro2[0]/fatorAjusteRPM)), location=(50, 190), color='red')
	        	
	        	layout['graphRPMDirCarro2'].delete_figure(textomedGraphRPMDirCarro2) #remove o texto do grafico id
	        	textomedGraphRPMDirCarro2 = layout['graphRPMDirCarro2'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphRPMDirCarro2)/fatorAjusteRPM)), location=(160, 190), color='black')

	        	layout['graphRPMDirCarro2'].delete_figure(textomaxGraphRPMDirCarro2) #remove o texto do grafico id
	        	textomaxGraphRPMDirCarro2 = layout['graphRPMDirCarro2'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphRPMDirCarro2/fatorAjusteRPM)), location=(220, 190), color='black')

	        	
	        	if xRPMDirCarro2 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphRPMDirCarro2'].DrawLine((lastxRPMDirCarro2, graphRPMDirCarro2[1]), (xRPMDirCarro2, graphRPMDirCarro2[0]), width=2, color='blue')
	        		layout['graphRPMDirCarro2'].DrawLine((lastxRPMDirCarro2, graphsPRPMDirCarro2[1]), (xRPMDirCarro2, graphsPRPMDirCarro2[0]), width=2, color='red')
	        		
	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphRPMDirCarro2'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphRPMDirCarro2'].DrawLine((lastxRPMDirCarro2, graphRPMDirCarro2[1]), (xRPMEsqCarro2, graphRPMDirCarro2[0]), width=2, color='blue')
		        	layout['graphRPMDirCarro2'].DrawLine((lastxRPMDirCarro2, graphsPRPMDirCarro2[1]), (xRPMEsqCarro2, graphsPRPMDirCarro2[0]), width=2, color='red')		        	


		        	xRPMDirCarro2 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxRPMDirCarro2 = xRPMDirCarro2
		        graphRPMDirCarro2[1] = graphRPMDirCarro2[0]
		        graphsPRPMDirCarro2[1] = graphsPRPMDirCarro2[0]
		       
	        	xRPMDirCarro2 += GRAPH_STEP_SIZE


	        	# Desenha o grafico da DIREÇÃO
	        	layout['graphDirecCarro2'].delete_figure(textoGraphDirecCarro2) #remove o texto do grafico id
	        	textoGraphDirecCarro2 = layout['graphDirecCarro2'].draw_text(text= "--- " + ("{:.1f}".format(graphDirecCarro2[0]/fatorAjusteDirecao)), location=(100, 190), color='blue')
 
	        	layout['graphDirecCarro2'].delete_figure(textoGraphsPDirecCarro2) #remove o texto do grafico id
	        	textoGraphsPDirecCarro2 = layout['graphDirecCarro2'].draw_text(text= "sp --- " + ("{:.1f}".format(graphsPDirecCarro2[0]/fatorAjusteDirecao)), location=(50, 190), color='red')

	        	layout['graphDirecCarro2'].delete_figure(textomedGraphDirecCarro2) #remove o texto do grafico id
	        	textomedGraphDirecCarro2 = layout['graphDirecCarro2'].draw_text(text= "Med: " + ("{:.1f}".format(mean(medGraphDirecCarro2)/fatorAjusteDirecao)), location=(160, 190), color='black')

	        	layout['graphDirecCarro2'].delete_figure(textomaxGraphsPDirecCarro2) #remove o texto do grafico id
	        	textomaxGraphsPDirecCarro2 = layout['graphDirecCarro2'].draw_text(text= "Máx: " + ("{:.1f}".format(maxGraphsPDirecCarro2/fatorAjusteDirecao)), location=(220, 190), color='black')



	        	if xgraphDirecCarro2 < GRAPH_SIZE[0]:               # if still drawing initial width of graph
	        		layout['graphDirecCarro2'].DrawLine((lastxDirecCarro2, graphDirecCarro2[1]), (xgraphDirecCarro2, graphDirecCarro2[0]), width=2, color='blue')
	        		layout['graphDirecCarro2'].DrawLine((lastxDirecCarro2, graphsPDirecCarro2[1]), (xgraphDirecCarro2, graphsPDirecCarro2[0]), width=2, color='red')
	        		
	        		#window['-GRAPH-'].DrawLine((lastx, lasty2), (x, y2), width=1, color='blue') 
	        	else: # finished drawing full graph width so move each time to make room
		        	layout['graphDirecCarro2'].Move(-GRAPH_STEP_SIZE, 0)
		        	layout['graphDirecCarro2'].DrawLine((lastxDirecCarro2, graphDirecCarro2[1]), (xgraphDirecCarro2, graphDirecCarro2[0]), width=2, color='blue')
		        	layout['graphDirecCarro2'].DrawLine((lastxDirecCarro2, graphsPDirecCarro2[1]), (xgraphDirecCarro2, graphsPDirecCarro2[0]), width=2, color='red')		        	

		        	

		        	xgraphDirecCarro2 -= GRAPH_STEP_SIZE


			    #lastx, lasty1, lasty2 = x, y1, y2
		        #grapRPMEsqCarro1[valorMedido,lastx,lasty]

		        lastxDirecCarro2 = xgraphDirecCarro2
		        graphDirecCarro2[1] = graphDirecCarro2[0]
		        graphsPDirecCarro2[1] = graphsPDirecCarro2[0]
		       
	        	xgraphDirecCarro2 += GRAPH_STEP_SIZE

				
	    #portaCOM.flush() #Limpa a comunicação
	    try:
	    	portaCOM.flush() #Limpa a comunicação
	    except:
	    	pass

	    if stop_thread:

	    	try:
		    	portaCOM.flush() #Limpa a comunicação
	    	except:
	    		pass
	    
	    	break

##################################################################################	
#																				 #
#									  INICIO									 #
#																				 #
##################################################################################	

tempoAux = time.time()
tempoECUsCarro1 = [tempoAux,tempoAux,tempoAux,tempoAux]
tempoECUsCarro2 = [tempoAux,tempoAux,tempoAux,tempoAux]


GRAPH_SIZE = (600,200) #Tamanho do grafico


periodoRqstDados = time.time()

contlistaIdCarros = 1 #inicia em 1 (endereço do carro lider)

stop_thread = False


ports = serial.tools.list_ports.comports() #pega as portas disponiveis

for p in ports:
    print(p.device) #lista a porta
    print(p) #lista a porta com o device conectado

print(len(ports), 'ports found') 



sg.theme('Dark Grey 13')


"""
separar tudo em 4 abas (1 para cada carro)
graficos com valores
ler valores do ardino
setar valores no arduino
gerar graficos

menu com todos os ajustes
timout para ler a resposta do arduino
botao para recalibara deireção
botão pra resetar 
"""

"""
#usados pra desenhar switch button
toggle_btn_off = b'iVBORw0KGgoAAAANSUhEUgAAADAAAAAWCAYAAACG9x+sAAAACXBIWXMAAA7DAAAOwwHHb6hkAAAAGXRFWHRTb2Z0d2FyZQB3d3cuaW5rc2NhcGUub3Jnm+48GgAAAu5JREFUWIW9l99LU2EYx7/ve7blpm6w3cTOYnVT7hwiBJMudHVR4KQUr7qOgYhehPbjf0hIBXG7OgXdBSEyyB90UxNyIIjhPBoNodzcjWexoSic7bxdOEPKzene7XP58vDh+/A+h/c8BCUIBoNOYjL1EUJ6CHCDEeIBY42l6rlCyD5hLMmATQARQ9dnFEXJnFr670F/f7+NCcKISRBeyJJkl3w+iKIIh90OAMjmckilUlhXVagbG8jn87Vt5ogcAUYbrdax8fHxg5INDAwMiHlg5qYst3UHAnA5nWWtmqZhbmEBa/F4DTKfyjdDEHqVqamfxwd/G3gyOHjlEhALdHW5Ozs6zmWNLi5idn4ejDGOWUuyk6f0zttQaBsABOBobATgU3cgcP284QHA6/XCbDbjRyLBOeupNAuM3fN3dr6LxWJ5CgCMkOeyLLdeJPwxd/1+yJLELWU5GNC6d3j4DADI0NCQyyBka+TpU7vL5apKnMlk8Hpiom4ftqHr16huGH2yJFUdHgCcTid8LS0cslWEnVgsvZQx9pDn1ddrjACAMPaIApBFt5ub1OPxcHOdBQEkCuCyvfhI8cDB0XUWDBApd2l93oJjDAogncvluBl5uipghwJQk6kUN+M2R1cFbFAAkXVV5WZUOboqIEINXZ9ZV9XcrqZVbdMyGWxsbnLIVRFZQ9cjVFGUjFEojM4vLFRt/Dg7W69XGGDslaIoGQoAjVbr2Fo8vhJdXLyw73M0Cp6jeAYrTTbbBFD8G43FYvlb7e1zW4nEY7PF0nzV6z2X7Us0Ch43WCE7eUrvhycnNaDYAACsLi/nbre1vf+eSPjT6bRbFEXYbLaypl1Nw4fpaXxdWqpx5iKMrRom04M3odCv46P/Vsrh4WHr/sHBCBWEl5LPZ5clCR5RhMPhAABks1kkT6yUhUKhHtGzjJDR5oaG8bIr5UmCwaCTWCy9xDB6QEgLAA+AplonLbIHIAnGNhmlETMwEw6Hf59W+AeEBxzSTJhqkQAAAABJRU5ErkJggg=='
toggle_btn_on = b'iVBORw0KGgoAAAANSUhEUgAAADAAAAAWCAYAAACG9x+sAAAACXBIWXMAAA7DAAAOwwHHb6hkAAAAGXRFWHRTb2Z0d2FyZQB3d3cuaW5rc2NhcGUub3Jnm+48GgAAAr9JREFUWIXFl0tPE1EYhp9v5syAiCwgXgItiXEhlxXEhTvBtXKJW417NZoU9T9IYkE0Ju68bF1gf4CAKzcWXXDZqLEdMF7AWBBs50yPixY1Ci3SoT7LM9955/2Sc2a+V9iCSNw0OpY/iCV9GI4CEWDvVvUh8w3wgHkRk8hpZ9yLyfJmhfLnQvM9U1eT1TEM14CGXTaKY0Nv1KcnGtDWGHCgzgDwcU2YX7aZSNlMek7GDxj2AxX3YrK+ZQORuGlxLD2OcGy3jQOcbPW50p0lss+UrPNWhFvJWp6m1Cts1f/2krzbePazgSM3TTSv9HOgefcsF7AFLndnOduR+6d9j2ZdbidrFo2vjr8ekjSABYVjk1f6CVUwDzszD3CuI8fFrmxzXuknkbjZA8UG3Ky+CnSFa3NzTrb6OzK/wfnOHL2tustReghAWu6YJjfQb6jShX18erXsmS/HwqpwJlGfWcupw5Yb+INUwTwUvjaVmgdoqTeciPgNru33WyCnQvC2LXqiQchactpCpDM01TK0N+rQtDqaAgx0WBhzKDTVMuyvC0+r+MNrscKTrC75wlXKW8D7ar300/pfk0ulWosWMBuaahnmluzQtGY/2wjMWSImEZpqGSZS4TUwmbbJi0lYOe2MA5nQlEswkXZIZyo/Rt6K8GzB+aq1k7CKc/Zw5fbKo/MwNl1bsc7Ii1p0wA0vJssWgB+ouAjJipW3wdOU4tGsu+P992dcpjyVzAVqFIrDnBeTdfHVALAYjs3SjCVreLiDJh7MuNydrlkUXw1sBJv/Gmh6WzVXur4TbSg9H6Uzwmiylsm0eomtBjYNNBtE4maPY+sYcJ0qDHnKgp5ipGxvCjhYjJQf1oS5pUKknPKcrzpg2M+rkZKR8ncicdPo2n4/In3G0EYh1Nfvaje/WAU8EeYxJqEdZzx1Qb5sVvgDAJEFQLjoGwcAAAAASUVORK5CYII='
flag_switch_button_conectar = False
"""

layout = [ #Matriz do nosso layout


			[sg.Column([

				[sg.Text('', size =(10, 1))],
				[sg.Combo(ports, key ='comboPortsCOM'), sg.Button('Desconectado', size = (12, 1), key ='buttonCOM', button_color='red')],
				[sg.Text('Baudrate: 115200', size =(20, 1), key = 'baudRate')],


			 ], scrollable=False,  vertical_scroll_only=True, size=(645, 110) #scroll da janela


        	),

        	sg.VerticalSeparator(pad=(20,0)),


			sg.Column([
			
				#[sg.Text('', size =(10, 1))],
				[sg.Text('Lista de MAC ativos', size =(20, 1))],
				[sg.Multiline(size=(25, 4), key='logMAC', autoscroll=True, disabled=True, do_not_clear=True, font=(sg.DEFAULT_FONT, 10))],


			], scrollable=False,  vertical_scroll_only=True, size=(630, 110) #scroll da janela

        	)],

        	


			
			
			
		    #[sg.Text('Conectar Carros', size =(15, 1), key = 'switch_button_conectar_texto'), sg.Button('', image_data = toggle_btn_off, key='switch_button_conectar', button_color=(sg.theme_background_color(), sg.theme_background_color()), border_width=0)],
		    [sg.HorizontalSeparator(pad=None)],

			
			[sg.Column([
			    
				[sg.Text('MAC Carro 1', size =(10, 1), key = 'idCarro1label'), sg.InputText('A8:03:2A:19:B1:08', size =(20, 1), key='idCarro1')],
				[sg.Multiline(size=(85, 6), key='logCarro1', autoscroll=True, disabled=True, do_not_clear=True, font=(sg.DEFAULT_FONT, 9))],
    			#[sg.Checkbox('Auto Scoll',key='logCar1AutoScroll', enable_events=True, default=True)],
    			[sg.Text('', size =(10, 1))],


    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('50', size =(10, 1), key='idCANCarro11'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro11')],
    			
				#[sg.Multiline(size=(85, 10), key='logEcuCar1', autoscroll=True, disabled=True, do_not_clear=True, font=(sg.DEFAULT_FONT, 8))],
    			#[sg.Checkbox('Auto Scoll',key='logEcuCar1AutoScroll', enable_events=True, default=True)],

    			[sg.Text('PID', size =(10, 1)), sg.InputText('', size =(10, 1), key='pidCarro1')],
    			[sg.Text('RPM Esq', size =(10, 1)), sg.InputText('', size =(10, 1), key='rpmEsqCarro1'), sg.Text('Sp. RPM Esq', size =(10, 1)), sg.InputText('', size =(10, 1), key='sPrpmEsqCarro1'), sg.Text('', size =(10, 1),key='sPrpmEsqHEXCarro1')],
    			[sg.Text('RPM Dir', size =(10, 1)), sg.InputText('', size =(10, 1), key='rpmDirCarro1'), sg.Text('Sp. RPM Dir', size =(10, 1)), sg.InputText('', size =(10, 1), key='sPrpmDirCarro1'), sg.Text('', size =(10, 1),key='sPrpmDirHEXCarro1')],
    			[sg.Text('Erro', size =(10, 1)), sg.InputText('', size =(10, 1), key='erro1Carro1')],

    			 [sg.HorizontalSeparator(pad=None)],

    			#[sg.Text('', size =(10, 1))],
    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('80', size =(10, 1), key='idCANCarro12'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro12')],

    			[sg.Text('Âng Direc', size =(10, 1)), sg.InputText('', size =(10, 1), key='posCarro1'),sg.Text('\u00b0', size =(10, 1))],
    			[sg.Text('Sp. Âng Direc', size =(10, 1)), sg.InputText('', size =(10, 1), key='setpointCarro1'), sg.Text('\u00b0', size =(10, 1))],
    			[sg.Text('Resistência', size =(10, 1)), sg.InputText('', size =(10, 1), key='ohmCarro1'), sg.Text('\u03A9', size =(10, 1))],
    			[sg.Text('Erro', size =(10, 1)), sg.InputText('', size =(10, 1), key='erro2Carro1')],


    			[sg.HorizontalSeparator(pad=None)],
    			#[sg.Text('', size =(10, 1))],
    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('00', size =(10, 1), key='idCANCarro13'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro13')],

    			[sg.Text('Byte 0', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte0Carro1')],
    			[sg.Text('Byte 1', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte1Carro1')],
    			[sg.Text('Byte 2', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte2Carro1')],
    			[sg.Text('Byte 3', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte3Carro1')],
    			[sg.Text('Byte 4', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte4Carro1')],
    			[sg.Text('Byte 5', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte5Carro1')],
    			[sg.Text('Byte 6', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte6Carro1')],
    			[sg.Text('Byte 7', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte7Carro1')],


    			[sg.Text('', size =(10, 1))],
			 	#[sg.Combo(['Ajuste', 'Velocidade Ambos', 'Velocidade Esq', 'Velocidade Dir', 'PWM Ambos','Angulo Direção', 'PID Esquerda', 'PID Direita', 'PID Direção'], default_value = 'Ajuste', key ='setValoresCarro1', enable_events=True, size = (22, 1)), sg.InputText('', size =(4, 1), key='valorCarro11', disabled=True), sg.InputText('', size =(4, 1), key='valorCarro12', disabled=True),sg.InputText('', size =(4, 1), key='valorCarro13', disabled=True),sg.Text('HEX', size =(10, 1), key='labelValorCarro1'), sg.Button('Ajustar', size = (12, 1), key='botaoAjustaCarro1')],
			 	[sg.Combo(['Ajuste', 'Velocidade Ambos', 'Velocidade Esq', 'Velocidade Dir', 'PWM Ambos','Angulo Direção'], default_value = 'Ajuste', key ='setValoresCarro1', enable_events=True, size = (22, 1)), sg.InputText('', size =(4, 1), key='valorCarro11', disabled=True), sg.InputText('', size =(4, 1), key='valorCarro12', disabled=True),sg.InputText('', size =(4, 1), key='valorCarro13', disabled=True),sg.Text('HEX', size =(10, 1), key='labelValorCarro1'), sg.Button('Ajustar', size = (12, 1), key='botaoAjustaCarro1')],
	

			 	[sg.Text('', size =(10, 1))],
			 	[sg.Button('Freio de Emergência', size = (18, 1), key='botaoEBSCarro1', button_color='red')],


				[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('RPM Esq:', size =(10, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphRPMEsqCarro1', background_color='white')],
			 
			 	[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('RPM Dir:', size =(10, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphRPMDirCarro1', background_color='white')],

			 	[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('Ângulo Direção:', size =(15, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphDirecCarro1', background_color='white')],


			 	[sg.Text('', size =(10, 1))],
			 	#[sg.HorizontalSeparator(pad=None)],
			 	#[sg.Text('Distância:', size =(15, 1))],
			 	#[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphDistanceCarro1', background_color='white')],
			 	[sg.Text('', size =(10, 1))],


			    ], scrollable=True,  vertical_scroll_only=True, size=(630, 650) #scroll da janela


        	),

        	sg.VerticalSeparator(pad=(20,0)),

        	sg.Column([

		   		[sg.Text('MAC Carro 2', size =(10, 1), key = 'idCarro2label'), sg.InputText('98:CD:AC:A9:7D:E4', size =(20, 1), key='idCarro2')],
				[sg.Multiline(size=(85, 6), key='logCarro2', autoscroll=True, disabled=True, do_not_clear=True, font=(sg.DEFAULT_FONT, 9))],
    			#[sg.Checkbox('Auto Scoll',key='logCar2AutoScroll', enable_events=True, default=True)],
    			[sg.Text('', size =(10, 1))],

    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('50', size =(10, 1), key='idCANCarro21'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro21')],
    			#[sg.Multiline(size=(85, 10), key='logEcuCar2', autoscroll=True, disabled=True, do_not_clear=True, font=(sg.DEFAULT_FONT, 9))],
    			#[sg.Checkbox('Auto Scoll',key='logEcuCar2AutoScroll', enable_events=True, default=True)],

    			[sg.Text('PID', size =(10, 1)), sg.InputText('', size =(10, 1), key='pidCarro2')],
    			[sg.Text('RPM Esq', size =(10, 1)), sg.InputText('', size =(10, 1), key='rpmEsqCarro2'), sg.Text('Sp. RPM Esq', size =(10, 1)), sg.InputText('', size =(10, 1), key='sPrpmEsqCarro2'), sg.Text('', size =(10, 1),key='sPrpmEsqHEXCarro2')],
    			[sg.Text('RPM Dir', size =(10, 1)), sg.InputText('', size =(10, 1), key='rpmDirCarro2'), sg.Text('Sp. RPM Dir', size =(10, 1)), sg.InputText('', size =(10, 1), key='sPrpmDirCarro2'), sg.Text('', size =(10, 1),key='sPrpmDirHEXCarro2')],
    			[sg.Text('Erro', size =(10, 1)), sg.InputText('', size =(10, 1), key='erro1Carro2')],

    			[sg.HorizontalSeparator(pad=None)],
    			#[sg.Text('', size =(10, 1))],
    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('80', size =(10, 1), key='idCANCarro22'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro22')],
    			

    			[sg.Text('Âng Direc', size =(10, 1)), sg.InputText('', size =(10, 1), key='posCarro2'),sg.Text('\u00b0', size =(10, 1))],
    			[sg.Text('Sp. Âng Direc', size =(10, 1)), sg.InputText('', size =(10, 1), key='setpointCarro2'), sg.Text('\u00b0', size =(10, 1))],
    			[sg.Text('Resistência', size =(10, 1)), sg.InputText('', size =(10, 1), key='ohmCarro2'), sg.Text('\u03A9', size =(10, 1))],
    			[sg.Text('Erro', size =(10, 1)), sg.InputText('', size =(10, 1), key='erro2Carro2')],


    			[sg.HorizontalSeparator(pad=None)],
    			#[sg.Text('', size =(10, 1))],
    			[sg.Text('ID msg CAN', size =(10, 1)), sg.InputText('00', size =(10, 1), key='idCANCarro23'), sg.Text('-', size =(10, 1), key='msgTypeTimeCarro23')],

    			[sg.Text('Byte 0', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte0Carro2')],
    			[sg.Text('Byte 1', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte1Carro2')],
    			[sg.Text('Byte 2', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte2Carro2')],
    			[sg.Text('Byte 3', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte3Carro2')],
    			[sg.Text('Byte 4', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte4Carro2')],
    			[sg.Text('Byte 5', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte5Carro2')],
    			[sg.Text('Byte 6', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte6Carro2')],
    			[sg.Text('Byte 7', size =(10, 1)), sg.InputText('', size =(10, 1), key='byte7Carro2')],



    			[sg.Text('', size =(10, 1))],
			 	#[sg.Combo(['Ajuste', 'Velocidade Ambos', 'Velocidade Esq', 'Velocidade Dir', 'PWM Ambos','Angulo Direção', 'PID Esquerda', 'PID Direita', 'PID Direção'], default_value = 'Ajuste', key ='setValoresCarro2', enable_events=True, size = (22, 1)), sg.InputText('', size =(4, 1), key='valorCarro21', disabled=True), sg.InputText('', size =(4, 1), key='valorCarro22', disabled=True),sg.InputText('', size =(4, 1), key='valorCarro23', disabled=True),sg.Text('HEX', size =(10, 1), key='labelValorCarro2'), sg.Button('Ajustar', size = (12, 1), key='botaoAjustaCarro2')],
			 	[sg.Combo(['Ajuste', 'Velocidade Ambos', 'Velocidade Esq', 'Velocidade Dir', 'PWM Ambos','Angulo Direção'], default_value = 'Ajuste', key ='setValoresCarro2', enable_events=True, size = (22, 1)), sg.InputText('', size =(4, 1), key='valorCarro21', disabled=True), sg.InputText('', size =(4, 1), key='valorCarro22', disabled=True),sg.InputText('', size =(4, 1), key='valorCarro23', disabled=True),sg.Text('HEX', size =(10, 1), key='labelValorCarro2'), sg.Button('Ajustar', size = (12, 1), key='botaoAjustaCarro2')],


			 	[sg.Text('', size =(10, 1))],
			 	[sg.Button('Freio de Emergência', size = (18, 1), key='botaoEBSCarro2', button_color='red')],


			 	[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('RPM Esq:', size =(10, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphRPMEsqCarro2', background_color='white')],
			 	
			 	[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('RPM Dir:', size =(10, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphRPMDirCarro2', background_color='white')],

				[sg.Text('', size =(10, 1))],
			 	[sg.HorizontalSeparator(pad=None)],
			 	[sg.Text('Ângulo Direção:', size =(15, 1))],
			 	[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphDirecCarro2', background_color='white')],


			 	[sg.Text('', size =(10, 1))],
			 	#[sg.HorizontalSeparator(pad=None)],
			 	#[sg.Text('Distância:', size =(15, 1))],
			 	#[sg.Graph(GRAPH_SIZE, (0,0), GRAPH_SIZE, key='graphDistanceCarro2', background_color='white')],
			 	[sg.Text('', size =(10, 1))],


			    ], scrollable=True,  vertical_scroll_only=True, size=(630, 650) #scroll da janela

		   	)],


		    #[sg.Combo(['Velocidade', 'Angulo direção', 'PID Esquerda', 'PID Direita'], default_value = 'Selecione', key ='getValores', enable_events=True)],
		    #[sg.Combo(['Velocidade', 'Angulo direção', 'PID Esquerda', 'PID Direita'], default_value = 'Selecione', key ='setValores', enable_events=True)],


		]



layout = sg.Window('RF Platoon Control', layout=layout, size=(1350, 680), resizable=True, finalize=True)
#layout = sg.Window('Script launcher', no_titlebar=True, grab_anywhere=True, keep_on_top=True, background_color='black')



portaCOM = srl.Serial()

commandCarro1 = "" #comando enviado via serial (RF) [apos pressionar o botão ajustar] para executar ações no carro1 (girar rodas, direção, etc)
commandCarro2 = "" #comando enviado via serial (RF) [apos pressionar o botão ajustar] para executar ações no carro2 (girar rodas, direção, etc)

commandEBS = "" #comaando enviado via (RF) para frear um veículo (Faz o PWM ir para Zero - Trava as rodas)

flagComandoCarro1 = False
flagComandoCarro2 = False


while True: #Laço para o envio do comandos


	#events, values = layout.Read()
	events, values = layout.Read(timeout = 200)#Leitural dos valores e eventos da tela
	#Window.maximize()
	#layout.read( timeout=250 )
	#events, values = layout.read(timeout = 250) 

	#atualiza a lista de carros (usado no pooling de dados via RF)
	#listaIdCarros[0] = layout['idCarro1'].get()
	#listaIdCarros[1] = layout['idCarro2'].get()


	#rotina que faz o pooling nos veículos

	if portaCOM.isOpen(): #testa se a serial esta aberta

		"""
		#contlistaIdCarros = 1

		#envia msg se freio de emergencia
		if contlistaIdCarros == 0 and commandEBS != "":
			command = bytearray.fromhex(commandEBS)		
			portaCOM.write(command)
			print("Freio de Emergência: " + str(command))
			commandEBS = ""

		if contlistaIdCarros == 1:
		#if listaIdCarros[contlistaIdCarros] == '1':

			if	commandCarro1 != "":
				portaCOM.write(commandCarro1)
				print("Action: " + str(commandCarro1))
				commandCarro1 = ""
				flagComandoCarro1 = False

			# comando q faz a requisição de dados das ecus dos carros
			
			#command = bytearray.fromhex('000' + layout['idCarro1'].get() + 'AAAA880000000000')		
			#portaCOM.write(command)
			#print("Requsest: " + str(command))
			

		if contlistaIdCarros == 2:
		#elif listaIdCarros[contlistaIdCarros] == '2':

			if	commandCarro2 != "":
				portaCOM.write(commandCarro2)
				print("Action: " + str(commandCarro2))
				commandCarro2 = ""
				flagComandoCarro2 = False

			# comando q faz a requisição de dados das ecus dos carros
			
			#command = bytearray.fromhex('000' + layout['idCarro2'].get() + 'AAAA880000000000')		
			#portaCOM.write(command)
			#print("Requsest: " + str(command))
			

		contlistaIdCarros = contlistaIdCarros + 1

		# reinicia o pooling
		if contlistaIdCarros >= 3:
			contlistaIdCarros = 0

		"""
	#FIM da rotina que faz o pooling nos veículos


	if events == sg.Window.read:
		pass


	if events == sg.WINDOW_CLOSED: #Checa se a janela é fechada
		break


	if events is None:
		break


	elif events == 'botaoEBSCarro1': #evento caso o botão de freio de emergencia seja presionado
		
		macCarroDestino= layout['idCarro1'].get().replace(":", "")
		msgPlatoon = "0000"

		commandEBS = macCarroDestino + msgPlatoon + "005C" + "0000000001" + "00" + "01" + "00"

		command = bytearray.fromhex(commandEBS)
		portaCOM.write(command)


	elif events == 'botaoEBSCarro2': #evento caso o botão de freio de emergencia seja presionado

		macCarroDestino= layout['idCarro2'].get().replace(":", "")
		msgPlatoon = "0000"

		commandEBS = macCarroDestino + msgPlatoon + "005C" + "0000000001" + "00" + "01" + "00"

		command = bytearray.fromhex(commandEBS)
		portaCOM.write(command)


	elif events == 'buttonCOM': #eventos do botao conectar na com

		if values['comboPortsCOM'] != "" and layout['buttonCOM'].get_text() == 'Desconectado' and not portaCOM.isOpen():
			print(values['comboPortsCOM'][0])
			layout['buttonCOM'].Update(text = 'Conectado', button_color = 'black')

			layout['comboPortsCOM'].update(disabled=True) #desabilita o combo

			try: #Tenta se conectar ao Arduino

			    #port = 'COM8'
			    port = values['comboPortsCOM'][0]
			    baudrate = 115200
			    portaCOM = srl.Serial(port=port, baudrate=baudrate, bytesize=8, timeout=1) #Cria um elemento conectado a porta COM
			    #portaCOM.flush()
			    portaCOM.flushInput()
			    #break

			    #thread q verifica os dados chegando via serial
			    t0 = Thread(target = verificaSerial, args=(portaCOM, layout, tempoECUsCarro1, tempoECUsCarro2, stop_thread)) #passa porta com e o elemento do layout
			    t0.start()


			    print('COM Conectado')


			except Exception: #Se não for possível, imprime a mensagem e dá um delay de 1 segundo
			    print('Não foi possível se conectar ao Arduino')


		elif layout['buttonCOM'].get_text() == 'Conectado' and portaCOM.isOpen():
			layout['buttonCOM'].Update(text = 'Desconectado', button_color = 'red')
			layout['comboPortsCOM'].update(disabled=False) #desabilita o combo

			stop_thread = True #faz com que a Thread q le a serial saia do loop e termine
			#t0.kill()

			#desconecta da COM
			portaCOM.close() #comando dando erro


	elif events == 'logCar1AutoScroll': #janela e log carro1

		if values['logCar1AutoScroll'] == True:
			layout['logCar1'].update(autoscroll=True)
			#print('on')
		else:
			layout['logCar1'].update(autoscroll=False)
			#print('off')

	elif events == 'logCar2AutoScroll': #janela e log carro2
		if values['logCar2AutoScroll'] == True:
			layout['logCar2'].update(autoscroll=True)
		else:
			layout['logCar2'].update(autoscroll=False)


	elif events == 'setValoresCarro1':
		#print('ok')
		combo = values['setValoresCarro1']  # use the combo key
		#print(values['setValoresCarro1'])

		if values['setValoresCarro1'] == 'Ajuste' :
			layout['valorCarro11'].update(disabled=True) #habilita o input
			layout['valorCarro12'].update(disabled=True) #desabilita o input
			layout['valorCarro13'].update(disabled=True) #habilita o input

			layout['valorCarro11']('')
			layout['valorCarro12']('')
			layout['valorCarro13']('')

		if values['setValoresCarro1'] == 'Velocidade Ambos' or values['setValoresCarro1'] == 'Velocidade Esq' or values['setValoresCarro1'] == 'Velocidade Dir' or values['setValoresCarro1'] == 'PWM Ambos'  or values['setValoresCarro1'] == 'Angulo Direção':
			layout['valorCarro11'].update(disabled=False) #habilita o inpu
			layout['valorCarro12'].update(disabled=True) #desabilita o input
			layout['valorCarro13'].update(disabled=True) #habilita o input

			layout['valorCarro11']('54')

			if values['setValoresCarro1'] == 'Angulo Direção':
				layout['valorCarro11']('01')

			layout['valorCarro12']('')
			layout['valorCarro13']('')

		if values['setValoresCarro1'] == 'PID Esquerda' or values['setValoresCarro1'] == 'PID Direita' or values['setValoresCarro1'] == 'PID Direção':
			layout['valorCarro11'].update(disabled=False) #habilita o input
			layout['valorCarro12'].update(disabled=False) #desabilita o input
			layout['valorCarro13'].update(disabled=False) #habilita o input

			layout['valorCarro11']('')
			layout['valorCarro12']('')
			layout['valorCarro13']('')

	# envia dados via serial
	if events == 'botaoAjustaCarro1': #envia dados via serial

		macCarroDestino= layout['idCarro1'].get().replace(":", "")
		msgPlatoon = "0000"
		
		dataToSend = ""
		flagErro = False #flag q indica que algum caracter não é válido (não é HEX)

		if (values['setValoresCarro1'] == 'Velocidade Esq') and (layout['valorCarro11'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0052" + "000000000001" + layout['valorCarro11'].get()
			#print('send: ' + dataToSend)

		
		if (values['setValoresCarro1'] == 'Velocidade Dir') and (layout['valorCarro11'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0054" + "000000000001" + layout['valorCarro11'].get()
			#print('send: ' + dataToSend)
		

		if (values['setValoresCarro1'] == 'Velocidade Ambos' ) and (layout['valorCarro11'].get() != "") and (len(layout['valorCarro11'].get()) == 2):		
			dataToSend = macCarroDestino + msgPlatoon + "0056" + "0000000001" + layout['valorCarro11'].get() + "01" + layout['valorCarro11'].get()
			#layout['valorCarro2']('') #limpa o campo



		if (values['setValoresCarro1'] == 'PWM Ambos') and (layout['valorCarro11'].get() != "") and (len(layout['valorCarro11'].get()) == 2):
			dataToSend = macCarroDestino + msgPlatoon + "005C" + "0000000001" + layout['valorCarro11'].get() + "01" + layout['valorCarro11'].get()
			#print('send: ' + dataToSend)


		if (values['setValoresCarro1'] == 'Angulo Direção') and (layout['valorCarro11'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0082" + "00000000000000" + layout['valorCarro11'].get()
			#print('send: ' + dataToSend)	

		"""
		if (values['setValoresCarro1'] == 'PID Esquerda') and (layout['valorCarro11'].get() != "" and layout['valorCarro12'].get() != "" and layout['valorCarro13'].get() != "") and (len(layout['valorCarro11'].get()) == 2 and len(layout['valorCarro12'].get()) == 2 and len(layout['valorCarro13'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA13'+ pidDirecao.SetTunings(0.5, 2, 0.01); layout['valorCarro21'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()	
			#print('send: ' + dataToSend)


		if (values['setValoresCarro1'] == 'PID Direita') and (layout['valorCarro11'].get() != "" and layout['valorCarro12'].get() != "" and layout['valorCarro13'].get() != "") and (len(layout['valorCarro11'].get()) == 2 and len(layout['valorCarro12'].get()) == 2 and len(layout['valorCarro13'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA15'+ layout['valorCarro11'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()		
			#print('send: ' + dataToSend)


		if (values['setValoresCarro1'] == 'PID Direção') and (layout['valorCarro11'].get() != "" and layout['valorCarro12'].get() != "" and layout['valorCarro13'].get() != "") and (len(layout['valorCarro11'].get()) == 2 and len(layout['valorCarro12'].get()) == 2 and len(layout['valorCarro13'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA17'+ layout['valorCarro11'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()		
			#print('send: ' + dataToSend)	
		"""

		
		#print('send: ' + dataToSend)

		#array.pop(index) #remove um item do array
		for i in dataToSend:
			if (i < chr(48) or i > chr(57)) and (i < chr(65) or i > chr(70)) and (i < chr(97) or i > chr(102)): #limita os caracteres [0,9], [a,f], [A,F]
				#se houver algum caracter q nao seja HEX marca o caracter 
				flagErro = True #indica que ha um erro na msg q vai ser enviada via serial
				layout['labelValorCarro1'].Update('ERRO HEX', text_color = 'red')
			#else:
				layout['labelValorCarro1'].Update('HEX', text_color = 'white')




		#print(dataToSend)
		#dataToSend = dataToSend.replace(" ", "") #remove todos os espaços
		#print(dataToSend)
		flagErro == False

		# se todo os caracteres são HEX envia a msg
		if flagErro == False:
			layout['labelValorCarro1'].Update('HEX', text_color = 'white')
			#print('send: ' + dataToSend) 
			command = bytearray.fromhex(dataToSend)
			portaCOM.write(command)

			commandCarro1 = command
			flagComandoCarro1 = True


			#command = b'\x00\x02\xAA\xAA\x11\x01\x50\x00\x50\x00'
			#command = b'\x00\x02\xAA\xAA\x11\x01\x00\x00\x00\x00'
			#portaCOM.write(command)
		else:
			layout['labelValorCarro1'].Update('ERRO HEX', text_color = 'red')


	###########################################################################

	# envia dados via serial
	elif events == 'setValoresCarro2':
		#print('ok')
		combo = values['setValoresCarro2']  # use the combo key
		#print(values['setValoresCarro2'])

		if values['setValoresCarro2'] == 'Ajuste' :
			layout['valorCarro21'].update(disabled=True) #habilita o input
			layout['valorCarro22'].update(disabled=True) #desabilita o input
			layout['valorCarro23'].update(disabled=True) #habilita o input

			layout['valorCarro21']('')
			layout['valorCarro22']('')
			layout['valorCarro23']('')

		if values['setValoresCarro2'] == 'Velocidade Ambos' or values['setValoresCarro2'] == 'Velocidade Esq' or values['setValoresCarro2'] == 'Velocidade Dir' or values['setValoresCarro2'] == 'PWM Ambos'  or values['setValoresCarro2'] == 'Angulo Direção':
			layout['valorCarro21'].update(disabled=False) #habilita o inpu
			layout['valorCarro22'].update(disabled=True) #desabilita o input
			layout['valorCarro23'].update(disabled=True) #habilita o input

			layout['valorCarro21']('54')
			
			if values['setValoresCarro2'] == 'Angulo Direção':
				layout['valorCarro21']('01')

			layout['valorCarro22']('')
			layout['valorCarro23']('')

		if values['setValoresCarro2'] == 'PID Esquerda' or values['setValoresCarro2'] == 'PID Direita' or values['setValoresCarro1'] == 'PID Direção':
			layout['valorCarro21'].update(disabled=False) #habilita o input
			layout['valorCarro22'].update(disabled=False) #desabilita o input
			layout['valorCarro23'].update(disabled=False) #habilita o input

			layout['valorCarro21']('')
			layout['valorCarro22']('')
			layout['valorCarro23']('')

	# envia dados via serial
	if events == 'botaoAjustaCarro2': #envia dados via serial

		macCarroDestino= layout['idCarro2'].get().replace(":", "")
		msgPlatoon = "0000"
		
		dataToSend = ""
		flagErro = False #flag q indica que algum caracter não é válido (não é HEX)

		if (values['setValoresCarro2'] == 'Velocidade Esq') and (layout['valorCarro21'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0052" + "000000000001" + layout['valorCarro21'].get()
			#print('send: ' + dataToSend)

		
		if (values['setValoresCarro2'] == 'Velocidade Dir') and (layout['valorCarro21'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0054" + "000000000001" + layout['valorCarro21'].get()
			#print('send: ' + dataToSend)
		

		if (values['setValoresCarro2'] == 'Velocidade Ambos' ) and (layout['valorCarro21'].get() != "") and (len(layout['valorCarro21'].get()) == 2):		
			dataToSend = macCarroDestino + msgPlatoon + "0056" + "0000000001" + layout['valorCarro21'].get() + "01" + layout['valorCarro21'].get()
			#layout['valorCarro2']('') #limpa o campo



		if (values['setValoresCarro2'] == 'PWM Ambos') and (layout['valorCarro21'].get() != "") and (len(layout['valorCarro21'].get()) == 2):
			dataToSend = macCarroDestino + msgPlatoon + "005C" + "0000000001" + layout['valorCarro21'].get() + "01" + layout['valorCarro21'].get()
			#print('send: ' + dataToSend)


		if (values['setValoresCarro2'] == 'Angulo Direção') and (layout['valorCarro21'].get() != ""):
			dataToSend = macCarroDestino + msgPlatoon + "0082" + "00000000000000" + layout['valorCarro21'].get()
			#print('send: ' + dataToSend)	


		"""
		if (values['setValoresCarro2'] == 'PID Esquerda') and (layout['valorCarro21'].get() != "" and layout['valorCarro22'].get() != "" and layout['valorCarro23'].get() != "") and (len(layout['valorCarro21'].get()) == 2 and len(layout['valorCarro22'].get()) == 2 and len(layout['valorCarro23'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA13'+ pidDirecao.SetTunings(0.5, 2, 0.01); layout['valorCarro21'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()	
			#print('send: ' + dataToSend)


		if (values['setValoresCarro2'] == 'PID Direita') and (layout['valorCarro21'].get() != "" and layout['valorCarro22'].get() != "" and layout['valorCarro23'].get() != "") and (len(layout['valorCarro21'].get()) == 2 and len(layout['valorCarro22'].get()) == 2 and len(layout['valorCarro23'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA15'+ layout['valorCarro21'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()		
			#print('send: ' + dataToSend)


		if (values['setValoresCarro2'] == 'PID Direção') and (layout['valorCarro21'].get() != "" and layout['valorCarro22'].get() != "" and layout['valorCarro23'].get() != "") and (len(layout['valorCarro21'].get()) == 2 and len(layout['valorCarro22'].get()) == 2 and len(layout['valorCarro23'].get()) == 2):
			dataToSend = '000' + macCarroDestino + 'AAAA17'+ layout['valorCarro21'].get() + layout['valorCarro22'].get() + layout['valorCarro23'].get()		
			#print('send: ' + dataToSend)	
		"""
		
		
		#array.pop(index) #remove um item do array
		for i in dataToSend:
			if (i < chr(48) or i > chr(57)) and (i < chr(65) or i > chr(70)) and (i < chr(97) or i > chr(102)): #limita os caracteres [0,9], [a,f], [A,F]
				#se houver algum caracter q nao seja HEX marca o caracter 
				flagErro = True #indica que ha um erro na msg q vai ser enviada via serial
				layout['labelValorCarro2'].Update('ERRO HEX', text_color = 'red')
			#else:
				layout['labelValorCarro2'].Update('HEX', text_color = 'white')


		#print('send: ' + dataToSend)


		#print(dataToSend)
		#dataToSend = dataToSend.replace(" ", "") #remove todos os espaços
		#print(dataToSend)
		flagErro == False

		# se todo os caracteres são HEX envia a msg
		if flagErro == False:
			layout['labelValorCarro2'].Update('HEX', text_color = 'white')
			#print('send: ' + dataToSend) 
			command = bytearray.fromhex(dataToSend)
			portaCOM.write(command)

			commandCarro2 = command
			flagComandoCarro2 = True

			#command = b'\x00\x02\xAA\xAA\x11\x01\x50\x00\x50\x00'
			#command = b'\x00\x02\xAA\xAA\x11\x01\x00\x00\x00\x00'
			#portaCOM.write(command)
		else:
			layout['labelValorCarro2'].Update('ERRO HEX', text_color = 'red')	




	# pega a hora do sistema (vai ser usado na janela de log)
	now = datetime.now()
	time = "["+now.strftime("%H:%M:%S")+"]"	


    ###########################################################################

	"""
	if events == 'switch_button_conectar':
		if flag_switch_button_conectar == False:
			flag_switch_button_conectar = True
			layout['switch_button_conectar'].update(image_data = toggle_btn_on)
			layout['switch_button_conectar_texto'].Update('Carros Conectados', text_color='yellow')
		else:
			flag_switch_button_conectar = False
			layout['switch_button_conectar'].update(image_data = toggle_btn_off) 
			layout['switch_button_conectar_texto'].Update('Conectar Carros', text_color='white')
	"""
	#window.Close()			