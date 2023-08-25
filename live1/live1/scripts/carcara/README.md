# Carcara

Implementação da Orin embarcada no carrinho

## ROS files

Arquivos escritos na arquitetura ROS que atuam no sistema

(foto do ros graph)

### deciosion_makerFSM
Arquivo principal do projeto. Contêm estruturas de controle e decisão lógicos para o funcionamento do projeto.

### platoon_modeling
Modela o Platoon com base numa estrutura de dados DSU/Union-find, assim obtem noção do contexto de todos os carros do conjunto universo(todos os carros do sistema). Para meior entendimento do algoritmo leia: https://cp-algorithms.com/data_structures/disjoint_set_union.html. Além desse ponto é enviado para o nó de decisão o pai/líder do carro pre-setado(o ID de cada carro é definido pelo projetista).

```py
class NodeModeling()
```

- Objeto de representação do Platoon para todos os carros.

```py
def pubMsgLeader(self, msg_leader)
```

- Publica o líder do carro.

```py
def callback_logger(self, can_message)
```

- Callback que recebe as mensagens de ação/primitivas da CAN. Foi implementado visando a função do projeto _Central Platoon_, por isso que as primitivas são dados obtidos da CAN.

```py
def getCANMessage(self)
```
- Retorna a mensagem da CAN.

```py
def getFlagLogger(self) -> bool
```
- Retorna a flag do _logger_.


```py
def setFlagLogger(self, value: bool)
```
- Define a flag do _logger_.



### bridge_vector
Interface entre o sistema ROS e a Vector Bus. Sua implementação foi visada para facilitar a leitura e escrita de dados entre todos os nós para o barramento CAN.

```py
class Bridge()
```

- Objeto que representa a classe que realiza a interface.

```py
def pubCANMessage(self, can_message)
```

- Publica a mensagem ROS provinda da CAN.

```py
def callBackDataFromOrin(self, orin_message)
```

- Callback de dados provindo do ROS

```py
def getDataFromOrin(self) -> list
```

- Retorna o dado obtido pelo _callback_

```py
def getFlagReceiveMessage(self) -> bool
```

- Retorna a flag de recebimento da mensagem provinda da comunicação ROS

```py
def setFlagReceiveMessage(self, value: bool)
```

- Define a flag de recebimento da mensagem provinda da comunicação ROS

#### Obs: ESTE PROJETO FOI PENSADO NO PARADIGMA DE ORIENTAÇÃO À OBJETOS, LOGO COM BASE EM BOAS PRÁTICAS JUSTIFICA-SE A UTILIZAÇÃO DE MÉTODOS GETS E SETS.

## Python files

Arquivos que não estão no formato ROS mas são chamados por algum nó, assim atuam no sistema

### vector

Realiza a interface com a Vector Bus abrindo um servidor Socket/TCP.

```py
def logCan(s)
```

- Retorna uma lista com os parametros obtidos da CAN e imprime no terminal a mensagem mapeada.

### dsu

Arquivo que implementa a estrutura de dados DSU/Union-find. Para meior entendimento do algoritmo leia: https://cp-algorithms.com/data_structures/disjoint_set_union.html.

```py
class Car:
    def __init__(self, idx, X, GAP):
        self.idx = idx
        self.X = X
        self.GAP = GAP
```

- Define um Objeto carro, analogo à uma _struct_ em C, este objeto contêm o ID do carro(idx), sua posição na pista(X) e o GAP(GAP).

```py
def dsBuild(self)
```

- Setup inicial da estrutura de dados.

```py
def dsFind(self, car: int) -> int
```

- Recebe como argumento o indice de um carro e retorna o seu pai/lider

```py
def dsUnion(self, car_u: Car, car_v: Car)
```

- Realiza a união/merge entre dois carros. O critério de avaliação para união é dado pelo carro mais distante na pista.

```py
def dsLeave(self, car_u : Car)
```

- Retira o lider de um carro na representação e volta para o seu estado inicial, logo, o carro é lider de si mesmo.

```py
def getSize(self, u: int) -> int
```

- Retorna o tamanho do Platoon de qualquer carro. Obtem-se este dado a parti da _Relação_(conceito matemático) de Pai de cada carro
