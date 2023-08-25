# Carcara

Implementação da Orin embarcada no carrinho

## ROS files

Arquivos escritos na arquitetura ROS que atuam no sistema

(foto do ros graph)

### deciosion_makerFSM

### platoon_modeling

### bridge_vector
Interface entre o sistema ROS e a Vector Bus. Sua implementação foi visada para facilitar a leitura e escrita de dados entre todos os nós para o barramento CAN.

```py
class Bridge()
```

- Objeto que representa a classe que realiza a interface.

```py
pubCANMessage(self, can_message)
```

- Publica a mensagem ROS provinda da CAN.

```py
callBackDataFromOrin(self, orin_message)
```

- Callback de dados provindo do ROS

```py
getDataFromOrin(self) -> list
```

- Retorna o dado obtido pelo _callback_

```py
getFlagReceiveMessage(self) -> bool
```

- Retorna a flag de recebimento da mensagem provinda da comunicação ROS

```py
setFlagReceiveMessage(self, value: bool)
```

- Define a flag de recebimento da mensagem provinda da comunicação ROS

#### Obs: ESTE PROJETO FOI PENSADO NO PARADIGMA DE ORIENTAÇÃO À OBJETOS, LOGO COM BASE EM BOAS PRÁTICAS JUSTIFICA-SE A UTILIZAÇÃO DE MÉTODOS GETS E SETS.

## Python files

Arquivos que não estão no formato ROS mas são chamados por algum nó, assim atuam no sistema

### vector


### dsu