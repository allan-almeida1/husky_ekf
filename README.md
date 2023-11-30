# Localização do Robô Husky UGV utilizando o filtro de Kalman Estendido

Este projeto foi realizado na disciplina de Introdução à Robótica Móvel do curso de pós-graduação em Engenharia Elétrica da Universidade Federal da Bahia (UFBA). O objetivo do projeto é a implementação de um filtro de Kalman Estendido para a localização de um robô móvel Husky UGV em um ambiente simulado no Gazebo utilizando dados de odometria e a IMU do robô.

## Dependências

O projeto foi desenvolvido utilizando o ROS Noetic em um sistema operacional Ubuntu 20.04. Além do ROS e do Gazebo, o projeto depende das seguintes bibliotecas:

- SDL2
- Eigen3
- SDL2-ttf

Para instalar as dependências, execute o seguinte comando:

```bash
sudo apt install libsdl2-dev libsdl2-ttf-dev libeigen3-dev
```

## TL;DR

Para executar o projeto, clone o repositório dentro do diretório `src` do seu workspace do ROS e execute os seguintes comandos:

1. `catkin_make` ou `catkin build`
2. `source devel/setup.bash`
3. `roslaunch husky_ekf gazebo.launch`
4. Em outro terminal, execute `rosrun husky_ekf plot`
5. Em outro terminal, execute `rosrun husky_ekf ekf_node`
6. Em outro terminal, execute `rosrun husky_ekf dead_reckoning`

## Launch Files

O launch file disponível é o `gazebo.launch` que apenas abre o Gazebo e spawna o Husky. Para executar o launch file, utilize o seguinte comando:

```bash
roslaunch husky_ekf gazebo.launch
```

## Nós

O projeto é composto por 3 nós:

- **plot**: Nó responsável por executar a interface gráfica do projeto, exibindo o mapa do ambiente e a posição do robô. Para executar o nó, utilize o seguinte comando:

```bash
rosrun husky_ekf plot
```

- **dead_reckoning**: Nó responsável por enviar os comandos de movimentação do robô e realizar o dead reckoning usando apenas dados de odometria, sem utilizar o filtro de Kalman. Para executar o nó, utilize o seguinte comando:

```bash
rosrun husky_ekf dead_reckoning
```

- **ekf_node**: Nó responsável por executar o filtro de Kalman Estendido, utilizando dados de odometria e da IMU do robô. Para executar o nó, utilize o seguinte comando:

```bash
rosrun husky_ekf ekf_node
```

**Nota:** Os nós devem ser executados na ordem a seguir:

1. `plot`
2. `ekf_node`
3. `dead_reckoning`