Este repositório contém pacotes do ROS que implementam os algoritmos de "Curve Following", "Attractive/Repulsive Potentials", "Tangent Bug" e "Wave-front".

Para utilizá-los é preciso:
1- instalar/copiar o pacote (catkin_ws/src/) para seu workspace do ROS.
2- dar source no setup file do seu catkin_ws (lembre sempre de dar este source antes de executar quaisquer um dos pontos seguintes)
3- abrir um terminal e rodar "roscore"
4- inicializar o stage com o .world do algoritmo desejado (catkin_ws/src//worlds/<.world file>):
  rosrun stage_ros stageros -d ./catkin_ws/src//worlds/<.world file>
5- rodar o script do algoritmo desejado
  python3 catkin_ws/src//scripts/<algorithm .py script>

Os resultados serão salvos em um .csv file.

OBS: este projeto foi desenvolvido utilizando ROS Noetic, mas não acredito que haja incompatibilidade com outras versões do ROS. Python3 foi utilizado para o desenvolvimento, pode ser que hajam incompatibilidades caso use Python.
