### Execute todos os scripts no diretório pai para que os mapas e bibliotecas criadas sejam importadas corretamente!

Antes de tudo, verifique se o seu ambiente possui todas as bibliotecas listadas no arquivo requirements.txt

Vídeo da apresentação do trabalho [aqui](https://youtu.be/rIwonSRZGuU).

- Campos potenciais
1) Abra a cena scenes/cave_scene.ttt no CoppeliaSim e inicie a simulação
2) Execute o script 
	python .\scripts\potential_fields.py
3) Feche o mapa que abrir
4) Insira as coordenadas desejadas para o alvo
5) A simulação irá rodar sozinha

Para rodar a cena paredes, carregue o arquivo scenes/paredes_scene.ttt CoppeliaSim e inicie a simulação. Troque a flag da linha 10 de:
	CAVE = True
Para:
	CAVE = False

- Roadmap
1) Abra a cena scenes/square_maze_scene.ttt e inicie a simulação
2) Execute o script 
	python .\scripts\roadmap.py
3) Feche o mapa que abrir
4) Insira as coordenadas desejadas para o alvo
5) A simulação irá rodar sozinha

Para rodar a cena paredes, carregue o arquivo scenes/circular_maze_scene.ttt CoppeliaSim e inicie a simulação. Troque a flag da linha 11 de:
	SQUARE = True
Para:
	SQUARE = False

- RRT
1) Abra a cena scenes/my_map_scene.ttt e inicie a simulação
2) Execute o script 
	python .\scripts\rrt.py
3) Insira as coordenadas desejadas para o alvo
4) Feche o plot quando o planejador encontrar um caminho até o alvo
4) A simulação irá rodar sozinha
